#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include "ui.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// Thông tin Access Point
const char* ssid = "ESP32_AP";
const char* password = "12345678";

// Cấu hình Server
const uint16_t port = 12345;
WiFiServer server(port);
WiFiClient clients[5];

// Enum định nghĩa các xe
enum CarID {
  CAR1, CAR2, CAR3, CAR4, CAR5, ALL_CARS = 0xFF
};
const int maxClients = 5;


// Struct lưu cấu hình IP cho từng xe
struct CarConfig {
  CarID id;
  IPAddress ip;
  bool idConfirmed; // Thêm để theo dõi trạng thái xác nhận carId
};

CarConfig carConfigs[maxClients] = {
  {CAR1, IPAddress(192, 168, 4, 2)},
  {CAR2, IPAddress(192, 168, 4, 3)},
  {CAR3, IPAddress(192, 168, 4, 4)},
  {CAR4, IPAddress(192, 168, 4, 5)},
  {CAR5, IPAddress(192, 168, 4, 6)}
};

// Hàng đợi để lưu dữ liệu từ client
QueueHandle_t dataQueue;
QueueHandle_t commandQueue;
// Queue để lưu STATUS_REPORT từ client
QueueHandle_t statusQueue;
SemaphoreHandle_t clientMutex;
Preferences preferences; //lưu giá trị vào flash nvs


// Command Types
enum CommandType {
  ON_OFF_ALL    = 0x01,
  SPEED_ALL     = 0x02,
  ON_OFF_CAR    = 0x03,
  SPEED_CAR     = 0x04,
  MODE          = 0x05,
  GPIO_CONTROL  = 0x06,
  STATUS_REPORT = 0x07,
  BAT_REPORT    = 0x08
};

// Struct cho dữ liệu lệnh
struct CommandData {
  CommandType cmd;
  CarID carId;
  uint8_t payload[6];
  uint8_t payloadLen;
};

// Trạng thái xe
struct CarStatus {
  bool isOn;
  uint8_t speed;
  uint8_t mode;
  uint8_t gpio[6];
  bool all_on;
  uint8_t speed_all;
  uint8_t battery_level;
};

// Lưu trạng thái cho từng xe
CarStatus carStatuses[maxClients] = {
  {false, 0, 0, {0, 0, 0, 0, 0, 0}, true, 100, 0},
  {false, 0, 0, {0, 0, 0, 0, 0, 0}, true, 100, 0},
  {false, 0, 0, {0, 0, 0, 0, 0, 0}, true, 100, 0},
  {false, 0, 0, {0, 0, 0, 0, 0, 0}, true, 100, 0},
  {false, 0, 0, {0, 0, 0, 0, 0, 0}, true, 100, 0}
};
//biến theo dõi trạng thái kết nối
bool carOnlineStatus[maxClients] = {false, false, false, false, false};
// Biến đếm số lần không phản hồi BAT_REPORT
uint8_t missedBatteryResponses[maxClients] = {0, 0, 0, 0, 0};

// Lưu trạng thái xe vào NVS
void saveCarStatus(CarID carId) {
  preferences.begin("car_status", false);
  String key = "car" + String(carId);
  preferences.putBool((key + "_on").c_str(), carStatuses[carId].isOn);
  preferences.putUChar((key + "_speed").c_str(), carStatuses[carId].speed);
  preferences.putUChar((key + "_mode").c_str(), carStatuses[carId].mode);
  preferences.putUChar((key + "_battery").c_str(), carStatuses[carId].battery_level);
  for (int i = 0; i < 6; i++) {
    preferences.putUChar((key + "_gpio" + String(i)).c_str(), carStatuses[carId].gpio[i]);
  }
  preferences.end();
  Serial.printf("Car %d status saved to NVS\n", carId + 1);
}

// Khôi phục trạng thái xe từ NVS
void restoreCarStatus() {
  preferences.begin("car_status", true);
  for (int i = 0; i < 5; i++) {
    String key = "car" + String(i);
    carStatuses[i].isOn = preferences.getBool((key + "_on").c_str(), false);
    carStatuses[i].speed = preferences.getUChar((key + "_speed").c_str(), 0);
    carStatuses[i].mode = preferences.getUChar((key + "_mode").c_str(), 1);
    for (int j = 0; j < 6; j++) {
      carStatuses[i].gpio[j] = preferences.getUChar((key + "_gpio" + String(j)).c_str(), 0);
    }
  }
  preferences.end();
  Serial.println("Car statuses restored from NVS");
}

// Tạo và gửi frame
void sendCommand(WiFiClient &client, CommandType cmd, CarID carId, uint8_t *payload, uint8_t payloadLen) {
  if (!client.connected()) {
    Serial.printf("Client not connected for command 0x%02X (IP=%s)\n", cmd, client.remoteIP().toString().c_str());
    return;
  }
  uint8_t frame[32];
  frame[0] = 0xAA;
  frame[1] = 0x01;
  frame[2] = cmd;
  frame[3] = carId;
  frame[4] = payloadLen;
  memcpy(frame + 5, payload, payloadLen);
  uint8_t checksum = 0;
  for (int i = 0; i < 5 + payloadLen; i++) checksum += frame[i];
  frame[5 + payloadLen] = checksum;
  frame[6 + payloadLen] = 0xFF;
  Serial.printf("Sending to client IP=%s, Type=0x%02X, CarID=%d, Len=%d\n", 
                client.remoteIP().toString().c_str(), cmd, carId, payloadLen);
  Serial.print("Packet: ");
  for (int i = 0; i < 7 + payloadLen; i++) {
    if (frame[i] < 0x10) Serial.print("0");
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  if (client.write(frame, 7 + payloadLen) != 7 + payloadLen) {
    Serial.println("Failed to send command to client!");
  }
  client.flush();
}

// Gửi lệnh tới tất cả client
void sendCommandToAll(CommandType cmd, uint8_t* payload, uint8_t payloadLen) {
  if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    Serial.println("sendCommandToAll: Cannot take semaphore!");
    return;
  }
  int sentCount = 0;
  for (int i = 0; i < maxClients; i++) {
    if (clients[i].connected() && carConfigs[i].idConfirmed) {
      int retries = 5;
      bool sent = false;
      while (retries > 0 && !sent) {
        // Xóa buffer client trước khi gửi
        while (clients[i].available()) {
          Serial.printf("Clearing stray byte from Client%d: 0x%02X\n", i, clients[i].read());
        }
        sendCommand(clients[i], cmd, ALL_CARS, payload, payloadLen);
        clients[i].setTimeout(1500); // Tăng timeout
        char response[3] = {0};
        if (clients[i].readBytes((uint8_t*)response, 2) == 2 && strcmp(response, "OK") == 0) {
          sent = true;
          Serial.printf("Received OK from Client%d (Car%d)\n", i, carConfigs[i].id + 1);
        } else {
          Serial.printf("No OK from Client%d (Car%d), retrying (%d left)\n", i, carConfigs[i].id + 1, retries - 1);
          retries--;
          vTaskDelay(200 / portTICK_PERIOD_MS);
        }
      }
      if (!sent) {
        Serial.printf("Failed to send command 0x%02X to Client%d (Car%d) after retries\n", cmd, i, carConfigs[i].id + 1);
        carOnlineStatus[carConfigs[i].id] = false;
        updateStatusArea();
      } else {
        sentCount++;
      }
    }
  }
  xSemaphoreGive(clientMutex);
  Serial.printf("Sent command 0x%02X to %d clients\n", cmd, sentCount);
}

// Tìm CarID dựa trên IP
CarID getCarIdByIP(IPAddress ip) {
  for (int i = 0; i < maxClients; i++) {
    if (carConfigs[i].ip == ip && carConfigs[i].idConfirmed) {
      return carConfigs[i].id;
    }
  }
  Serial.printf("No confirmed carId for IP %s\n", ip.toString().c_str());
  return CAR1; // Mặc định, sẽ được cập nhật sau khi nhận STATUS_REPORT
}

// Kiểm tra trạng thái client
void checkClientStatus() {
  if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    Serial.println("Cannot take semaphore for checkClientStatus");
    return;
  }
  for (int i = 0; i < maxClients; i++) {
    if (clients[i].connected() && !clients[i].availableForWrite()) {
      CarID carId = carConfigs[i].id;
      carOnlineStatus[carId] = false;
      carStatuses[carId].isOn = false;
      saveCarStatus(carId);
      clients[i].stop();
      carConfigs[i].idConfirmed = false;
      Serial.printf("Car%d: Disconnected, set OFFLINE\n", carId + 1);
      updateStatusArea();
    }
  }
  xSemaphoreGive(clientMutex);
}

// Task xử lý client
// Task xử lý client
void clientTask(void *pvParameters) {
  int clientId = (int)pvParameters;
  // Mảng các label pin cho từng xe
  lv_obj_t *batteryLabels[] = {
    ui_Car1Batlabel, // Car1
    ui_Car2Batlabel, // Car2
    ui_Car3Batlabel, // Car3
    ui_Car4Batlabel, // Car4
    ui_Car5Batlabel  // Car5
  };

  while (1) {
    if (commandQueue == NULL) {
      Serial.printf("ClientTask%d: commandQueue is NULL, waiting...\n", clientId);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    CommandData cmdData;
    if (xQueueReceive(commandQueue, &cmdData, 0)) {
      if (cmdData.carId == ALL_CARS) {
        sendCommandToAll(cmdData.cmd, cmdData.payload, cmdData.payloadLen);
      } else {
        if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
          Serial.println("ClientTask: Cannot take semaphore for command");
          vTaskDelay(300 / portTICK_PERIOD_MS);
          continue;
        }
        for (int i = 0; i < maxClients; i++) {
          if (clients[i].connected() && carConfigs[i].id == cmdData.carId) {
            sendCommand(clients[i], cmdData.cmd, cmdData.carId, cmdData.payload, cmdData.payloadLen);
            Serial.printf("Sent command 0x%02X to Car%d\n", cmdData.cmd, cmdData.carId + 1);
            break;
          }
        }
        xSemaphoreGive(clientMutex);
      }
    }

    bool isConnected = false;
    if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
      isConnected = clients[clientId].connected();
      if (!isConnected) {
        CarID carId = carConfigs[clientId].id;
        carOnlineStatus[carId] = false;
        Serial.printf("Car%d disconnected\n", carId + 1);
        updateStatusArea();
        clients[clientId].stop();
        xSemaphoreGive(clientMutex);
        vTaskDelete(NULL);
      }
      xSemaphoreGive(clientMutex);
    }

    if (isConnected && xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
      if (clients[clientId].available() >= 7) {
        uint8_t buffer[32];
        clients[clientId].setTimeout(1500);
        int len = clients[clientId].readBytes(buffer, sizeof(buffer));
        xSemaphoreGive(clientMutex);
        if (len == 0) {
          Serial.printf("No data received from client %d\n", clientId);
          if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
            CarID carId = carConfigs[clientId].id;
            carOnlineStatus[carId] = false;
            clients[clientId].stop();
            updateStatusArea();
            xSemaphoreGive(clientMutex);
            vTaskDelete(NULL);
          }
          continue;
        }
        Serial.printf("Received from client %d: len=%d\n", clientId, len);
        Serial.print("Packet: ");
        for (int i = 0; i < len; i++) {
          if (buffer[i] < 0x10) Serial.print("0");
          Serial.print(buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        int frameStart = -1;
        for (int i = 0; i < len - 1; i++) {
          if (buffer[i] == 0xAA && buffer[i+1] == 0x01) {
            frameStart = i;
            break;
          }
        }
        if (frameStart == -1 || frameStart + 7 > len) {
          Serial.println("No valid frame found!");
          continue;
        }
        int frameLen = buffer[frameStart + 4] + 7;
        if (frameStart + frameLen > len || buffer[frameStart + frameLen - 1] != 0xFF) {
          Serial.println("Invalid packet header or footer!");
          continue;
        }
        uint8_t cmd = buffer[frameStart + 2];
        uint8_t carId = buffer[frameStart + 3];
        uint8_t dataLen = buffer[frameStart + 4];
        Serial.printf("Packet info: cmd=0x%02X, car_id=%d, dataLen=%d\n", cmd, carId, dataLen);
        if (frameLen >= 7 && frameLen <= len && carId <= CAR5) {
          uint8_t checksum = 0;
          for (int i = frameStart; i < frameStart + frameLen - 2; i++) checksum += buffer[i];
          if (checksum == buffer[frameStart + frameLen - 2]) {
            CarID id = (CarID)carId;
            if (cmd == STATUS_REPORT) {
              if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
                carConfigs[clientId].id = id;
                carConfigs[clientId].idConfirmed = true;
                xSemaphoreGive(clientMutex);
              }
              if (dataLen == 10) {
                carStatuses[id].isOn = buffer[frameStart + 5];
                carStatuses[id].speed = buffer[frameStart + 6];
                carStatuses[id].mode = buffer[frameStart + 7];
                memcpy(carStatuses[id].gpio, buffer + frameStart + 8, 6);
                carOnlineStatus[id] = true;
                missedBatteryResponses[clientId] = 0;
                saveCarStatus(id);
                char statusMsg[128];
                snprintf(statusMsg, sizeof(statusMsg), "Car%d: isOn=%d,speed=%d,mode=%d,gpio=%d,%d,%d,%d,%d,%d",
                        id + 1, carStatuses[id].isOn, carStatuses[id].speed, carStatuses[id].mode,
                        carStatuses[id].gpio[0], carStatuses[id].gpio[1], carStatuses[id].gpio[2],
                        carStatuses[id].gpio[3], carStatuses[id].gpio[4], carStatuses[id].gpio[5]);
                xQueueSend(statusQueue, statusMsg, pdMS_TO_TICKS(100));
                Serial.printf("Car%d: Status OK\n", id + 1);
              }
            } else if (cmd == BAT_REPORT && dataLen == 1) {
              carStatuses[id].battery_level = buffer[frameStart + 5];
              carOnlineStatus[id] = true;
              missedBatteryResponses[clientId] = 0;
              saveCarStatus(id);
              Serial.printf("Car%d: Battery=%d%%\n", id + 1, carStatuses[id].battery_level);
              // Cập nhật label pin tương ứng với carId
              if (id >= CAR1 && id <= CAR5 && batteryLabels[id]) {
                char label_text[32];
                snprintf(label_text, sizeof(label_text), "%d%%", carStatuses[id].battery_level);
                lvgl_port_lock(-1);
                lv_label_set_text(batteryLabels[id], label_text);
                lvgl_port_unlock();
                Serial.printf("Updated ui_Car%dBatlabel: %d%%\n", id + 1, carStatuses[id].battery_level);
              } else {
                Serial.printf("Error: No valid battery label for Car%d\n", id + 1);
              }
            }
            if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
              clients[clientId].write("OK", 2);
              clients[clientId].flush();
              xSemaphoreGive(clientMutex);
            }
          } else {
            Serial.println("Checksum error!");
          }
        } else {
          Serial.println("Invalid packet format or car ID!");
        }
      } else {
        xSemaphoreGive(clientMutex);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task server
void serverTask(void *pvParameters) {
  while (1) {
    WiFiClient newClient = server.accept();
    if (newClient) {
      if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
        Serial.println("Cannot take semaphore for new client");
        newClient.stop();
        vTaskDelay(300 / portTICK_PERIOD_MS);
        continue;
      }
      bool added = false;
      IPAddress newClientIP = newClient.remoteIP();
      for (int i = 0; i < maxClients; i++) {
        if (clients[i].connected() && clients[i].remoteIP() == newClientIP) {
          Serial.printf("Client %s already connected, rejecting\n", newClientIP.toString().c_str());
          newClient.stop();
          break;
        }
      }
      for (int i = 0; i < maxClients; i++) {
        if (!clients[i].connected()) {
          clients[i] = newClient;
          CarID carId = getCarIdByIP(newClientIP);
          carConfigs[i].id = carId;
          carConfigs[i].idConfirmed = false; // Chờ STATUS_REPORT để xác nhận
          carOnlineStatus[carId] = true;
          Serial.printf("New client: Car%d (%s), awaiting STATUS_REPORT\n", carId + 1, newClientIP.toString().c_str());
          updateStatusArea();
          char taskName[20];
          snprintf(taskName, sizeof(taskName), "ClientTask%d", i);
          if (xTaskCreate(clientTask, taskName, (1024*15), (void *)i, 1, NULL) != pdPASS) {
            Serial.printf("Failed to create ClientTask%d\n", i);
            clients[i].stop();
            carOnlineStatus[carId] = false;
            carConfigs[i].idConfirmed = false;
            updateStatusArea();
          } else {
            added = true;
          }
          break;
        }
      }
      if (!added) {
        Serial.println("Max clients reached!");
        newClient.stop();
      }
      xSemaphoreGive(clientMutex);
    }
    if (millis() % 5000 == 0) checkClientStatus();
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

// Task in dữ liệu và cập nhật LVGL
void printTask(void *pvParameters) {
  char buffer[128];
  while (1) {
    if (xQueueReceive(dataQueue, buffer, portMAX_DELAY)) {
      Serial.println(buffer);
      lvgl_port_lock(-1);
      // if (ui_Label_Car1 && strstr(buffer, "Car1")) lv_label_set_text(ui_Label_Car1, buffer);
      // else if (ui_Label_Car2 && strstr(buffer, "Car2")) lv_label_set_text(ui_Label_Car2, buffer);
      // else if (ui_Label_Car3 && strstr(buffer, "Car3")) lv_label_set_text(ui_Label_Car3, buffer);
      // else if (ui_Label_Car4 && strstr(buffer, "Car4")) lv_label_set_text(ui_Label_Car4, buffer);
      // else if (ui_Label_Car5 && strstr(buffer, "Car5")) lv_label_set_text(ui_Label_Car5, buffer);
      lvgl_port_unlock();
    }
  }
}
void updateStatusArea() {
  if (!ui_StatusArea) {
    Serial.println("Error: ui_StatusArea is NULL!");
    return;
  }
  char statusText[128];
  snprintf(statusText, sizeof(statusText), 
           "Car1: %s, Car2: %s, Car3: %s, Car4: %s, Car5: %s",
           carOnlineStatus[CAR1] ? "Online" : "Offline",
           carOnlineStatus[CAR2] ? "Online" : "Offline",
           carOnlineStatus[CAR3] ? "Online" : "Offline",
           carOnlineStatus[CAR4] ? "Online" : "Offline",
           carOnlineStatus[CAR5] ? "Online" : "Offline");
  lvgl_port_lock(-1);
  lv_textarea_set_text(ui_StatusArea, statusText);
  lvgl_port_unlock();
  Serial.printf("Updated ui_StatusArea: %s\n", statusText);
}

// task để gửi yêu cầu BAT_REPORT mỗi 10 giây
void batteryRequestTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
      int sentCount = 0;
      for (int i = 0; i < maxClients; i++) {
        if (clients[i].connected() && carConfigs[i].id <= CAR5 && carConfigs[i].idConfirmed) {
          uint8_t payload[1] = {0};
          sendCommand(clients[i], BAT_REPORT, carConfigs[i].id, payload, 0);
          missedBatteryResponses[i]++;
          Serial.printf("Sent BAT_REPORT to Car%d (IP=%s), missed count=%d\n", 
                        carConfigs[i].id + 1, clients[i].remoteIP().toString().c_str(), missedBatteryResponses[i]);
          if (missedBatteryResponses[i] >= 5) {
            CarID carId = carConfigs[i].id;
            carOnlineStatus[carId] = false;
            carStatuses[carId].isOn = false;
            clients[i].stop();
            carConfigs[i].idConfirmed = false;
            Serial.printf("Car%d missed BAT_REPORT 5 times, set OFFLINE\n", carId + 1);
            updateStatusArea();
          }
          sentCount++;
        } else {
          missedBatteryResponses[i] = 0;
        }
      }
      xSemaphoreGive(clientMutex);
      if (sentCount > 0) {
        Serial.printf("Sent BAT_REPORT request to %d clients\n", sentCount);
      } else {
        Serial.println("No clients with confirmed ID for BAT_REPORT request");
      }
    } else {
      Serial.println("Cannot take semaphore for batteryRequestTask");
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// Task cập nhật giao diện từ STATUS_REPORT
// Task cập nhật các đối tượng LVGL từ STATUS_REPORT
void statusUpdateTask(void *pvParameters) {
  char statusMsg[128];
  lv_obj_t *onOffButtons[] = {ui_OnOffCar1btn, ui_OnOffCar2btn, ui_OnOffCar3btn, ui_OnOffCar4btn, ui_OnOffCar5btn};
  lv_obj_t *speedDropdowns[] = {ui_SpeedCar1, ui_SpeedCar2, ui_SpeedCar3, ui_SpeedCar4, ui_SpeedCar5};
  lv_obj_t *modeDropdowns[] = {ui_ModeCar1, ui_ModeCar2, ui_ModeCar3, ui_ModeCar4, ui_ModeCar5};
  lv_obj_t *ioButtons[][6] = {
    {ui_Car1Io1btn, ui_Car1Io2btn, ui_Car1Io3btn, ui_Car1Io4btn, ui_Car1Io5btn, ui_Car1Io6btn},
    {ui_Car2Io1btn, ui_Car2Io2btn, ui_Car2Io3btn, ui_Car2Io4btn, ui_Car2Io5btn, ui_Car2Io6btn},
    {ui_Car3Io1btn, ui_Car3Io2btn, ui_Car3Io3btn, ui_Car3Io4btn, ui_Car3Io5btn, ui_Car3Io6btn},
    {ui_Car4Io1btn, ui_Car4Io2btn, ui_Car4Io3btn, ui_Car4Io4btn, ui_Car4Io5btn, ui_Car4Io6btn},
    {ui_Car5Io1btn, ui_Car5Io2btn, ui_Car5Io3btn, ui_Car5Io4btn, ui_Car5Io5btn, ui_Car5Io6btn}
  };
  while (1) {
    if (xQueueReceive(statusQueue, statusMsg, portMAX_DELAY)) {
      int carId, isOn, speed, mode, gpio[6];
      Serial.printf("Received statusMsg: %s\n", statusMsg);
      if (sscanf(statusMsg, "Car%d: isOn=%d,speed=%d,mode=%d,gpio=%d,%d,%d,%d,%d,%d",
                 &carId, &isOn, &speed, &mode, &gpio[0], &gpio[1], &gpio[2], &gpio[3], &gpio[4], &gpio[5]) == 10) {
        if (carId >= 1 && carId <= 5) {
          int id = carId - 1;
          lvgl_port_lock(-1);
          // Cập nhật switch OnOff
          if (onOffButtons[id]) {
            if (isOn) {
              lv_obj_add_state(onOffButtons[id], LV_STATE_CHECKED);
            } else {
              lv_obj_clear_state(onOffButtons[id], LV_STATE_CHECKED);
            }
            Serial.printf("Updated ui_OnOffCar%dbtn: %s\n", carId, isOn ? "ON" : "OFF");
          } else {
            Serial.printf("Error: ui_OnOffCar%dbtn is NULL\n", carId);
          }
          // Cập nhật dropdown Speed
          if (speedDropdowns[id]) {
            int index = (speed-5) / 10;
            if (index >= 0 && index <= 9) {
              lv_dropdown_set_selected(speedDropdowns[id], index);
              Serial.printf("Updated ui_SpeedCar%d: index=%d (%d%%)\n", carId, index, (index) * 10);
            } else {
              Serial.printf("Error: Invalid speed index %d for Car%d (speed=%d)\n", index, carId, speed);
            }
          } else {
            Serial.printf("Error: ui_SpeedCar%d is NULL\n", carId);
          }
          // Cập nhật dropdown Mode
          if (modeDropdowns[id]) {
            if (mode >= 0 && mode <= 1) {
              lv_dropdown_set_selected(modeDropdowns[id], mode); // mode=0 -> index=0 (Mode 1), mode=1 -> index=1 (Mode 2)
              Serial.printf("Updated ui_ModeCar%d: index=%d (Mode %d)\n", carId, mode, mode);
            } else {
              lv_dropdown_set_selected(modeDropdowns[id], 0); // Default Mode 1
              Serial.printf("Error: Invalid mode %d for Car%d, set to Mode 1\n", mode, carId);
            }
          } else {
            Serial.printf("Error: ui_ModeCar%d is NULL\n", carId);
          }
          // Cập nhật switch GPIO
          for (int i = 0; i < 6; i++) {
            if (ioButtons[id][i]) {
              if (gpio[i]) {
                lv_obj_add_state(ioButtons[id][i], LV_STATE_CHECKED);
              } else {
                lv_obj_clear_state(ioButtons[id][i], LV_STATE_CHECKED);
              }
              Serial.printf("Updated ui_Car%dIo%dbtn: %s\n", carId, i + 1, gpio[i] ? "ON" : "OFF");
            } else {
              Serial.printf("Error: ui_Car%dIo%dbtn is NULL\n", carId, i + 1);
            }
          }
          lvgl_port_unlock();
          Serial.printf("Updated UI for Car%d: isOn=%d, speed=%d, mode=%d, gpio=%d,%d,%d,%d,%d,%d\n",
                        carId, isOn, speed, mode, gpio[0], gpio[1], gpio[2], gpio[3], gpio[4], gpio[5]);
        } else {
          Serial.printf("Error: Invalid carId %d\n", carId);
        }
      } else {
        Serial.printf("Failed to parse status message: %s\n", statusMsg);
      }
    }
  }
}

// Task thiết lập Access Point
void wifiTask(void *pvParameters) {
  WiFi.softAP(ssid, password, 6);
  Serial.println("Access Point đã được tạo!");
  Serial.print("Địa chỉ IP của Server: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
  Serial.println("Server TCP đã khởi động trên cổng: " + String(port));
  vTaskDelete(NULL);
}

// Task xử lý LVGL
void lvglTask(void *pvParameters) {
  while (1) {
    lvgl_port_lock(-1);
    lv_timer_handler(); // Xử lý giao diện LVGL
    lvgl_port_unlock();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
    String title = "LVGL porting example";
    Serial.begin(115200);

    Serial.println("Initializing board");
    Board *board = new Board();
    board->init();

    #if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    // When avoid tearing function is enabled, the frame buffer number should be set in the board driver
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd->getBus();
    /**
     * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the
     * "bounce buffer" functionality to enhance the RGB data bandwidth.
     * This feature will consume `bounce_buffer_size * bytes_per_pixel * 2` of SRAM memory.
     */
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
    }
#endif
#endif
    assert(board->begin());

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    Serial.println("Creating UI");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */

  

    // Khởi tạo hàng đợi và semaphore
    dataQueue = xQueueCreate(30, 128);
    if (!dataQueue) {
      Serial.println("Failed to create dataQueue!");
      while (1);
    }
    commandQueue = xQueueCreate(30, sizeof(CommandData));
    clientMutex = xSemaphoreCreateMutex();
    if (!clientMutex) {
      Serial.println("Failed to create clientMutex!");
      while (1);
    }
    statusQueue = xQueueCreate(30, 128);
    if (!statusQueue) {
      Serial.println("Failed to create statusQueue!");
      while (1);
    }

    // Khởi tạo các task
    xTaskCreate(wifiTask, "WiFiTask", (1024*10), NULL, 2, NULL);
    xTaskCreate(serverTask, "ServerTask", (1024*30), NULL, 2, NULL);
    //xTaskCreate(printTask, "PrintTask", 16384, NULL, 2, NULL);

    if (xTaskCreate(batteryRequestTask, "BatteryRequestTask", (1024*15), NULL, 1, NULL) != pdPASS) {
      Serial.println("Failed to create BatteryRequestTask");
    }
    //xTaskCreate(lvglTask, "LVGLTask", 16384, NULL, 1, NULL);
    for (int i = 0; i < maxClients; i++) {
      xTaskCreate(clientTask, ("ClientTask" + String(i)).c_str(), (1024*15), (void *)i, 3, NULL);
    }

    Serial.printf("Setup complete. Free heap: %d bytes\n", ESP.getFreeHeap());

    // // Khởi tạo LVGL UI
    lvgl_port_lock(-1);

    /* Release the mutex */
    lvgl_port_unlock();
    ui_init();
    Serial.println(title + " end");

    if (ui_StatusArea) {
      lv_textarea_set_text(ui_StatusArea, "Car1: Offline, Car2: Offline, Car3: Offline, Car4: Offline, Car5: Offline");
      Serial.println("Initialized ui_StatusArea with default text");
    } else {
      Serial.println("Error: ui_StatusArea is NULL after ui_init!");
    }
    lv_dropdown_set_selected(ui_SpeedAllCardropdown, 10); // Chọn 100% (tùy chọn cuối)
    xTaskCreate(statusUpdateTask, "StatusUpdateTask", (1024*20), NULL, 2, NULL);
}

void loop()
{
  static uint32_t last = 0;
  if (millis() - last > 5000) {
    //Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    last = millis();
  }
}

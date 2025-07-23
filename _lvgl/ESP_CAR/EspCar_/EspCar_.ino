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
  {false, 0, 1, {0, 0, 0, 0, 0, 0}, true, 100, 0},
  {false, 0, 1, {0, 0, 0, 0, 0, 0}, true, 100, 0},
  {false, 0, 1, {0, 0, 0, 0, 0, 0}, true, 100, 0},
  {false, 0, 1, {0, 0, 0, 0, 0, 0}, true, 100, 0},
  {false, 0, 1, {0, 0, 0, 0, 0, 0}, true, 100, 0}
};
//biến theo dõi trạng thái kết nối
bool carOnlineStatus[maxClients] = {false, false, false, false, false};

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
}

// Gửi lệnh tới tất cả client
void sendCommandToAll(CommandType cmd, uint8_t* payload, uint8_t payloadLen) {
  TaskHandle_t holder = xSemaphoreGetMutexHolder(clientMutex);
  if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    Serial.printf("Cannot take semaphore for command 0x%02X, current task: %s, holder: %s\n", 
                  cmd, pcTaskGetName(NULL), holder ? pcTaskGetName(holder) : "None");
    return;
  }
  int sentCount = 0;
  for (int i = 0; i < maxClients; i++) {
    if (clients[i].connected()) {
      Serial.printf("Sending to client %d (%s)\n", i, clients[i].remoteIP().toString().c_str());
      sendCommand(clients[i], cmd, ALL_CARS, payload, payloadLen);
      sentCount++;
    } else {
      Serial.printf("Client %d not connected, skipping\n", i);
    }
  }
  xSemaphoreGive(clientMutex);
  if (sentCount == 0) {
    Serial.println("No clients connected, no commands sent");
  } else {
    Serial.printf("Sent command 0x%02X to %d clients\n", cmd, sentCount);
  }
}

// Tìm CarID dựa trên IP
CarID getCarIdByIP(IPAddress ip) {
  for (int i = 0; i < maxClients; i++) {
    if (carConfigs[i].ip == ip) {
      return carConfigs[i].id;
    }
  }
  return CAR1;
}

// Kiểm tra trạng thái client
void checkClientStatus() {
  if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    Serial.println("Cannot take semaphore for checkClientStatus");
    return;
  }
  for (int i = 0; i < maxClients; i++) {
    if (clients[i].connected()) {
      clients[i].setTimeout(100); // Timeout 100ms
      if (clients[i].write((uint8_t)0) == 0 || !clients[i].availableForWrite()) {
        CarID carId = getCarIdByIP(clients[i].remoteIP());
        carOnlineStatus[carId] = false;
        carStatuses[carId].isOn = false;
        saveCarStatus(carId);
        char msg[64];
        snprintf(msg, sizeof(msg), "Car%d disconnected.", carId + 1);
        Serial.println(msg);
        xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
        clients[i].stop();
      } else {
        Serial.printf("Client %d (IP=%s) still connected\n", i, clients[i].remoteIP().toString().c_str());
      }
    }
  }
  xSemaphoreGive(clientMutex);
}

// Task xử lý client
// Task xử lý client
void clientTask(void *pvParameters) {
  int clientId = (int)pvParameters;
  while (1) {
    // Kiểm tra commandQueue
    if (commandQueue == NULL) {
      Serial.printf("ClientTask%d: commandQueue is NULL, waiting...\n", clientId);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    // Xử lý lệnh từ commandQueue
    CommandData cmdData;
    if (xQueueReceive(commandQueue, &cmdData, 0)) {
      if (cmdData.carId == ALL_CARS) {
        sendCommandToAll(cmdData.cmd, cmdData.payload, cmdData.payloadLen);
      } else {
        if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
          Serial.printf("ClientTask%d: Cannot take semaphore for command 0x%02X\n", clientId, cmdData.cmd);
          vTaskDelay(300 / portTICK_PERIOD_MS);
          continue;
        }
        for (int i = 0; i < maxClients; i++) {
          if (clients[i].connected() && clients[i].remoteIP() == carConfigs[cmdData.carId].ip) {
            sendCommand(clients[i], cmdData.cmd, cmdData.carId, cmdData.payload, cmdData.payloadLen);
            char msg[64];
            snprintf(msg, sizeof(msg), "Sent command 0x%02X to Car%d", cmdData.cmd, cmdData.carId + 1);
            xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
            break;
          }
        }
        xSemaphoreGive(clientMutex);
      }
    }

    // Kiểm tra kết nối client
    if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
      Serial.printf("ClientTask%d: Cannot take semaphore for connection check\n", clientId);
      vTaskDelay(300 / portTICK_PERIOD_MS);
      continue;
    }
    if (!clients[clientId].connected() || clients[clientId].write((uint8_t)0) == 0) {
      CarID carId = getCarIdByIP(clients[clientId].remoteIP());
      carOnlineStatus[carId] = false;
      carStatuses[carId].isOn = false;
      saveCarStatus(carId);
      char msg[64];
      snprintf(msg, sizeof(msg), "Car%d disconnected.", carId + 1);
      Serial.println(msg);
      xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
      clients[clientId].stop();
      xSemaphoreGive(clientMutex);
      vTaskDelete(NULL);
    }
    xSemaphoreGive(clientMutex);

    // Xử lý dữ liệu từ client
    if (clients[clientId].connected() && clients[clientId].available() >= 7) {
      if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.printf("ClientTask%d: Cannot take semaphore for data read\n", clientId);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        continue;
      }
      uint8_t buffer[32];
      clients[clientId].setTimeout(100);
      int len = clients[clientId].readBytes(buffer, sizeof(buffer));
      xSemaphoreGive(clientMutex);
      if (len == 0) {
        Serial.printf("ClientTask%d: No data received, possible timeout\n", clientId);
        if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          CarID carId = getCarIdByIP(clients[clientId].remoteIP());
          carOnlineStatus[carId] = false;
          carStatuses[carId].isOn = false;
          saveCarStatus(carId);
          char msg[64];
          snprintf(msg, sizeof(msg), "Car%d disconnected.", carId + 1);
          Serial.println(msg);
          xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
          clients[clientId].stop();
          xSemaphoreGive(clientMutex);
          vTaskDelete(NULL);
        }
        continue;
      }
      Serial.printf("Received from client %d: len=%d\n", clientId, len);
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
      if (frameLen >= 7 && frameLen <= len && (carId <= CAR5 || carId == ALL_CARS)) {
        uint8_t checksum = 0;
        for (int i = frameStart; i < frameStart + frameLen - 2; i++) checksum += buffer[i];
        if (checksum == buffer[frameStart + frameLen - 2]) {
          CarID id = (carId == ALL_CARS) ? getCarIdByIP(clients[clientId].remoteIP()) : (CarID)carId;
          char msg[128];
          bool resend = false;
          switch (cmd) {
            case STATUS_REPORT:
              break;
            case ON_OFF_ALL:
            case ON_OFF_CAR:
              carStatuses[id].isOn = buffer[frameStart + 5];
              saveCarStatus(id);
              snprintf(msg, sizeof(msg), "Car%d: Power=%s", id + 1, carStatuses[id].isOn ? "ON" : "OFF");
              break;
            case SPEED_ALL:
            case SPEED_CAR:
              carStatuses[id].speed = buffer[frameStart + 5];
              saveCarStatus(id);
              snprintf(msg, sizeof(msg), "Car%d: Speed=%d%%", id + 1, carStatuses[id].speed);
              break;
            case MODE:
              carStatuses[id].mode = buffer[frameStart + 5];
              saveCarStatus(id);
              snprintf(msg, sizeof(msg), "Car%d: Mode=%d", id + 1, carStatuses[id].mode);
              break;
            case GPIO_CONTROL:
              for (int i = 0; i < 6; i++) carStatuses[id].gpio[i] = buffer[frameStart + 5 + i];
              saveCarStatus(id);
              snprintf(msg, sizeof(msg), "Car%d: GPIO=%d %d %d %d %d %d", id + 1,
                       carStatuses[id].gpio[0], carStatuses[id].gpio[1], carStatuses[id].gpio[2],
                       carStatuses[id].gpio[3], carStatuses[id].gpio[4], carStatuses[id].gpio[5]);
              break;
            case BAT_REPORT:
              if (dataLen == 1) {
                carStatuses[id].battery_level = buffer[frameStart + 5];
                saveCarStatus(id);
                snprintf(msg, sizeof(msg), "Car%d: Battery=%d%%", id + 1, carStatuses[id].battery_level);
                if (id == CAR1 && ui_Car1Batlabel) {
                  char label_text[32];
                  snprintf(label_text, sizeof(label_text), "%d%%", carStatuses[id].battery_level);
                  lvgl_port_lock(-1);
                  lv_label_set_text(ui_Car1Batlabel, label_text);
                  lvgl_port_unlock();
                }
              }
              break;
          }
          if (msg[0] != '\0') xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
          if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            clients[clientId].write("OK", 2);
            clients[clientId].flush();
            xSemaphoreGive(clientMutex);
            vTaskDelay(50 / portTICK_PERIOD_MS);
          }
        } else {
          Serial.println("Checksum error!");
        }
      } else {
        Serial.println("Invalid packet format or car ID!");
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // Giảm delay để kiểm tra nhanh hơn
  }
}

// Task server
void serverTask(void *pvParameters) {
  while (1) {
    WiFiClient newClient = server.accept();
    if (newClient) {
      if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.println("Cannot take semaphore for new client");
        newClient.stop();
        vTaskDelay(300 / portTICK_PERIOD_MS);
        continue;
      }
      bool added = false;
      IPAddress newClientIP = newClient.remoteIP();
      for (int i = 0; i < maxClients; i++) {
        if (clients[i].connected() && clients[i].remoteIP() == newClientIP) {
          Serial.printf("Client %s already connected, rejecting new connection\n", newClientIP.toString().c_str());
          newClient.stop();
          xSemaphoreGive(clientMutex);
          vTaskDelay(300 / portTICK_PERIOD_MS);
          continue;
        }
      }
      for (int i = 0; i < maxClients; i++) {
        if (!clients[i].connected()) {
          clients[i] = newClient;
          CarID carId = getCarIdByIP(newClientIP);
          carOnlineStatus[carId] = true;
          char msg[64];
          snprintf(msg, sizeof(msg), "New client: Car%d (%s)", carId + 1, newClientIP.toString().c_str());
          Serial.println(msg);
          xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
          char taskName[20];
          snprintf(taskName, sizeof(taskName), "ClientTask%d", i);
          if (xTaskCreate(clientTask, taskName, 6144, (void *)i, 1, NULL) != pdPASS) {
            Serial.printf("Failed to create ClientTask%d\n", i);
            clients[i].stop();
          } else {
            added = true;
          }
          break;
        }
      }
      if (!added) {
        char msg[32] = "Max clients reached!";
        xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
        newClient.stop();
      }
      xSemaphoreGive(clientMutex);
    }
    if (millis() % 1000 == 0) checkClientStatus(); // Kiểm tra mỗi 1 giây
    vTaskDelay(100 / portTICK_PERIOD_MS); 
  }
}

// Task in dữ liệu và cập nhật LVGL
void printTask(void *pvParameters) {
  char buffer[128];
  while (1) {
    if (xQueueReceive(dataQueue, buffer, portMAX_DELAY)) {
      Serial.printf("Processing buffer: %s\n", buffer); // Debug
      if (lvgl_port_lock(-1)) {
        if (ui_StatusArea) {
          if (strstr(buffer, "New client: Car") || strstr(buffer, "Car") && strstr(buffer, "disconnected")) {
            char status_text[64];
            snprintf(status_text, sizeof(status_text), 
                     "C1: %s, C2: %s, C3: %s, C4: %s, C5: %s",
                     carOnlineStatus[CAR1] ? "On" : "Off",
                     carOnlineStatus[CAR2] ? "On" : "Off",
                     carOnlineStatus[CAR3] ? "On" : "Off",
                     carOnlineStatus[CAR4] ? "On" : "Off",
                     carOnlineStatus[CAR5] ? "On" : "Off");
            lv_textarea_set_text(ui_StatusArea, status_text);
            lv_obj_invalidate(ui_StatusArea); // Làm mới textarea
            Serial.printf("Updated ui_StatusArea: %s\n", status_text); // Debug
          } else {
            Serial.printf("Buffer ignored: %s\n", buffer); // Debug các thông điệp không xử lý
          }
        } else {
          Serial.println("Error: ui_StatusArea is NULL in printTask!");
        }
        lvgl_port_unlock();
      } else {
        Serial.println("Failed to lock LVGL mutex in printTask!");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Giảm delay để xử lý nhanh hơn
  }
}

// task để gửi yêu cầu BAT_REPORT mỗi 15 giây
void batteryRequestTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
      int sentCount = 0;
      for (int i = 0; i < maxClients; i++) {
        if (clients[i].connected()) {
          uint8_t payload[1] = {0}; // Payload rỗng cho yêu cầu BAT_REPORT
          sendCommand(clients[i], BAT_REPORT, carConfigs[i].id, payload, 0);
          sentCount++;
        }
      }
      xSemaphoreGive(clientMutex);
      if (sentCount > 0) {
        Serial.printf("Sent BAT_REPORT request to %d clients\n", sentCount);
      } else {
        Serial.println("No clients connected for BAT_REPORT request");
      }
    } else {
      Serial.println("Cannot take semaphore for batteryRequestTask");
    }
    vTaskDelay(20000 / portTICK_PERIOD_MS); // Gửi mỗi 20 giây
  }
}

// Task thiết lập Access Point
void wifiTask(void *pvParameters) {
  WiFi.softAP(ssid, password);
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

    lvgl_port_lock(-1);
    ui_init();
    if (ui_StatusArea) {
      lv_textarea_set_text(ui_StatusArea, "Car1: Offline, Car2: Offline, Car3: Offline, Car4: Offline, Car5: Offline");
      Serial.println("Initialized ui_StatusArea with default text");
    } else {
      Serial.println("Error: ui_StatusArea is NULL after ui_init!");
    }
    lvgl_port_unlock();

    // Khởi tạo hàng đợi và semaphore
    dataQueue = xQueueCreate(10, 128);
    if (!dataQueue) {
      Serial.println("Failed to create dataQueue!");
      while (1);
    }
    commandQueue = xQueueCreate(10, sizeof(CommandData));
    clientMutex = xSemaphoreCreateMutex();
    if (!clientMutex) {
      Serial.println("Failed to create clientMutex!");
      while (1);
    }

    // Khởi tạo các task
    xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 2, NULL);
    xTaskCreate(serverTask, "ServerTask", 8192, NULL, 2, NULL);
    xTaskCreate(printTask, "PrintTask", 16384, NULL, 1, NULL);

    if (xTaskCreate(batteryRequestTask, "BatteryRequestTask", 4096, NULL, 1, NULL) != pdPASS) {
    Serial.println("Failed to create BatteryRequestTask");
    }
    xTaskCreate(lvglTask, "LVGLTask", 16384, NULL, 1, NULL);

    Serial.printf("Setup complete. Free heap: %d bytes\n", ESP.getFreeHeap());

    // // Khởi tạo LVGL UI
    // lvgl_port_lock(-1);

    // /* Release the mutex */
    // lvgl_port_unlock();
    // ui_init();

    Serial.println(title + " end");
}

void loop()
{
  static uint32_t last = 0;
  if (millis() - last > 5000) {
    //Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    last = millis();
  }
}

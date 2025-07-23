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

CarConfig carConfigs[5] = {
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
  TaskHandle_t holder = xSemaphoreGetMutexHolder(clientMutex);
  if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    Serial.printf("Cannot take semaphore for checkClientStatus, current task: %s, holder: %s\n", 
                  pcTaskGetName(NULL), holder ? pcTaskGetName(holder) : "None");
    return;
  }
  for (int i = 0; i < maxClients; i++) {
    if (clients[i].connected() && !clients[i].availableForWrite()) {
      CarID carId = getCarIdByIP(clients[i].remoteIP());
      carStatuses[carId].isOn = false;
      saveCarStatus(carId);
      char msg[64];
      snprintf(msg, sizeof(msg), "Car%d: Disconnected, set OFF", carId + 1);
      xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
      clients[i].stop();
    }
  }
  xSemaphoreGive(clientMutex);
}

// Task xử lý client
void clientTask(void *pvParameters) {
  int clientId = (int)pvParameters;
  while (1) {
    // Kiểm tra commandQueue có hợp lệ không
    if (commandQueue == NULL) {
      Serial.printf("ClientTask%d: commandQueue is NULL, waiting...\n", clientId);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    // Xử lý lệnh từ commandQueue
    CommandData cmdData;
    if (xQueueReceive(commandQueue, &cmdData, 0)) {
      if (cmdData.carId == ALL_CARS) {
        // Gọi sendCommandToAll trực tiếp, không cần clientMutex ở đây
        sendCommandToAll(cmdData.cmd, cmdData.payload, cmdData.payloadLen);
      } else {
        // Lệnh cho một xe cụ thể
        if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
          Serial.printf("ClientTask%d: Cannot take semaphore for command 0x%02X, current task: %s, holder: %s\n", 
                        clientId, cmdData.cmd, pcTaskGetName(NULL), xSemaphoreGetMutexHolder(clientMutex) ? pcTaskGetName(xSemaphoreGetMutexHolder(clientMutex)) : "None");
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
    bool isConnected = false;
    {
      if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
        Serial.printf("ClientTask%d: Cannot take semaphore for connection check, current task: %s, holder: %s\n", 
                      clientId, pcTaskGetName(NULL), xSemaphoreGetMutexHolder(clientMutex) ? pcTaskGetName(xSemaphoreGetMutexHolder(clientMutex)) : "None");
        vTaskDelay(300 / portTICK_PERIOD_MS);
        continue;
      }
      isConnected = clients[clientId].connected();
      if (!isConnected) {
        CarID carId = getCarIdByIP(clients[clientId].remoteIP());
        char msg[64];
        snprintf(msg, sizeof(msg), "Car%d disconnected.", carId + 1);
        xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
        clients[clientId].stop();
        xSemaphoreGive(clientMutex);
        vTaskDelete(NULL);
      }
      xSemaphoreGive(clientMutex);
    }

    // Xử lý dữ liệu từ client
    if (isConnected) {
      if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
        Serial.printf("ClientTask%d: Cannot take semaphore for data read, current task: %s, holder: %s\n", 
                      clientId, pcTaskGetName(NULL), xSemaphoreGetMutexHolder(clientMutex) ? pcTaskGetName(xSemaphoreGetMutexHolder(clientMutex)) : "None");
        vTaskDelay(300 / portTICK_PERIOD_MS);
        continue;
      }
      if (clients[clientId].available() >= 7) {
        uint8_t buffer[16];
        clients[clientId].setTimeout(100); // Giảm timeout xuống 100ms
        int len = clients[clientId].readBytes(buffer, sizeof(buffer));
        xSemaphoreGive(clientMutex); // Thả mutex ngay sau khi đọc
        if (len == 0) {
          Serial.printf("No data received from client %d, possible timeout\n", clientId);
          if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
            clients[clientId].stop();
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
        if (len >= 7 && buffer[0] == 0xAA && buffer[1] == 0x01 && buffer[len-1] == 0xFF) {
          uint8_t cmd = buffer[2];
          uint8_t carId = buffer[3];
          uint8_t dataLen = buffer[4];
          Serial.printf("Packet info: cmd=0x%02X, car_id=%d, dataLen=%d\n", cmd, carId, dataLen);
          if (len >= 6 + dataLen && (carId <= CAR5 || carId == ALL_CARS)) {
            uint8_t checksum = 0;
            for (int i = 0; i < len-2; i++) checksum += buffer[i];
            if (checksum == buffer[len-2]) {
              CarID id = (carId == ALL_CARS) ? getCarIdByIP(clients[clientId].remoteIP()) : (CarID)carId;
              char msg[128];
              bool resend = false;
              switch (cmd) {
                case STATUS_REPORT:
                  if (dataLen == 9) {
                    bool isOn = buffer[5];
                    uint8_t speed = buffer[6];
                    uint8_t mode = buffer[7];
                    uint8_t gpio[6];
                    memcpy(gpio, buffer + 8, 6);
                    if (carStatuses[id].isOn != isOn) {
                      uint8_t payload[1] = {carStatuses[id].isOn};
                      sendCommand(clients[clientId], ON_OFF_CAR, id, payload, 1);
                      resend = true;
                    } else if (carStatuses[id].speed != speed) {
                      uint8_t payload[1] = {carStatuses[id].speed};
                      sendCommand(clients[clientId], SPEED_CAR, id, payload, 1);
                      resend = true;
                    } else if (carStatuses[id].mode != mode) {
                      uint8_t payload[1] = {carStatuses[id].mode};
                      sendCommand(clients[clientId], MODE, id, payload, 1);
                      resend = true;
                    } else {
                      for (int i = 0; i < 6; i++) {
                        if (carStatuses[id].gpio[i] != gpio[i]) {
                          uint8_t payload[6];
                          memcpy(payload, carStatuses[id].gpio, 6);
                          sendCommand(clients[clientId], GPIO_CONTROL, id, payload, 6);
                          resend = true;
                          break;
                        }
                      }
                    }
                    if (!resend) {
                      snprintf(msg, sizeof(msg), "Car%d: Status OK - ON=%d, Speed=%d, Mode=%d, GPIO=%d %d %d %d %d %d",
                               id + 1, isOn, speed, mode, gpio[0], gpio[1], gpio[2], gpio[3], gpio[4], gpio[5]);
                    }
                  }
                  break;
                case ON_OFF_ALL:
                case ON_OFF_CAR:
                  carStatuses[id].isOn = buffer[5];
                  saveCarStatus(id);
                  snprintf(msg, sizeof(msg), "Car%d: Power=%s", id + 1, carStatuses[id].isOn ? "ON" : "OFF");
                  break;
                case SPEED_ALL:
                case SPEED_CAR:
                  carStatuses[id].speed = buffer[5];
                  saveCarStatus(id);
                  snprintf(msg, sizeof(msg), "Car%d: Speed=%d%%", id + 1, carStatuses[id].speed);
                  break;
                case MODE:
                  carStatuses[id].mode = buffer[5];
                  saveCarStatus(id);
                  snprintf(msg, sizeof(msg), "Car%d: Mode=%d", id + 1, carStatuses[id].mode);
                  break;
                case GPIO_CONTROL:
                  for (int i = 0; i < 6; i++) carStatuses[id].gpio[i] = buffer[5 + i];
                  saveCarStatus(id);
                  snprintf(msg, sizeof(msg), "Car%d: GPIO=%d %d %d %d %d %d", id + 1,
                           carStatuses[id].gpio[0], carStatuses[id].gpio[1], carStatuses[id].gpio[2],
                           carStatuses[id].gpio[3], carStatuses[id].gpio[4], carStatuses[id].gpio[5]);
                  break;
                case BAT_REPORT:
                  if (dataLen == 1) {
                    carStatuses[id].battery_level = buffer[5];
                    saveCarStatus(id);
                    snprintf(msg, sizeof(msg), "Car%d: Battery=%d%%", id + 1, carStatuses[id].battery_level);
                  }
                  break;
              }
              if (msg[0] != '\0') xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
              if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
                clients[clientId].write("OK", 2);
                xSemaphoreGive(clientMutex);
              }
            } else {
              Serial.println("Checksum error!");
            }
          } else {
            Serial.println("Invalid packet format or car ID!");
          }
        } else {
          Serial.println("Invalid packet header or footer!");
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
      TaskHandle_t holder = xSemaphoreGetMutexHolder(clientMutex);
      if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
        Serial.printf("Cannot take semaphore for new client, current task: %s, holder: %s\n", 
                      pcTaskGetName(NULL), holder ? pcTaskGetName(holder) : "None");
        newClient.stop();
        vTaskDelay(300 / portTICK_PERIOD_MS);
        continue;
      }
      bool added = false;
      IPAddress newClientIP = newClient.remoteIP();
      // Kiểm tra xem IP đã tồn tại trong danh sách clients chưa
      for (int i = 0; i < maxClients; i++) {
        if (clients[i].connected() && clients[i].remoteIP() == newClientIP) {
          Serial.printf("Client %s already connected, rejecting new connection\n", newClientIP.toString().c_str());
          newClient.stop();
          xSemaphoreGive(clientMutex);
          vTaskDelay(300 / portTICK_PERIOD_MS);
          continue;
        }
      }
      // Thêm client mới
      for (int i = 0; i < maxClients; i++) {
        if (!clients[i].connected()) {
          clients[i] = newClient;
          CarID carId = getCarIdByIP(newClientIP);
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
    vTaskDelay(100 / portTICK_PERIOD_MS); // Cập nhật 200 Hz
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

    // // Khởi tạo LVGL UI
    // lvgl_port_lock(-1);

    // /* Release the mutex */
    // lvgl_port_unlock();
    // ui_init();
    // Serial.println(title + " end");
     // Khôi phục trạng thái xe từ NVS
    //restoreCarStatus();

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
    xTaskCreate(printTask, "PrintTask", 4096, NULL, 1, NULL);
    //xTaskCreate(lvglTask, "LVGLTask", 8192, NULL, 1, NULL);

    Serial.printf("Setup complete. Free heap: %d bytes\n", ESP.getFreeHeap());

    // Khởi tạo LVGL UI
    lvgl_port_lock(-1);

    /* Release the mutex */
    lvgl_port_unlock();
    ui_init();
    Serial.println(title + " end");
}

void loop()
{
  static uint32_t last = 0;
  if (millis() - last > 5000) {
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    last = millis();
  }
}

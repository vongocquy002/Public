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
const int maxClients = 5;

// Enum định nghĩa các xe
enum CarID {
  CAR1, CAR2, CAR3, CAR4, CAR5
};

// Struct lưu cấu hình IP cho từng xe
struct CarConfig {
  CarID id;
  const char* name;
  IPAddress ip;
};

const CarConfig carConfigs[] = {
  {CAR1, "Car1", IPAddress(192, 168, 4, 2)},
  {CAR2, "Car2", IPAddress(192, 168, 4, 3)},
  {CAR3, "Car3", IPAddress(192, 168, 4, 4)},
  {CAR4, "Car4", IPAddress(192, 168, 4, 5)},
  {CAR5, "Car5", IPAddress(192, 168, 4, 6)}
};

// Command Types
enum CommandType {
  ON_OFF_ALL    = 0x01,
  SPEED_ALL     = 0x02,
  ON_OFF_CAR    = 0x03,
  SPEED_CAR     = 0x04,
  MODE          = 0x05,
  GPIO_CONTROL  = 0x06,
  STATUS_REPORT = 0x07
};

// Trạng thái xe
struct CarStatus {
  bool isOn;
  uint8_t speed;
  uint8_t mode;
  uint8_t gpio[6];
};

// Lưu trạng thái cho từng xe
CarStatus carStatuses[5] = {
  {false, 0, 1, {0, 0, 0, 0, 0, 0}},
  {false, 0, 1, {0, 0, 0, 0, 0, 0}},
  {false, 0, 1, {0, 0, 0, 0, 0, 0}},
  {false, 0, 1, {0, 0, 0, 0, 0, 0}},
  {false, 0, 1, {0, 0, 0, 0, 0, 0}}
};

// Lưu trạng thái xe vào NVS
void saveCarStatus(CarID carId) {
  preferences.begin("car_status", false);
  String key = "car" + String(carId);
  preferences.putBool(key + "_on", carStatuses[carId].isOn);
  preferences.putUChar(key + "_speed", carStatuses[carId].speed);
  preferences.putUChar(key + "_mode", carStatuses[carId].mode);
  for (int i = 0; i < 6; i++) {
    preferences.putUChar(key + "_gpio" + String(i), carStatuses[carId].gpio[i]);
  }
  preferences.end();
  Serial.println("Car " + String(carId) + " status saved to NVS");
}

// Khôi phục trạng thái xe từ NVS
void restoreCarStatus() {
  preferences.begin("car_status", true);
  for (int i = 0; i < 5; i++) {
    String key = "car" + String(i);
    carStatuses[i].isOn = preferences.getBool(key + "_on", false);
    carStatuses[i].speed = preferences.getUChar(key + "_speed", 0);
    carStatuses[i].mode = preferences.getUChar(key + "_mode", 1);
    for (int j = 0; j < 6; j++) {
      carStatuses[i].gpio[j] = preferences.getUChar(key + "_gpio" + String(j), 0);
    }
  }
  preferences.end();
  Serial.println("Car statuses restored from NVS");
}

// Tạo và gửi frame
void sendCommand(WiFiClient &client, CommandType cmd, CarID carId, uint8_t* payload, uint8_t payloadLen) {
  uint8_t frame[256] = {0xAA, 0x01, cmd, carId, payloadLen};
  memcpy(frame + 5, payload, payloadLen);
  uint8_t checksum = 0;
  for (int i = 0; i < 5 + payloadLen; i++) checksum += frame[i];
  frame[5 + payloadLen] = checksum;
  frame[6 + payloadLen] = 0xFF;
  client.write(frame, 7 + payloadLen);
}

// Hàng đợi để lưu dữ liệu từ client
QueueHandle_t dataQueue;
SemaphoreHandle_t clientMutex;
Preferences preferences; //lưu giá trị vào flash nvs

// Gửi lệnh tới tất cả client
void sendCommandToAll(CommandType cmd, uint8_t* payload, uint8_t payloadLen) {
  xSemaphoreTake(clientMutex, portMAX_DELAY);
  for (int i = 0; i < maxClients; i++) {
    if (clients[i].connected()) {
      sendCommand(clients[i], cmd, (CarID)0xFF, payload, payloadLen);
    }
  }
  xSemaphoreGive(clientMutex);
}

// Tìm CarID dựa trên IP
CarID getCarIdByIP(IPAddress ip) {
  for (int i = 0; i < 5; i++) {
    if (carConfigs[i].ip == ip) {
      return carConfigs[i].id;
    }
  }
  return CAR1; // Mặc định trả về CAR1 nếu không tìm thấy
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

// Task xử lý client
void clientTask(void *pvParameters) {
  int clientId = (int)pvParameters;
  while (1) {
    xSemaphoreTake(clientMutex, portMAX_DELAY);
    if (clients[clientId].connected()) {
      if (clients[clientId].available() >= 7) {
        uint8_t buffer[256];
        int len = clients[clientId].readBytes(buffer, 256);
        if (buffer[0] == 0xAA && buffer[1] == 0x01 && buffer[len-1] == 0xFF) {
          uint8_t cmd = buffer[2];
          uint8_t carId = buffer[3];
          uint8_t dataLen = buffer[4];
          if (len >= 6 + dataLen && carId <= CAR5 || carId == 0xFF) {
            uint8_t checksum = 0;
            for (int i = 0; i < len-2; i++) checksum += buffer[i];
            if (checksum == buffer[len-2]) {
              String msg = String(carConfigs[carId].name) + ": Cmd=" + String(cmd, HEX);
              xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
              clients[clientId].write("OK");
            }
          }
        }
      }
    } else {
      CarID carId = getCarIdByIP(clients[clientId].remoteIP());
      String msg = String(carConfigs[carId].name) + " disconnected.";
      xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
      clients[clientId].stop();
      xSemaphoreGive(clientMutex);
      vTaskDelete(NULL);
    }
    xSemaphoreGive(clientMutex);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task chấp nhận client mới
void serverTask(void *pvParameters) {
  while (1) {
    WiFiClient newClient = server.accept();
    if (newClient) {
      xSemaphoreTake(clientMutex, portMAX_DELAY);
      bool added = false;
      for (int i = 0; i < maxClients; i++) {
        if (!clients[i].connected()) {
          clients[i] = newClient;
          CarID carId = getCarIdByIP(newClient.remoteIP());
          String msg = String("Client mới kết nối, ID: ") + String(i) + ", " + carConfigs[carId].name + " (" + newClient.remoteIP().toString() + ")";
          xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);

          // Tạo task xử lý client
          char taskName[20];
          sprintf(taskName, "ClientTask%d", i);
          xTaskCreate(clientTask, taskName, 4096, (void *)i, 1, NULL);
          added = true;
          break;
        }
      }
      if (!added) {
        String msg = "Đã đạt tối đa số client, từ chối kết nối mới!";
        xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
        newClient.stop();
      }
      xSemaphoreGive(clientMutex);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task xử lý dữ liệu từ client
void clientTask(void *pvParameters) {
  int clientId = (int)pvParameters;
  while (1) {
    xSemaphoreTake(clientMutex, portMAX_DELAY);
    if (clients[clientId].connected()) {
      if (clients[clientId].available()) {
        String data = clients[clientId].readStringUntil('\n');
        CarID carId = getCarIdByIP(clients[clientId].remoteIP());
        String msg = String(carConfigs[carId].name) + " (" + clients[clientId].remoteIP().toString() + "): " + data;
        xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);

        // Gửi phản hồi
        clients[clientId].println("Server nhận: " + data);
      }
    } else {
      CarID carId = getCarIdByIP(clients[clientId].remoteIP());
      String msg = String(carConfigs[carId].name) + " (" + clients[clientId].remoteIP().toString() + ") đã ngắt kết nối.";
      xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
      clients[clientId].stop();
      xSemaphoreGive(clientMutex);
      vTaskDelete(NULL);
    }
    xSemaphoreGive(clientMutex);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task in dữ liệu từ hàng đợi
void printTask(void *pvParameters) {
  char buffer[128];
  while (1) {
    if (xQueueReceive(dataQueue, buffer, portMAX_DELAY)) {
      Serial.println(buffer);
    }
  }
}

// Task xử lý LVGL
void lvglTask(void *pvParameters) {
  while (1) {
    lvgl_port_lock(-1);
    lv_timer_handler(); // Xử lý giao diện LVGL
    lvgl_port_unlock();
    vTaskDelay(5 / portTICK_PERIOD_MS); // Cập nhật 200 Hz
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
    dataQueue = xQueueCreate(10, 128 * sizeof(char));
    clientMutex = xSemaphoreCreateMutex();

    // Khởi tạo các task
    xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 2, NULL);
    xTaskCreate(serverTask, "ServerTask", 4096, NULL, 2, NULL);
    xTaskCreate(printTask, "PrintTask", 2048, NULL, 1, NULL);
    //xTaskCreate(lvglTask, "LVGLTask", 4096, NULL, 1, NULL);

    // Khởi tạo LVGL UI
    lvgl_port_lock(-1);

    /* Release the mutex */
    lvgl_port_unlock();
    ui_init();
    Serial.println(title + " end");
}

void loop()
{
    
}

#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

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
  CAR1,
  CAR2,
  CAR3,
  CAR4,
  CAR5
};

// Struct lưu cấu hình IP cho từng xe
struct CarConfig {
  CarID id;
  const char* name;
  IPAddress ip;
};

// Cấu hình IP tĩnh cho 5 xe
const CarConfig carConfigs[] = {
  {CAR1, "Car1", IPAddress(192, 168, 4, 2)},
  {CAR2, "Car2", IPAddress(192, 168, 4, 3)},
  {CAR3, "Car3", IPAddress(192, 168, 4, 4)},
  {CAR4, "Car4", IPAddress(192, 168, 4, 5)},
  {CAR5, "Car5", IPAddress(192, 168, 4, 6)}
};

// Hàng đợi để lưu dữ liệu từ client
QueueHandle_t dataQueue;
SemaphoreHandle_t clientMutex;

// Biến kiểm soát timeout
unsigned long lastActivity[5];
const unsigned long timeout = 30000000; // 30 giây

// Tìm CarID dựa trên IP
CarID getCarIdByIP(IPAddress ip) {
  for (int i = 0; i < 5; i++) {
    if (carConfigs[i].ip == ip) {
      return carConfigs[i].id;
    }
  }
  return CAR1; // Mặc định trả về CAR1 nếu không tìm thấy
}

// Hàm gửi dữ liệu đến client từ Serial
void sendCommandTask(void *pvParameters) {
  while (1) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.length() > 0) {
        // Định dạng lệnh: "CARX:command" (ví dụ: CAR1:forward)
        int colonIndex = input.indexOf(':');
        if (colonIndex > 0) {
          String carName = input.substring(0, colonIndex);
          String command = input.substring(colonIndex + 1);

          // Tìm CarConfig tương ứng
          const CarConfig* targetCar = nullptr;
          for (int i = 0; i < 5; i++) {
            if (carName.equalsIgnoreCase(carConfigs[i].name)) {
              targetCar = &carConfigs[i];
              break;
            }
          }

          if (targetCar) {
            xSemaphoreTake(clientMutex, portMAX_DELAY);
            bool sent = false;
            for (int i = 0; i < maxClients; i++) {
              if (clients[i].connected() && clients[i].remoteIP() == targetCar->ip) {
                clients[i].println(command);
                String msg = String("Đã gửi lệnh đến ") + targetCar->name + ": " + command;
                xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
                sent = true;
                break;
              }
            }
            if (!sent) {
              String msg = String("Không tìm thấy ") + targetCar->name + " hoặc chưa kết nối!";
              xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
            }
            xSemaphoreGive(clientMutex);
          } else {
            String msg = String("Tên xe không hợp lệ: ") + carName;
            xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
          }
        } else {
          String msg = String("Lệnh không đúng định dạng (CARX:command): ") + input;
          xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Kiểm tra Serial mỗi 100ms
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
  vTaskDelete(NULL); // Kết thúc task sau khi thiết lập
}

// Task chấp nhận client mới
void serverTask(void *pvParameters) {
  while (1) {
    WiFiClient newClient = server.accept(); // Thay available() bằng accept()
    if (newClient) {
      xSemaphoreTake(clientMutex, portMAX_DELAY);
      bool added = false;
      for (int i = 0; i < maxClients; i++) {
        if (!clients[i].connected()) {
          clients[i] = newClient;
          lastActivity[i] = millis();
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
        lastActivity[clientId] = millis();
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
      lastActivity[clientId] = 0;
      xSemaphoreGive(clientMutex);
      vTaskDelete(NULL);
    }
    xSemaphoreGive(clientMutex);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task giám sát timeout
void monitorTask(void *pvParameters) {
  while (1) {
    xSemaphoreTake(clientMutex, portMAX_DELAY);
    for (int i = 0; i < maxClients; i++) {
      if (clients[i].connected() && (millis() - lastActivity[i] > timeout)) {
        CarID carId = getCarIdByIP(clients[i].remoteIP());
        String msg = String(carConfigs[carId].name) + " hết thời gian, ngắt kết nối!";
        xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY);
        clients[i].stop();
        lastActivity[i] = 0;
      }
    }
    xSemaphoreGive(clientMutex);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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

void setup() {
  Serial.begin(115200);

  // Khởi tạo hàng đợi và semaphore
  dataQueue = xQueueCreate(10, 128 * sizeof(char));
  clientMutex = xSemaphoreCreateMutex();

  // Khởi tạo các task
  xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 2, NULL);
  xTaskCreate(serverTask, "ServerTask", 4096, NULL, 2, NULL);
  xTaskCreate(monitorTask, "MonitorTask", 2048, NULL, 1, NULL);
  xTaskCreate(printTask, "PrintTask", 2048, NULL, 1, NULL);
  xTaskCreate(sendCommandTask, "SendCommandTask", 4096, NULL, 1, NULL);

  // Khởi tạo thời gian hoạt động
  for (int i = 0; i < maxClients; i++) {
    lastActivity[i] = 0;
  }
}

void loop() {
  
}
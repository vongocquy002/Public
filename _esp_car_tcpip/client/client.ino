#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Định nghĩa xe để chọn (thay đổi giá trị này cho từng client/xe)
#define CAR_NAME CAR1

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

// Thông tin Access Point
const char* ssid = "ESP32_AP";
const char* password = "12345678";

// Thông tin Server
const char* serverIP = "192.168.4.1"; // IP của Server (AP)
const uint16_t serverPort = 12345;

// Cấu hình mạng chung
IPAddress gateway(192, 168, 4, 1);   // Gateway là IP của Server
IPAddress subnet(255, 255, 255, 0);  // Subnet mask

WiFiClient client;

// Lấy cấu hình cho xe được chọn
const CarConfig& getCarConfig() {
  for (int i = 0; i < 5; i++) {
    if (carConfigs[i].id == CAR_NAME) {
      return carConfigs[i];
    }
  }
  return carConfigs[0]; // Mặc định trả về CAR1
}

// Task kết nối WiFi
void wifiTask(void *pvParameters) {
  const CarConfig& config = getCarConfig();
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Đang kết nối đến Access Point: " + String(ssid));

      // Cấu hình IP tĩnh
      if (!WiFi.config(config.ip, gateway, subnet)) {
        Serial.println("Lỗi cấu hình IP tĩnh cho " + String(config.name) + " (" + config.ip.toString() + ")");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        continue;
      }

      // Kết nối WiFi
      WiFi.begin(ssid, password);
      int attempts = 0;
      const int maxAttempts = 15; // Tăng số lần thử
      while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
        Serial.print("Thử kết nối WiFi lần " + String(attempts + 1) + "... ");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        attempts++;
        switch (WiFi.status()) {
          case WL_NO_SSID_AVAIL:
            Serial.println("Thất bại: Không tìm thấy SSID!");
            break;
          case WL_CONNECT_FAILED:
            Serial.println("Thất bại: Sai mật khẩu hoặc lỗi kết nối!");
            break;
          case WL_CONNECTION_LOST:
            Serial.println("Thất bại: Mất kết nối!");
            break;
          default:
            Serial.println("Thất bại: Mã trạng thái " + String(WiFi.status()));
            break;
        }
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Đã kết nối đến Access Point!");
        Serial.print("Địa chỉ IP của Client (");
        Serial.print(config.name);
        Serial.print("): ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("Kết nối WiFi thất bại, thử lại sau 2 giây...");
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Kiểm tra kết nối mỗi 2 giây
  }
}

// Task giao tiếp với Server
void commTask(void *pvParameters) {
  const CarConfig& config = getCarConfig();
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Mất kết nối WiFi, chờ kết nối lại...");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }

    if (!client.connected()) {
      Serial.println("Đang kết nối đến Server: " + String(serverIP) + ":" + String(serverPort));
      if (client.connect(serverIP, serverPort)) {
        Serial.println("Đã kết nối đến Server!");
      } else {
        Serial.println("Kết nối Server thất bại!");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        continue;
      }
    }

    // Gửi dữ liệu (bao gồm tên xe và IP)
    String message = "Xin chào từ " + String(config.name) + " (" + String(WiFi.localIP()) + ")";
    client.println(message);
    Serial.println("Đã gửi: " + message);

    // Nhận dữ liệu và in ra Serial Monitor
    while (client.available()) {
      String data = client.readStringUntil('\n');
      data.trim(); // Xóa khoảng trắng và ký tự thừa
      Serial.println(String(config.name) + " nhận: " + data);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Gửi mỗi 2 giây
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Khởi động Client " + String(getCarConfig().name));
  Serial.print("Heap tự do: ");
  Serial.println(ESP.getFreeHeap());

  // Đặt vùng WiFi để tránh vấn đề kênh
  WiFi.mode(WIFI_STA);
  //esp_wifi_set_country_code("VN", true); // Đặt vùng Việt Nam, thay đổi nếu cần

  // Thêm delay để Server khởi động trước
  Serial.println("Chờ 5 giây để Server khởi động...");
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  // Khởi tạo các task
  xTaskCreate(wifiTask, "WiFiTask", 8192, NULL, 2, NULL); // Tăng stack size
  xTaskCreate(commTask, "CommTask", 8192, NULL, 1, NULL); // Tăng stack size
}

void loop() {
  // Loop trống, FreeRTOS quản lý tất cả
}
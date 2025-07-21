#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Định nghĩa xe để chọn (thay đổi giá trị này cho từng client/xe)
// Ví dụ: #define CAR_NAME CAR1 cho xe thứ nhất
//        #define CAR_NAME CAR2 cho xe thứ hai, v.v.
#define CAR_NAME CAR1 // <--- Thay đổi giá trị này cho từng ESP32 client

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
const char* serverIP = "192.168.4.1"; // IP của Server (AP ESP32)
const uint16_t serverPort = 12345;

// Cấu hình mạng chung
IPAddress gateway(192, 168, 4, 1);   // Gateway là IP của Server
IPAddress subnet(255, 255, 255, 0);  // Subnet mask

WiFiClient client; // Đối tượng WiFiClient cho xe này

// Biến cờ để theo dõi trạng thái đã gửi lời chào hay chưa
bool hasSentHello = false;

// Lấy cấu hình cho xe được chọn
const CarConfig& getCarConfig() {
  for (int i = 0; i < 5; i++) {
    if (carConfigs[i].id == CAR_NAME) {
      return carConfigs[i];
    }
  }
  return carConfigs[0]; // Mặc định trả về CAR1 nếu không tìm thấy
}

// Task kết nối WiFi
void wifiTask(void *pvParameters) {
  const CarConfig& config = getCarConfig();
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Đang kết nối đến Access Point: " + String(ssid));

      // Cấu hình IP tĩnh cho ESP32 client này
      if (!WiFi.config(config.ip, gateway, subnet)) {
        Serial.println("Lỗi cấu hình IP tĩnh cho " + String(config.name) + " (" + config.ip.toString() + ")");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        continue; // Thử lại cấu hình
      }

      // Bắt đầu kết nối WiFi
      WiFi.begin(ssid, password);
      int attempts = 0;
      const int maxAttempts = 15; // Tăng số lần thử kết nối
      while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
        Serial.print("Thử kết nối WiFi lần " + String(attempts + 1) + "... ");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Đợi 1 giây giữa các lần thử
        attempts++;
        // In trạng thái lỗi chi tiết hơn
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

// Task giao tiếp với Server (Chỉ chào 1 lần, sau đó chỉ nhận lệnh)
void commTask(void *pvParameters) {
  const CarConfig& config = getCarConfig();
  while (1) {
    // Nếu mất kết nối WiFi, chờ kết nối lại
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Mất kết nối WiFi, chờ kết nối lại...");
      hasSentHello = false; // Đặt lại cờ để có thể gửi chào khi kết nối lại
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }

    // Nếu chưa kết nối tới Server, thử kết nối
    if (!client.connected()) {
      Serial.println("Đang kết nối đến Server: " + String(serverIP) + ":" + String(serverPort));
      if (client.connect(serverIP, serverPort)) {
        Serial.println("Đã kết nối đến Server!");
        hasSentHello = false; // Đặt lại cờ để gửi chào khi kết nối lại
      } else {
        Serial.println("Kết nối Server thất bại!");
        hasSentHello = false; // Đặt lại cờ
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        continue;
      }
    }

    // Gửi tin nhắn chào mừng chỉ một lần khi kết nối thành công và chưa gửi
    if (client.connected() && !hasSentHello) {
      String message = String(config.name) + ":Hello:" + String(WiFi.localIP());
      client.println(message);
      Serial.println("Đã gửi tin nhắn báo danh: " + message);
      hasSentHello = true; // Đánh dấu là đã gửi
    }

    // Nhận dữ liệu và in ra Serial Monitor
    if (client.available()) {
      String data = client.readStringUntil('\n');
      data.trim(); // Xóa khoảng trắng và ký tự thừa

      // Xử lý lệnh nhận được từ server tại đây
      // Ví dụ: "mode:auto", "speed:150", "led:on", "forward"
      Serial.println(String(config.name) + " nhận lệnh: " + data);

      // Nếu cần, bạn có thể gửi một phản hồi lại server sau khi nhận lệnh
      // client.println("ACK:" + data);
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Kiểm tra dữ liệu đến mỗi 50ms
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Khởi động Client " + String(getCarConfig().name) + " ---");
  Serial.print("Địa chỉ IP dự kiến: ");
  Serial.println(getCarConfig().ip.toString());
  Serial.print("Heap tự do: ");
  Serial.println(ESP.getFreeHeap());

  // Đặt chế độ WiFi là Station (STA)
  WiFi.mode(WIFI_STA);
  // Cài đặt mã quốc gia để tối ưu hóa WiFi (tùy chọn)
  // esp_wifi_set_country_code("VN", true);

  // Thêm delay để Server khởi động trước khi client cố gắng kết nối
  Serial.println("Chờ 5 giây trước khi client bắt đầu kết nối...");
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  // Khởi tạo các task FreeRTOS
  // Tăng stack size cho các task để đảm bảo đủ bộ nhớ
  xTaskCreate(wifiTask, "WiFiTask", 8192, NULL, 2, NULL);
  xTaskCreate(commTask, "CommTask", 8192, NULL, 1, NULL);
}

void loop() {
  // Loop trống, FreeRTOS quản lý các task
}
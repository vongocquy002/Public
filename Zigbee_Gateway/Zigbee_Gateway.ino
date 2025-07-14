#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee coordinator mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h" // Thư viện Zigbee cho ESP32 của Espressif
#include <WiFi.h>   // Thư viện WiFi

/* Cấu hình Zigbee Gateway */
// Endpoint này thường là endpoint mặc định cho gateway/coordinator
#define GATEWAY_ENDPOINT_NUMBER 1

// Khởi tạo đối tượng ZigbeeGateway
// Đối tượng này sẽ được thêm vào Zigbee stack và đại diện cho gateway
ZigbeeGateway zbGateway = ZigbeeGateway(GATEWAY_ENDPOINT_NUMBER);

/* Thông tin đăng nhập Wi-Fi */
const char *ssid = "idea";     // Thay thế bằng tên Wi-Fi của bạn
const char *password = "idea12345"; // Thay thế bằng mật khẩu Wi-Fi của bạn

/********************* Arduino functions **************************/
void setup() {
  // Khởi tạo Serial để in thông tin debug ra Serial Monitor
  Serial.begin(115200);
  Serial.println("\nStarting ESP32-C6 Single-Chip Zigbee Gateway...");

  // --- Cấu hình và kết nối Wi-Fi ---
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password); // Bắt đầu kết nối Wi-Fi

  // Chờ cho đến khi kết nối Wi-Fi thành công
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500); // Chờ 0.5 giây
  //   Serial.print(".");
  // }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // In địa chỉ IP đã nhận được

  // --- Cấu hình và Khởi tạo Zigbee trên ESP32-C6 ---
  // Tùy chọn: Đặt tên nhà sản xuất và kiểu mẫu cho thiết bị Zigbee này
  zbGateway.setManufacturerAndModel("Esp32__c3*2", "ESP32C6_ZigbeeGateway");

  // Thêm endpoint của gateway vào Zigbee Core.
  // Đây là bước cần thiết để Zigbee stack biết về gateway của bạn.
  Serial.println("Adding Zigbee Gateway endpoint...");
  Zigbee.addEndpoint(&zbGateway);

  // Tùy chọn: Mở mạng cho phép các thiết bị Zigbee khác tham gia
  // Mạng sẽ mở trong 180 giây sau khi khởi động.
  Zigbee.setRebootOpenNetwork(180);

  // Khởi động Zigbee stack ở chế độ ZIGBEE_COORDINATOR.
  // ZIGBEE_COORDINATOR: Tạo một mạng Zigbee mới và quản lý nó.
  // ZIGBEE_ROUTER: Tham gia vào một mạng hiện có và mở rộng phạm vi.
  // Vì bạn không dùng chip ngoài, ESP32-C6 sẽ chạy trực tiếp stack Zigbee.
  if (!Zigbee.begin(ZIGBEE_COORDINATOR)) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Please check your board and partition scheme settings.");
    Serial.println("Rebooting...");
    ESP.restart(); // Khởi động lại nếu Zigbee không thể bắt đầu
  } else {
    Serial.println("Zigbee started successfully as Coordinator on ESP32-C6!");
  }
}

void loop() {
  // Trong ví dụ này, không có gì cần làm trong vòng lặp loop()
  // Các hoạt động của Zigbee và WiFi được xử lý bởi các thư viện nền một cách tự động.
  // Bạn có thể thêm các logic ứng dụng của mình ở đây, ví dụ: đọc cảm biến, xử lý dữ liệu Zigbee nhận được.
}


// #ifndef ZIGBEE_MODE_ZCZR // Vẫn giữ kiểm tra này cho Coordinator
// #error "Zigbee coordinator mode is not selected in Tools->Zigbee mode"
// #endif

// #include "ZigbeeCarControl.h"
// #include "Zigbee.h" // Thư viện Zigbee Core

// /* Cấu hình Zigbee Car Control (Coordinator) */
// #define ZIGBEE_CAR_CONTROL_ENDPOINT 1

// #ifdef RGB_BUILTIN // Sử dụng LED RGB tích hợp để mô phỏng trạng thái
// uint8_t led = RGB_BUILTIN;
// #else
// uint8_t led = 2;
// #endif

// uint8_t button = BOOT_PIN; // Nút nhấn để thay đổi chế độ quạt và factory reset

// // Khởi tạo instance của ZigbeeCarControl
// ZigbeeCarControl zbCarControl = ZigbeeCarControl(ZIGBEE_CAR_CONTROL_ENDPOINT);

// // Trạng thái chế độ quạt hiện tại cho điều khiển (sẽ chuyển đổi qua các chế độ)
// // Sử dụng các chế độ car từ ZigbeeCarMode đã được định nghĩa trong ZigbeeCarCommon.h
// ZigbeeCarMode currentControlCarMode = CAR_MODE_OFF;
// // Mảng các chế độ quạt để chuyển đổi qua lại
// const ZigbeeCarMode carModes[] = {CAR_MODE_OFF, CAR_MODE_LOW, CAR_MODE_MEDIUM, CAR_MODE_HIGH, CAR_MODE_ON, CAR_MODE_AUTO, CAR_MODE_SMART};
// const uint8_t numCarModes = sizeof(carModes) / sizeof(carModes[0]); // Số lượng chế độ
// uint8_t carModeIndex = 0; // Chỉ số của chế độ hiện tại

// /********************* Các hàm Arduino **************************/
// void setup() {
//     Serial.begin(115200);

//     // Khởi tạo LED (sử dụng để hiển thị trạng thái của Coordinator, không phải quạt)
//     rgbLedWrite(led, 0, 0, 0);

//     // Khởi tạo nút nhấn cho factory reset và điều khiển quạt
//     pinMode(button, INPUT_PULLUP);

//     // Tùy chọn: đặt tên nhà sản xuất và model cho thiết bị
//     zbCarControl.setManufacturerAndModel("Espressif", "ZBCarControl_Coord");

//     // Thêm endpoint vào Zigbee Core
//     Serial.println("Adding ZigbeeCarControl endpoint to Zigbee Core");
//     Zigbee.addEndpoint(&zbCarControl);

//     Serial.println("Đang khởi động Zigbee...");
//     // Khi tất cả các EP được đăng ký, bắt đầu Zigbee ở chế độ COORDINATOR
//     if (!Zigbee.begin(ZIGBEE_COORDINATOR)) {
//         Serial.println("Zigbee khởi động thất bại!");
//         Serial.println("Đang khởi động lại...");
//         ESP.restart();
//     } else {
//         Serial.println("Zigbee đã khởi động thành công!");
//     }

//     // Mở mạng cho phép thiết bị khác tham gia (trong 3 phút = 180 giây)
//     Serial.println("Mở mạng Zigbee trong 3 phút...");
//     Zigbee.setRebootOpenNetwork(180);

//     Serial.println("Đang chờ Car Remote (Server) khả dụng và liên kết...");
//     // Bắt đầu quá trình khám phá và liên kết với Car Control Server (ZigbeeCarRemote)
//     zbCarControl.findCarControlServer();

//     // Chờ cho đến khi điều khiển được liên kết với car server
//     // Đặt thời gian chờ để tránh bị treo vô hạn
//     unsigned long start_time = millis();
//     while (!zbCarControl.isBound()) {
//         Serial.printf("."); // In dấu chấm để hiển thị quá trình chờ
//         delay(500); // Đợi nửa giây
//         if (millis() - start_time > 300000) { // Hết thời gian chờ sau 5 phút (300000 ms)
//             Serial.println("\nLiên kết hết thời gian chờ. Đảm bảo thiết bị quạt (Remote) đã bật nguồn.");
//             // Có thể thử lại việc tìm kiếm/liên kết hoặc factory reset ở đây nếu muốn
//             ESP.restart(); // Khởi động lại để thử lại
//         }
//     }
//     Serial.println("\nĐã liên kết thành công với ZigbeeCarRemote!");
// }

// void loop() {
//     // Xử lý nút nhấn để thay đổi chế độ quạt và gửi lệnh
//     if (digitalRead(button) == LOW) { // Nếu nút nhấn được nhấn (nối GND)
//         // Xử lý debounce cho nút nhấn
//         delay(100); // Chờ 100ms để ổn định
//         int startTime = millis();
//         while (digitalRead(button) == LOW) {
//             delay(50); // Chờ thêm khi nút vẫn được giữ
//             if ((millis() - startTime) > 3000) { // Giữ nút > 3 giây để factory reset
//                 Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
//                 delay(1000);
//                 Zigbee.factoryReset();
//             }
//         }

//         // Nếu nút chỉ được nhấn nhanh (không giữ để reset)
//         if (zbCarControl.isBound()) {
//             // Chuyển đổi qua chế độ quạt tiếp theo
//             carModeIndex = (carModeIndex + 1) % numCarModes;
//             currentControlCarMode = carModes[carModeIndex];

//             Serial.printf("Nút nhấn. Đang gửi lệnh Car Mode: %d (%s)\n", (uint8_t)currentControlCarMode,
//                           (currentControlCarMode == CAR_MODE_OFF) ? "OFF" :
//                           (currentControlCarMode == CAR_MODE_LOW) ? "LOW" :
//                           (currentControlCarMode == CAR_MODE_MEDIUM) ? "MEDIUM" :
//                           (currentControlCarMode == CAR_MODE_HIGH) ? "HIGH" :
//                           (currentControlCarMode == CAR_MODE_ON) ? "ON" :
//                           (currentControlCarMode == CAR_MODE_AUTO) ? "AUTO" : "SMART");

//             // Gửi lệnh thay đổi chế độ quạt qua Zigbee
//             if (zbCarControl.setCarMode(currentControlCarMode)) {
//                 Serial.println("Lệnh chế độ quạt đã được gửi thành công.");
//             } else {
//                 Serial.println("Không gửi được lệnh chế độ quạt.");
//             }
//         } else {
//             Serial.println("Chưa liên kết với Car Remote. Không thể gửi lệnh.");
//             Serial.println("Đang thử tìm kiếm lại Car Remote...");
//             zbCarControl.findCarControlServer(); // Thử tìm kiếm lại
//         }
//     }
//     delay(100); // Độ trễ nhỏ để tránh vòng lặp quá nhanh, giảm tải CPU
// }

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

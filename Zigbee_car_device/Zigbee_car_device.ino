#ifndef ZIGBEE_MODE_ED // Kiểm tra để đảm bảo chế độ Zigbee phù hợp được chọn
#error "Chế độ Zigbee End Device hoặc Router không được chọn trong Tools->Zigbee mode"
#endif

// Bao gồm các file header cần thiết
#include "ZigbeeFanRemote.h"
#include "Zigbee.h" // Thư viện Zigbee Core

/* Cấu hình Zigbee Fan Remote (thiết bị quạt thực tế) */
#define FAN_REMOTE_ENDPOINT_NUMBER   1

#ifdef RGB_BUILTIN // Sử dụng LED RGB tích hợp nếu có, để mô phỏng trạng thái quạt
uint8_t led = RGB_BUILTIN;
#else
uint8_t led = 18; // Hoặc chân GPIO 2 nếu không có RGB_BUILTIN
#endif

uint8_t button = BOOT_PIN; // Nút nhấn cho Factory Reset

// Khởi tạo instance của ZigbeeFanRemote
ZigbeeFanRemote zbFanRemote = ZigbeeFanRemote(FAN_REMOTE_ENDPOINT_NUMBER);

/********************* Hàm Callback điều khiển quạt cục bộ **************************/
// Hàm này sẽ được gọi khi ZigbeeFanRemote nhận lệnh thay đổi chế độ quạt
void controlPhysicalFan(ZigbeeFanMode mode) {
    switch (mode) {
        case FAN_MODE_OFF:
            rgbLedWrite(led, 0, 0, 0); // Tắt LED
            Serial.println("Quạt: TẮT");
            break;
        case FAN_MODE_LOW:
            rgbLedWrite(led, 0, 0, 255); // LED xanh dương cho tốc độ Thấp
            Serial.println("Quạt: Tốc độ THẤP");
            break;
        case FAN_MODE_MEDIUM:
            rgbLedWrite(led, 255, 255, 0); // LED vàng cho tốc độ Trung bình
            Serial.println("Quạt: Tốc độ TRUNG BÌNH");
            break;
        case FAN_MODE_HIGH:
            rgbLedWrite(led, 255, 0, 0); // LED đỏ cho tốc độ Cao
            Serial.println("Quạt: Tốc độ CAO");
            break;
        case FAN_MODE_ON:
            rgbLedWrite(led, 255, 255, 255); // LED trắng cho chế độ BẬT (tùy thuộc tốc độ mặc định)
            Serial.println("Quạt: BẬT (chế độ mặc định)");
            break;
        case FAN_MODE_AUTO:
            rgbLedWrite(led, 0, 255, 0); // LED xanh lá cây cho chế độ Tự động
            Serial.println("Quạt: Chế độ TỰ ĐỘNG");
            break;
        case FAN_MODE_SMART:
            rgbLedWrite(led, 128, 0, 128); // LED tím cho chế độ Thông minh
            Serial.println("Quạt: Chế độ THÔNG MINH");
            break;
        default:
            log_e("Chế độ quạt không xác định: %d", mode);
            break;
    }
    // Ở đây bạn sẽ thêm code để điều khiển động cơ quạt thực tế
    // Ví dụ: điều khiển PWM, relay, v.v.
}

/********************* Các hàm Arduino **************************/
void setup() {
    Serial.begin(115200);

    // Khởi tạo LED để biểu thị trạng thái quạt
    rgbLedWrite(led, 0, 0, 0); // Đảm bảo LED tắt ban đầu

    // Khởi tạo nút nhấn để factory reset
    pinMode(button, INPUT_PULLUP);

    // Tùy chọn: đặt tên nhà sản xuất và model cho thiết bị
    zbFanRemote.setManufacturerAndModel("Espressif", "ZBFanRemote");

    // Đặt chuỗi chế độ quạt mà thiết bị này hỗ trợ
    // Phải khớp với các chế độ mà Coordinator sẽ gửi.
    zbFanRemote.setFanModeSequence(FAN_MODE_SEQUENCE_LOW_MED_HIGH_AUTO);

    // Đăng ký hàm callback sẽ được gọi khi chế độ quạt thay đổi
    zbFanRemote.onFanModeChange(controlPhysicalFan);

    // Thêm endpoint vào Zigbee Core
    Serial.println("Thêm endpoint ZigbeeFanRemote vào Zigbee Core");
    Zigbee.addEndpoint(&zbFanRemote);

    Serial.println("Đang khởi động Zigbee...");
    // Khi tất cả các EP được đăng ký, bắt đầu Zigbee ở chế độ End Device
    if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
        Serial.println("Zigbee khởi động thất bại!");
        Serial.println("Đang khởi động lại...");
        ESP.restart();
    } else {
        Serial.println("Zigbee đã khởi động thành công!");
    }
    Serial.println("Đang kết nối vào mạng Zigbee...");
    // End Device sẽ tìm và kết nối với một Router hoặc Coordinator trong mạng
    while (!Zigbee.connected()) {
        Serial.print(".");
        delay(100);
    }
    Serial.println("\nĐã kết nối vào mạng Zigbee!");
}

void loop() {
    // Kiểm tra nút nhấn để factory reset
    if (digitalRead(button) == LOW) { // Nút nhấn được nhấn (nối với GND)
        // Xử lý debounce
        delay(100);
        int startTime = millis();
        while (digitalRead(button) == LOW) {
            delay(50);
            if ((millis() - startTime) > 3000) {
                // Nếu nút nhấn được giữ hơn 3 giây, factory reset Zigbee và khởi động lại
                Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
                delay(1000);
                Zigbee.factoryReset();
            }
        }
    }
    delay(100);
}
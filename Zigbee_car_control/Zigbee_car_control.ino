// --- Bao gồm các thư viện cần thiết ---
#include "esp_log.h"     // Để sử dụng ESP_LOGx cho logging
#include "driver/gpio.h" // Để sử dụng GPIO
#include "sdkconfig.h"   // Để kiểm tra CONFIG_ZB_ENABLED
#include "Zigbee.h"      // Lớp Zigbee chính
#include "ZigbeeCarControl.h" // Lớp điều khiển xe

#if CONFIG_ZB_ENABLED // Đảm bảo chỉ biên dịch mã Zigbee khi đã bật cấu hình Zigbee

// --- Định nghĩa các hằng số ---
#define ZB_OUR_ENDPOINT         1        // Endpoint của thiết bị Coordinator
#define BOOT_BUTTON_PIN        BOOT_PIN  // Sử dụng nút BOOT
#define DEBOUNCE_DELAY         50       // Thời gian debounce (ms)
#define RESET_HOLD_TIME        3000     // Thời gian giữ nút để reset (ms)
#define TAG_MAIN               "MAIN_APP" // Tag cho logging

// --- Biến toàn cục ---
ZigbeeCarControl *carControl = nullptr; // Instance của ZigbeeCarControl

// --- Hàm setup() của Arduino ---
void setup() {
    // Khởi tạo Serial
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_INFO); // Mức log mặc định
    esp_log_level_set(TAG_MAIN, ESP_LOG_INFO);

    Serial.println("Khoi tao ung dung dieu khien xe Zigbee...");

    // Thiết lập nút BOOT
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

    // Khởi tạo ZigbeeCarControl
    carControl = new ZigbeeCarControl(ZB_OUR_ENDPOINT);
    if (!carControl) {
        Serial.println("Khong the khoi tao ZigbeeCarControl!");
        ESP.restart();
    }

    // Thêm endpoint vào Zigbee Core
    Serial.println("Dang them ZigbeeCarControl endpoint vao Zigbee Core...");
    Zigbee.addEndpoint(carControl);

    // Bắt đầu Zigbee trong chế độ COORDINATOR
    if (!Zigbee.begin(ZIGBEE_COORDINATOR)) {
        Serial.println("Zigbee khong khoi dong duoc!");
        Serial.println("Rebooting...");
        ESP.restart();
    }

    // Chờ kết nối mạng Zigbee
    Serial.println("Dang ket noi mang Zigbee...");
    while (!Zigbee.connected()) {
        Serial.print(".");
        delay(100);
    }
    Serial.println("\nMang Zigbee da ket noi!");

    // Mở mạng cho phép thiết bị tham gia
    Zigbee.openNetwork(180); // Mở mạng trong 180 giây
    Serial.println("Mang Zigbee da mo cho phep thiet bi tham gia trong 180 giay.");

    // Bắt đầu tìm kiếm các ZigbeeCarDevice
    carControl->findCarControlServer();
    Serial.println("Ung dung da san sang. Nhap lenh qua Serial hoac nhan nut BOOT...");
    Serial.println("Lenh Serial: 'on', 'off', 'level <0-254>', 'mode <0-6>'");
    Serial.println("Nut BOOT: Nhan ngan de gui lenh, giu 3s de factory reset.");
}

// --- Hàm loop() của Arduino ---
void loop() {
    static unsigned long last_button_press_time = 0;
    static unsigned long button_press_start_time = 0;
    static bool button_held = false;

    // Kiểm tra kết nối Zigbee
    if (!Zigbee.connected()) {
        Serial.println("Mang Zigbee mat ket noi. Thu ket noi lai...");
        Zigbee.begin(ZIGBEE_COORDINATOR);
        while (!Zigbee.connected()) {
            Serial.print(".");
            delay(100);
        }
        Serial.println("\nDa ket noi lai mang Zigbee!");
        carControl->findCarControlServer(); // Tìm lại thiết bị
    }

    // Xử lý nút BOOT
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
        unsigned long current_time = millis();

        if (!button_held) {
            // Bắt đầu nhấn nút
            button_held = true;
            button_press_start_time = current_time;
        }

        // Kiểm tra nhấn dài để factory reset
        if (current_time - button_press_start_time >= RESET_HOLD_TIME) {
            Serial.println("Resetting Zigbee to factory and rebooting in 1s...");
            delay(1000);
            Zigbee.factoryReset();
            ESP.restart();
        }
    } else if (button_held) {
        // Thả nút
        button_held = false;
        unsigned long press_duration = millis() - button_press_start_time;

        // Nhấn ngắn để gửi lệnh
        if (press_duration < RESET_HOLD_TIME && 
            millis() - last_button_press_time > DEBOUNCE_DELAY) {
            last_button_press_time = millis();
            if (Zigbee.connected() && Zigbee.getRole() == ZIGBEE_COORDINATOR) {
                Serial.println("Nut BOOT nhan ngan! Gui cac lenh mac dinh...");

                // Gửi các lệnh mặc định
                carControl->sendOnOffCommand(true, 0xFFFF, 0xFF); // Broadcast ON
                carControl->sendLevelControlCommand(150, 5, 0xFFFF, 0xFF); // Level 150, 0.5s
                carControl->setCarMode(FAN_MODE_LOW, 0xFFFF, 0xFF); // Fan Mode LOW

                // Cấu hình báo cáo cho tất cả thiết bị đã liên kết
                if (carControl->getBoundDeviceCount() > 0) {
                    for (const auto& device : carControl->getBoundDevices()) {
                        log_i("Cau hinh bao cao cho thiet bi 0x%04x/EP%d", 
                              device.short_addr, device.endpoint);
                        carControl->configureOnOffReporting(device.short_addr, device.endpoint);
                        carControl->configureLevelControlReporting(device.short_addr, device.endpoint);
                        carControl->configureFanModeReporting(device.short_addr, device.endpoint);
                    }
                } else {
                    Serial.println("Chua co thiet bi nao duoc lien ket de cau hinh bao cao.");
                }
                Serial.println("Da gui xong cac lenh mac dinh.");
            } else {
                Serial.println("Mang Zigbee chua san sang de gui lenh.");
            }
        }
    }

    // Xử lý lệnh qua Serial
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (Zigbee.connected() && Zigbee.getRole() == ZIGBEE_COORDINATOR) {
            if (input.equalsIgnoreCase("on")) {
                carControl->sendOnOffCommand(true, 0xFFFF, 0xFF);
                Serial.println("Da gui lenh ON.");
            } else if (input.equalsIgnoreCase("off")) {
                carControl->sendOnOffCommand(false, 0xFFFF, 0xFF);
                Serial.println("Da gui lenh OFF.");
            } else if (input.startsWith("level ")) {
                int level = input.substring(6).toInt();
                if (level >= 0 && level <= 254) {
                    carControl->sendLevelControlCommand(level, 5, 0xFFFF, 0xFF);
                    Serial.printf("Da gui lenh Level Control: %d\n", level);
                } else {
                    Serial.println("Level phai tu 0 den 254!");
                }
            } else if (input.startsWith("mode ")) {
                int mode = input.substring(5).toInt();
                if (mode >= FAN_MODE_OFF && mode <= FAN_MODE_SMART) {
                    carControl->setCarMode((ZigbeeFanMode)mode, 0xFFFF, 0xFF);
                    Serial.printf("Da gui lenh Fan Mode: %d\n", mode);
                } else {
                    Serial.println("Mode phai tu 0 (OFF) den 6 (SMART)!");
                }
            } else {
                Serial.println("Lenh khong hop le! Dung: 'on', 'off', 'level <0-254>', 'mode <0-6>'");
            }
        } else {
            Serial.println("Mang Zigbee chua san sang de gui lenh.");
        }
    }

    delay(10); // Delay nhỏ để tránh CPU chạy 100%
}

#else // CONFIG_ZB_ENABLED is not defined
void setup() {
    Serial.begin(115200);
    Serial.println("Zigbee is NOT enabled in sdkconfig. Please enable it.");
}
void loop() {
    // Do nothing
}
#endif // CONFIG_ZB_ENABLED
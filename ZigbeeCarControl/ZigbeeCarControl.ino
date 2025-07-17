#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "Zigbee.h"
#include "ZigbeeCarControl.h"

#if CONFIG_ZB_ENABLED

#define ZB_OUR_ENDPOINT         1        // Endpoint của Coordinator
#define BOOT_BUTTON_PIN        BOOT_PIN  // Sử dụng nút BOOT
#define DEBOUNCE_DELAY         50       // Thời gian debounce (ms)
#define RESET_HOLD_TIME        3000     // Thời gian giữ nút để reset (ms)
#define SIMULATION_INTERVAL    30000    // Gửi lệnh giả lập mỗi 30 giây
#define TAG_MAIN               "MAIN_APP"

ZigbeeCarControl *carControl = nullptr;

void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_INFO);
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

    // Mở mạng vô thời hạn để thiết bị tham gia
    Zigbee.openNetwork(0xFFFF); // 0xFFFF = vô hạn
    Serial.println("Mang Zigbee da mo vo thoi han cho phep thiet bi tham gia.");

    // Bắt đầu tìm kiếm các ZigbeeCarDevice
    carControl->findCarControlServer();
    Serial.println("Ung dung da san sang. Nhap lenh qua Serial hoac nhan nut BOOT...");
    Serial.println("Lenh Serial: 'on', 'off', 'level <0-254>', 'mode <0-6>'");
    Serial.println("Nut BOOT: Nhan ngan de gui lenh, giu 3s de factory reset.");
    Serial.println("Bat dau gui lenh gia lap moi 30 giay...");
}

void loop() {
    static unsigned long last_button_press_time = 0;
    static unsigned long button_press_start_time = 0;
    static bool button_held = false;
    static unsigned long last_simulation_time = 0;
    static int simulation_step = 0;

    // Kiểm tra kết nối Zigbee
    if (!Zigbee.connected()) {
        Serial.println("Mang Zigbee mat ket noi. Thu ket noi lai...");
        Zigbee.begin(ZIGBEE_COORDINATOR);
        while (!Zigbee.connected()) {
            Serial.print(".");
            delay(100);
        }
        Serial.println("\nDa ket noi lai mang Zigbee!");
        Zigbee.openNetwork(0xFFFF); // Mở lại mạng vô thời hạn
        carControl->findCarControlServer(); // Tìm lại thiết bị
    }

    // In danh sách thiết bị đã liên kết (để debug)
    static unsigned long last_bound_check = 0;
    if (millis() - last_bound_check >= 10000) { // Kiểm tra mỗi 10 giây
        if (carControl->getBoundDeviceCount() > 0) {
            Serial.println("Danh sach thiet bi da lien ket:");
            for (const auto& device : carControl->getBoundDevices()) {
                Serial.printf(" - Thiet bi 0x%04x, Endpoint %d\n", device.short_addr, device.endpoint);
            }
        } else {
            Serial.println("Chua co thiet bi nao duoc lien ket.");
            carControl->findCarControlServer(); // Thử tìm lại
        }
        last_bound_check = millis();
    }

    // Giả lập gửi lệnh
    if (millis() - last_simulation_time >= SIMULATION_INTERVAL) {
        if (Zigbee.connected() && Zigbee.getRole() == ZIGBEE_COORDINATOR) {
            Serial.println("Gui lenh gia lap...");
            switch (simulation_step) {
                case 0:
                    carControl->sendOnOffCommand(true, 0xFFFF, 0xFF); // ON
                    Serial.println("Da gui lenh ON (broadcast).");
                    break;
                case 1:
                    carControl->sendLevelControlCommand(100, 5, 0xFFFF, 0xFF); // Level 100
                    Serial.println("Da gui lenh Level Control: 100");
                    break;
                case 2:
                    carControl->setCarMode(FAN_MODE_LOW, 0xFFFF, 0xFF); // Fan Mode LOW
                    Serial.println("Da gui lenh Fan Mode: LOW (1)");
                    break;
                case 3:
                    carControl->sendLevelControlCommand(200, 5, 0xFFFF, 0xFF); // Level 200
                    Serial.println("Da gui lenh Level Control: 200");
                    break;
                case 4:
                    carControl->setCarMode(FAN_MODE_HIGH, 0xFFFF, 0xFF); // Fan Mode HIGH
                    Serial.println("Da gui lenh Fan Mode: HIGH (3)");
                    break;
                case 5:
                    carControl->sendOnOffCommand(false, 0xFFFF, 0xFF); // OFF
                    Serial.println("Da gui lenh OFF (broadcast).");
                    break;
            }
            simulation_step = (simulation_step + 1) % 6; // Lặp lại sau 6 bước
            last_simulation_time = millis();

            // Cấu hình báo cáo cho các thiết bị đã liên kết
            if (carControl->getBoundDeviceCount() > 0) {
                for (const auto& device : carControl->getBoundDevices()) {
                    Serial.printf("Cau hinh bao cao cho thiet bi 0x%04x/EP%d\n", 
                                  device.short_addr, device.endpoint);
                    carControl->configureOnOffReporting(device.short_addr, device.endpoint);
                    carControl->configureLevelControlReporting(device.short_addr, device.endpoint);
                    carControl->configureFanModeReporting(device.short_addr, device.endpoint);
                }
            }
        } else {
            Serial.println("Mang Zigbee chua san sang de gui lenh gia lap.");
        }
    }

    // Xử lý nút BOOT
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
        unsigned long current_time = millis();

        if (!button_held) {
            button_held = true;
            button_press_start_time = current_time;
        }

        if (current_time - button_press_start_time >= RESET_HOLD_TIME) {
            Serial.println("Resetting Zigbee to factory and rebooting in 1s...");
            delay(1000);
            Zigbee.factoryReset();
            ESP.restart();
        }
    } else if (button_held) {
        button_held = false;
        unsigned long press_duration = millis() - button_press_start_time;

        if (press_duration < RESET_HOLD_TIME && 
            millis() - last_button_press_time > DEBOUNCE_DELAY) {
            last_button_press_time = millis();
            if (Zigbee.connected() && Zigbee.getRole() == ZIGBEE_COORDINATOR) {
                Serial.println("Nut BOOT nhan ngan! Gui cac lenh mac dinh...");
                carControl->sendOnOffCommand(true, 0xFFFF, 0xFF);
                carControl->sendLevelControlCommand(150, 5, 0xFFFF, 0xFF);
                carControl->setCarMode(FAN_MODE_LOW, 0xFFFF, 0xFF);

                if (carControl->getBoundDeviceCount() > 0) {
                    for (const auto& device : carControl->getBoundDevices()) {
                        Serial.printf("Cau hinh bao cao cho thiet bi 0x%04x/EP%d\n", 
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

    delay(10);
}

#else
void setup() {
    Serial.begin(115200);
    Serial.println("Zigbee is NOT enabled in sdkconfig. Please enable it.");
}
void loop() {}
#endif // CONFIG_ZB_ENABLED
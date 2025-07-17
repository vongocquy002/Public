#include "esp_log.h"
#include "sdkconfig.h"
#include "Zigbee.h"
#include "ZigbeeCarDevice.h"

#if CONFIG_ZB_ENABLED

#define ZB_DEVICE_ENDPOINT 10 // Endpoint của End Device
#define TAG_MAIN "CAR_DEVICE"
#define MAX_RETRIES 5        // Số lần thử khởi động Zigbee
#define RETRY_DELAY 10000    // Delay giữa các lần thử (ms)
#define INITIAL_DELAY 5000   // Delay ban đầu sau khi kết nối (ms)

ZigbeeCarDevice *carDevice = nullptr;

void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG_MAIN, ESP_LOG_INFO);

    Serial.println("Khoi tao ZigbeeCarDevice (End Device)...");

    // Khởi tạo ZigbeeCarDevice
    carDevice = new ZigbeeCarDevice(ZB_DEVICE_ENDPOINT);
    if (!carDevice) {
        Serial.println("Khong the khoi tao ZigbeeCarDevice!");
        ESP.restart();
    }

    // Thêm endpoint vào Zigbee Core
    Serial.println("Dang them endpoint vao Zigbee Core...");
    Zigbee.addEndpoint(carDevice);

    // Thử khởi động Zigbee
    int retries = 0;
    bool started = false;
    while (retries < MAX_RETRIES && !started) {
        Serial.printf("Thu khoi dong Zigbee lan %d/%d...\n", retries + 1, MAX_RETRIES);
        started = Zigbee.begin(ZIGBEE_END_DEVICE);
        if (!started) {
            Serial.println("Zigbee khong khoi dong duoc! Kiem tra cau hinh hoac phan cung.");
            retries++;
            delay(RETRY_DELAY);
        }
    }

    if (!started) {
        Serial.println("Het so lan thu. Rebooting...");
        ESP.restart();
    }

    // Chờ kết nối mạng Zigbee
    Serial.println("Dang ket noi mang Zigbee... (Toi da 60 giay)");
    unsigned long start_time = millis();
    while (!Zigbee.connected() && millis() - start_time < 60000) {
        Serial.print(".");
        delay(100);
    }

    if (Zigbee.connected()) {
        Serial.println("\nMang Zigbee da ket noi!");
        Serial.println("ZigbeeCarDevice da san sang nhan lenh tu Coordinator.");
        // Delay để đảm bảo mạng ổn định
        Serial.println("Waiting for network stabilization...");
        delay(INITIAL_DELAY);
    } else {
        Serial.println("\nKhong the ket noi mang Zigbee!");
        Serial.println("Nhap 'reset' qua Serial de factory reset Zigbee.");
    }
}

void loop() {
    // Kiểm tra kết nối Zigbee
    if (!Zigbee.connected()) {
        Serial.println("Mang Zigbee mat ket noi. Thu ket noi lai...");
        int retries = 0;
        bool started = false;
        while (retries < MAX_RETRIES && !started) {
            Serial.printf("Thu khoi dong lai Zigbee lan %d/%d...\n", retries + 1, MAX_RETRIES);
            started = Zigbee.begin(ZIGBEE_END_DEVICE);
            if (!started) {
                Serial.println("Zigbee khong khoi dong duoc! Kiem tra cau hinh hoac phan cung.");
                retries++;
                delay(RETRY_DELAY);
            }
        }

        if (!started) {
            Serial.println("Het so lan thu. Rebooting...");
            ESP.restart();
        }

        Serial.println("Dang ket noi lai mang Zigbee... (Toi da 60 giay)");
        unsigned long start_time = millis();
        while (!Zigbee.connected() && millis() - start_time < 60000) {
            Serial.print(".");
            delay(100);
        }

        if (Zigbee.connected()) {
            Serial.println("\nDa ket noi lai mang Zigbee!");
            Serial.println("Waiting for network stabilization...");
            delay(INITIAL_DELAY);
        } else {
            Serial.println("\nKhong the ket noi lai mang Zigbee!");
            Serial.println("Nhap 'reset' qua Serial de factory reset Zigbee.");
        }
    }

    // Xử lý lệnh Serial (cho factory reset)
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.equalsIgnoreCase("reset")) {
            Serial.println("Resetting Zigbee to factory and rebooting in 1s...");
            delay(1000);
            Zigbee.factoryReset();
            ESP.restart();
        } else {
            Serial.println("Lenh khong hop le! Dung: 'reset' de factory reset Zigbee.");
        }
    }

    // In trạng thái hiện tại định kỳ (mỗi 10 giây)
    static unsigned long last_report_time = 0;
    if (millis() - last_report_time >= 10000) {
        Serial.println("Checking state...");
        if (carDevice && Zigbee.connected()) {
            Serial.println("carDevice is valid, getting state...");
            const ZigbeeCarDeviceState& state = carDevice->getState();
            Serial.printf("Trang thai hien tai: On/Off=%s, Level=%d, Fan Mode=%d\n",
                          state.on_off_state ? "ON" : "OFF", state.level, state.fan_mode);
            // Tạm thời vô hiệu hóa sendStateReport để kiểm tra
            // Serial.println("Sending state report...");
            // if (!carDevice->sendStateReport()) {
            //     Serial.println("Failed to send state report!");
            // }
        } else {
            Serial.println("carDevice is null or Zigbee not connected!");
        }
        last_report_time = millis();
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
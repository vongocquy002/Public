#include "esp_log.h"
#include "sdkconfig.h"
#include "Zigbee.h"
#include "ZigbeeCarDevice.h"

#if CONFIG_ZB_ENABLED

#define ZB_DEVICE_ENDPOINT 10 // Endpoint của End Device
#define TAG_MAIN "CAR_DEVICE"

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
    Zigbee.addEndpoint(carDevice);

    // Bắt đầu Zigbee trong chế độ END_DEVICE
    if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
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

    Serial.println("ZigbeeCarDevice da san sang nhan lenh tu Coordinator.");
}

void loop() {
    // Kiểm tra kết nối Zigbee
    if (!Zigbee.connected()) {
        Serial.println("Mang Zigbee mat ket noi. Thu ket noi lai...");
        Zigbee.begin(ZIGBEE_END_DEVICE);
        while (!Zigbee.connected()) {
            Serial.print(".");
            delay(100);
        }
        Serial.println("\nDa ket noi lai mang Zigbee!");
    }

    // In trạng thái hiện tại định kỳ (mỗi 10 giây)
    static unsigned long last_report_time = 0;
    if (millis() - last_report_time >= 10000) {
        if (carDevice) {
            const ZigbeeCarDeviceState& state = carDevice->getState();
            Serial.printf("Trang thai hien tai: On/Off=%s, Level=%d, Fan Mode=%d\n",
                          state.on_off_state ? "ON" : "OFF", state.level, state.fan_mode);
            carDevice->sendStateReport(); // Gửi báo cáo định kỳ
        }
        last_report_time = millis();
    }

    delay(10); // Delay nhỏ để tránh CPU chạy 100%
}

#else
void setup() {
    Serial.begin(115200);
    Serial.println("Zigbee is NOT enabled in sdkconfig. Please enable it.");
}
void loop() {}
#endif // CONFIG_ZB_ENABLED
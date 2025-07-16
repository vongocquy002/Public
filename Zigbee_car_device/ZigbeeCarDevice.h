#pragma once
#include "soc/soc_caps.h"
#include "sdkconfig.h"
#if CONFIG_ZB_ENABLED
#include "ZigbeeEP.h"
#include "ha/esp_zigbee_ha_standard.h" // Cần để có các định nghĩa ESP_ZB_ZCL_FAN_CONTROL_*

// Các enum tùy chỉnh, thân thiện với Arduino cho các giá trị chế độ quạt
enum ZigbeeFanMode {
    FAN_MODE_OFF = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_OFF,
    FAN_MODE_LOW = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW,
    FAN_MODE_MEDIUM = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_MEDIUM,
    FAN_MODE_HIGH = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH,
    FAN_MODE_ON = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_ON,
    FAN_MODE_AUTO = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_AUTO,
    FAN_MODE_SMART = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SMART,
};

// Các enum tùy chỉnh, thân thiện với Arduino cho chuỗi chế độ quạt
enum ZigbeeFanModeSequence {
    FAN_MODE_SEQUENCE_LOW_MED_HIGH = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH,
    FAN_MODE_SEQUENCE_LOW_HIGH = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_HIGH,
    FAN_MODE_SEQUENCE_LOW_MED_HIGH_AUTO = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH_AUTO,
    FAN_MODE_SEQUENCE_LOW_HIGH_AUTO = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_HIGH_AUTO,
    FAN_MODE_SEQUENCE_ON_AUTO = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_ON_AUTO,
};

class ZigbeeFanRemote : public ZigbeeEP {
public:
    /**
     * @brief Hàm tạo cho ZigbeeFanRemote.
     * @param endpoint Số endpoint Zigbee cho thiết bị này.
     */
    ZigbeeFanRemote(uint8_t endpoint);
    ~ZigbeeFanRemote() {}

    /**
     * @brief Đặt chuỗi chế độ quạt mà thiết bị này hỗ trợ.
     * Hàm này nên được gọi trong quá trình khởi tạo để cấu hình quạt.
     * @param sequence Chuỗi chế độ quạt mong muốn (ví dụ: LOW_MED_HIGH).
     * @return True nếu thành công, false nếu ngược lại.
     */
    bool setFanModeSequence(ZigbeeFanModeSequence sequence);

    /**
     * @brief Lấy chế độ quạt hiện tại của thiết bị.
     * @return Chế độ quạt hiện tại.
     */
    ZigbeeFanMode getFanMode() const {
        return _current_fan_mode;
    }

    /**
     * @brief Lấy chuỗi chế độ quạt hiện tại của thiết bị.
     * @return Chuỗi chế độ quạt hiện tại.
     */
    ZigbeeFanModeSequence getFanModeSequence() const {
        return _current_fan_mode_sequence;
    }

    /**
     * @brief Đăng ký hàm callback sẽ được gọi khi chế độ quạt thay đổi
     * do lệnh Zigbee từ một Client.
     * @param callback Con trỏ hàm sẽ được gọi, nhận ZigbeeFanMode mới.
     */
    void onFanModeChange(void (*callback)(ZigbeeFanMode mode)) {
        _on_fan_mode_change = callback;
    }

private:
    /**
     * @brief Ghi đè hàm xử lý khi một thuộc tính ZCL được thiết lập (ghi) bởi một Client.
     * Đây là nơi FanRemote nhận lệnh thay đổi chế độ quạt.
     * @param message Con trỏ tới thông điệp thiết lập thuộc tính ZCL.
     */
    void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) override;

    /**
     * @brief Hàm nội bộ được gọi để thông báo về sự thay đổi chế độ quạt.
     * Nó sẽ kích hoạt callback đã đăng ký.
     */
    void fanModeChanged();

    // Biến lưu trữ chế độ quạt hiện tại
    ZigbeeFanMode _current_fan_mode;
    // Biến lưu trữ chuỗi chế độ quạt hiện tại
    ZigbeeFanModeSequence _current_fan_mode_sequence;
    // Con trỏ hàm cho callback khi chế độ quạt thay đổi
    void (*_on_fan_mode_change)(ZigbeeFanMode mode);
};

#endif // CONFIG_ZB_ENABLED
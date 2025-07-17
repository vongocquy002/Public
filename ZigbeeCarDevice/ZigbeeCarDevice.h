#pragma once

#include "soc/soc_caps.h"
#include "sdkconfig.h"
#if CONFIG_ZB_ENABLED

#include "ZigbeeEP.h"
#include "ha/esp_zigbee_ha_standard.h"

#ifndef ZB_IEEE_ADDR_SIZE
#define ZB_IEEE_ADDR_SIZE 8
#endif

// Enum cho Fan Mode, đồng bộ với ZigbeeCarControl
enum ZigbeeFanMode {
    FAN_MODE_OFF = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_OFF,
    FAN_MODE_LOW = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW,
    FAN_MODE_MEDIUM = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_MEDIUM,
    FAN_MODE_HIGH = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH,
    FAN_MODE_ON = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_ON,
    FAN_MODE_AUTO = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_AUTO,
    FAN_MODE_SMART = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SMART,
};

// Cấu trúc lưu trạng thái thiết bị
struct ZigbeeCarDeviceState {
    bool on_off_state;         // Trạng thái On/Off
    uint8_t level;             // Mức độ (PWM, 0-254)
    ZigbeeFanMode fan_mode;    // Chế độ quạt
    uint8_t endpoint;          // Endpoint của thiết bị
    uint8_t ieee_addr[ZB_IEEE_ADDR_SIZE]; // Địa chỉ IEEE
};

class ZigbeeCarDevice : public ZigbeeEP {
public:
    /**
     * @brief Constructor cho ZigbeeCarDevice.
     * @param endpoint Số endpoint Zigbee cho thiết bị này.
     */
    ZigbeeCarDevice(uint8_t endpoint);

    /**
     * @brief Destructor.
     */
    ~ZigbeeCarDevice() {}

    /**
     * @brief Lấy trạng thái hiện tại của thiết bị.
     * @return Tham chiếu tới trạng thái thiết bị.
     */
    const ZigbeeCarDeviceState& getState() const { return _state; }

    /**
     * @brief Xử lý cập nhật thuộc tính ZCL (ghi đè phương thức ảo từ ZigbeeEP).
     * @param message Thông điệp cập nhật thuộc tính.
     */
    virtual void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message);

    /**
     * @brief Gửi báo cáo trạng thái (On/Off, Level, Fan Mode) về Coordinator.
     * @return True nếu báo cáo được gửi thành công, false nếu ngược lại.
     */
    bool sendStateReport();

private:
    ZigbeeCarDeviceState _state; // Trạng thái hiện tại của thiết bị

    /**
     * @brief Cập nhật trạng thái và gửi báo cáo nếu cần.
     * @param cluster_id ID của cluster.
     * @param attr_id ID của thuộc tính.
     * @param value Giá trị mới của thuộc tính.
     */
    void updateStateAndReport(uint16_t cluster_id, uint16_t attr_id, void *value);
};

#endif // CONFIG_ZB_ENABLED
/* Class of Zigbee Pressure sensor endpoint inherited from common EP class */

#pragma once

#include "soc/soc_caps.h"
#include "sdkconfig.h"
#if CONFIG_ZB_ENABLED

#include "ZigbeeEP.h"
#include "ha/esp_zigbee_ha_standard.h"

// Custom Arduino-friendly enums for fan mode values
enum ZigbeeFanMode {
    FAN_MODE_OFF = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_OFF,
    FAN_MODE_LOW = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW,
    FAN_MODE_MEDIUM = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_MEDIUM,
    FAN_MODE_HIGH = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH,
    FAN_MODE_ON = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_ON,
    FAN_MODE_AUTO = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_AUTO,
    FAN_MODE_SMART = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SMART,
};
enum ZigbeeFanModeSequence {
    FAN_MODE_SEQUENCE_LOW_MED_HIGH = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH,
    FAN_MODE_SEQUENCE_LOW_HIGH = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_HIGH,
    FAN_MODE_SEQUENCE_LOW_MED_HIGH_AUTO = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH_AUTO,
    FAN_MODE_SEQUENCE_ON_AUTO = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_ON_AUTO,
};
struct ZigbeeCarDeviceState {
    zb_device_params_t params; 
    uint8_t motor_level;
    bool motor_on_off_state;
    ZigbeeFanMode gpio_mode;
};

class ZigbeeCarControl : public ZigbeeEP {
public:
    /**
     * @brief Hàm tạo cho ZigbeeFanControl.
     * @param endpoint Số endpoint Zigbee cho thiết bị này.
     */
    ZigbeeCarControl(uint8_t endpoint);

    /**
     * @brief Hàm callback tĩnh cho các yêu cầu liên kết ZDO.
     * Được sử dụng nội bộ bởi stack Zigbee khi cố gắng liên kết.
     * @param zdo_status Trạng thái của hoạt động ZDO.
     * @param user_ctx Con trỏ ngữ cảnh người dùng (trỏ tới instance này).
     */
    static void bindCbWrapper(esp_zb_zdp_status_t zdo_status, void *user_ctx);

    /**
     * @brief Hàm callback tĩnh cho các yêu cầu tìm kiếm ZDO (Phản hồi Mô tả Khớp).
     * Được sử dụng nội bộ bởi stack Zigbee khi tìm thấy một endpoint khớp.
     * @param zdo_status Trạng thái của hoạt động ZDO.
     * @param addr Địa chỉ ngắn của thiết bị tìm thấy.
     * @param endpoint Số endpoint của thiết bị tìm thấy.
     * @param user_ctx Con trỏ ngữ cảnh người dùng (trỏ tới instance này).
     */
    static void findCbWrapper(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx);

    /**
     * @brief Tìm kiếm một Fan Control Server Zigbee trong mạng.
     * Hàm này bắt đầu quá trình khám phá.
     */
    void findCarControlServer();

    /**
     * @brief Đặt chế độ quạt trên một Fan Control Server đã được liên kết.
     * @param mode Chế độ quạt mong muốn (FAN_MODE_OFF, FAN_MODE_LOW, FAN_MODE_MEDIUM, FAN_MODE_HIGH, FAN_MODE_ON, FAN_MODE_AUTO, FAN_MODE_SMART).
     * @return True nếu lệnh được gửi thành công, false nếu ngược lại.
     */
    bool setCarMode(ZigbeeFanMode mode, uint16_t target_addr = 0xFFFF, uint8_t target_ep = 0xFF);

    /**
     * @brief Gửi lệnh On hoặc Off tới một thiết bị/motor/GPIO cụ thể hoặc tất cả các thiết bị đã liên kết
     * thông qua On/Off Cluster.
     * @param on True để bật (On), false để tắt (Off).
     * @param target_addr Địa chỉ ngắn của thiết bị mục tiêu (0xFFFF để gửi broadcast cho tất cả đã liên kết).
     * @param target_ep Endpoint của thiết bị mục tiêu chứa On/Off Server (chỉ dùng khi target_addr không phải 0xFFFF).
     * @return True nếu lệnh được gửi thành công ít nhất một lần, false nếu không gửi được lệnh nào.
     */
    bool sendOnOffCommand(bool on, uint16_t target_addr = 0xFFFF, uint8_t target_ep = 0xFF);

    /**
     * @brief Đặt mức độ (ví dụ: tốc độ motor) cho một thiết bị cụ thể hoặc tất cả các thiết bị đã liên kết
     * thông qua Level Control Cluster.
     * @param level Giá trị mức mong muốn (0-254).
     * @param transition_time Thời gian chuyển đổi đến mức mới theo đơn vị 1/10 giây (0 để chuyển ngay lập tức).
     * @param target_addr Địa chỉ ngắn của thiết bị mục tiêu (0xFFFF cho tất cả đã liên kết).
     * @param target_ep Endpoint của thiết bị mục tiêu chứa Level Control Server (chỉ dùng khi target_addr không phải 0xFFFF).
     * @return True nếu lệnh được gửi thành công ít nhất một lần, false nếu không gửi được lệnh nào.
     */
    bool sendLevelControlCommand(uint8_t level, uint16_t transition_time = 0, uint16_t target_addr = 0xFFFF, uint8_t target_ep = 0xFF);

    /**
     * @brief Cấu hình báo cáo thuộc tính On/Off cho một thiết bị cụ thể.
     * Điều này yêu cầu ZigbeeCarDevice báo cáo trạng thái On/Off của nó (ví dụ: motor chính hoặc GPIO).
     * @param target_addr Địa chỉ ngắn của thiết bị ZigbeeCarDevice.
     * @param target_ep Endpoint của ZigbeeCarDevice chứa On/Off Cluster cần báo cáo.
     * @return True nếu yêu cầu cấu hình được gửi thành công, false nếu ngược lại.
     */
    bool configureOnOffReporting(uint16_t target_addr, uint8_t target_ep);

    /**
     * @brief Cấu hình báo cáo thuộc tính Level Control cho một thiết bị cụ thể.
     * Điều này yêu cầu ZigbeeCarDevice báo cáo mức độ hiện tại của nó (ví dụ: tốc độ motor).
     * @param target_addr Địa chỉ ngắn của thiết bị ZigbeeCarDevice.
     * @param target_ep Endpoint của ZigbeeCarDevice chứa Level Control Cluster cần báo cáo.
     * @return True nếu yêu cầu cấu hình được gửi thành công, false nếu ngược lại.
     */
    bool configureLevelControlReporting(uint16_t target_addr, uint8_t target_ep);

    /**
     * @brief Cấu hình báo cáo thuộc tính Fan Mode cho một thiết bị cụ thể.
     * Điều này yêu cầu ZigbeeCarDevice báo cáo chế độ hoạt động hiện tại của GPIO.
     * @param target_addr Địa chỉ ngắn của thiết bị ZigbeeCarDevice.
     * @param target_ep Endpoint của ZigbeeCarDevice chứa Fan Control Cluster cần báo cáo.
     * @return True nếu yêu cầu cấu hình được gửi thành công, false nếu ngược lại.
     */
    bool configureFanModeReporting(uint16_t target_addr, uint8_t target_ep);

    /**
     * @brief Kiểm tra xem điều khiển có hiện được liên kết với một fan control server hay không.
     * @return True nếu được liên kết, false nếu ngược lại.
     */
    bool isBound() const { return _is_bound; }

    /**
     * @brief Lấy số lượng thiết bị Fan Control Server đã liên kết.
     * @return Số lượng thiết bị Fan Control Server đã liên kết.
     */
    size_t getBoundDeviceCount() const { return _bound_fan_servers.size(); }

    /**
     * @brief Trả về danh sách các thiết bị ZigbeeCarDevice đã liên kết.
     * @return Tham chiếu hằng đến vector chứa thông tin các thiết bị đã liên kết.
     */
    const std::vector<ZigbeeCarDeviceState>& getBoundDevices() const { return _bound_fan_servers; }

private:

    std::vector<ZigbeeCarDeviceState> _bound_fan_servers;
    static ZigbeeCarControl *_instance;
   
    void bindCb(esp_zb_zdp_status_t zdo_status, void *user_ctx);
    void findCb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx);

};

#endif // CONFIG_ZB_ENABLED
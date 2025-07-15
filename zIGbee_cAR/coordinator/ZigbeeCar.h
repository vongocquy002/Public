#pragma once

#include "soc/soc_caps.h"
#include "sdkconfig.h"

#if CONFIG_ZB_ENABLED

#include "ZigbeeEP.h"
#include "ha/esp_zigbee_ha_standard.h" // Cần cho ESP_ZB_ZCL_CLUSTER_ID_ON_OFF và các định nghĩa khác
// Nếu esp_zb_core.h là cần thiết cho các kiểu dữ liệu như ZB_ZCL_ATTR_TYPE_BOOL, v.v.,
// và nó không gây lỗi include, hãy thêm nó vào.
// Lỗi redefinition vừa rồi cho thấy esp_zigbee_type.h đã được include, và nó chứa các định nghĩa này.
// Vậy, tôi sẽ giả định các định nghĩa kiểu dữ liệu ZCL nằm trong đó.

// Định nghĩa ID Cluster tùy chỉnh cho Điều khiển Động cơ (thay thế bằng ID tùy chỉnh mong muốn của bạn)
#define ZB_ZCL_CLUSTER_ID_MOTOR_CONTROL 0xFC01
// Định nghĩa ID Cluster tùy chỉnh cho Điều khiển GPIO
#define ZB_ZCL_CLUSTER_ID_GPIO_CONTROL  0xFC02

// --- Các Thuộc tính của Cluster Điều khiển Động cơ ---
#define ZB_ZCL_ATTR_MOTOR_ROTATION_MODE_ID 0x0000
// Sử dụng các định nghĩa từ thư viện nếu có, nếu không, dùng giá trị số.
// Vì esp_zigbee_type.h được include, các kiểu dữ liệu này có thể đã có sẵn.
#define ZB_ZCL_ATTR_TYPE_MOTOR_ROTATION_MODE ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM // Hoặc 0x30 nếu ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM không có

#define ZB_ZCL_ATTR_MOTOR_PWM_SPEED_ID 0x0001
#define ZB_ZCL_ATTR_TYPE_MOTOR_PWM_SPEED ESP_ZB_ZCL_ATTR_TYPE_U8 // Hoặc 0x20 nếu ESP_ZB_ZCL_ATTR_TYPE_U8 không có

// --- Các Thuộc tính của Cluster Điều khiển GPIO ---
#define ZB_ZCL_ATTR_GPIO1_STATE_ID 0x0000
#define ZB_ZCL_ATTR_GPIO2_STATE_ID 0x0001
#define ZB_ZCL_ATTR_GPIO3_STATE_ID 0x0002
#define ZB_ZCL_ATTR_GPIO4_STATE_ID 0x0003
#define ZB_ZCL_ATTR_GPIO5_STATE_ID 0x0004
#define ZB_ZCL_ATTR_GPIO6_STATE_ID 0x0005
#define ZB_ZCL_ATTR_TYPE_GPIO_STATE ESP_ZB_ZCL_ATTR_TYPE_BOOL // Hoặc 0x10 nếu ESP_ZB_ZCL_ATTR_TYPE_BOOL không có


class ZigbeeCar : public ZigbeeEP {
public:
    /**
     * @brief Hàm tạo cho endpoint ZigbeeCar.
     * @param endpoint Số endpoint Zigbee cho chiếc xe này.
     */
    ZigbeeCar(uint8_t endpoint = 10);
    ~ZigbeeCar() {}

    // --- Điều khiển Bật/Tắt ---
    /**
     * @brief Thiết lập hàm callback được gọi khi trạng thái BẬT/TẮT của xe thay đổi.
     * @param callback Con trỏ tới một hàm nhận đối số boolean (true cho BẬT, false cho TẮT).
     */
    void onMainOnOffChange(void (*callback)(bool)) {
        _on_main_on_off_change_cb = callback;
    }
    /**
     * @brief Lấy trạng thái BẬT/TẮT hiện tại của xe.
     * @return True nếu xe đang BẬT, false nếu TẮT.
     */
    bool getCarOnOffState() {
        return _current_on_off_state;
    }

    // --- Điều khiển Động cơ ---
    /**
     * @brief Thiết lập hàm callback được gọi khi chế độ quay của động cơ thay đổi.
     * @param callback Con trỏ tới một hàm nhận đối số uint8_t (chế độ quay).
     */
    void onRotationModeChange(void (*callback)(uint8_t)) {
        _on_rotation_mode_change_cb = callback;
    }
    /**
     * @brief Thiết lập hàm callback được gọi khi tốc độ PWM của động cơ thay đổi.
     * @param callback Con trỏ tới một hàm nhận đối số uint8_t (giá trị tốc độ 0-255).
     */
    void onPwmSpeedChange(void (*callback)(uint8_t)) {
        _on_pwm_speed_change_cb = callback;
    }
    /**
     * @brief Lấy chế độ quay động cơ hiện tại.
     * @return Chế độ quay hiện tại (uint8_t).
     */
    uint8_t getRotationMode() {
        return _current_rotation_mode;
    }
    /**
     * @brief Lấy tốc độ PWM động cơ hiện tại.
     * @return Tốc độ PWM hiện tại (uint8_t, 0-255).
     */
    uint8_t getPwmSpeed() {
        return _current_pwm_speed;
    }

    // --- Điều khiển GPIO ---
    /**
     * @brief Thiết lập hàm callback được gọi khi trạng thái của bất kỳ GPIO nào thay đổi.
     * @param callback Con trỏ tới một hàm nhận uint8_t (chỉ số GPIO 1-6) và boolean (trạng thái) làm đối số.
     */
    void onGpioChange(void (*callback)(uint8_t, bool)) {
        _on_gpio_change_cb = callback;
    }
    /**
     * @brief Lấy trạng thái hiện tại của một GPIO cụ thể.
     * @param gpio_index Chỉ số của GPIO (từ 1 đến 6).
     * @return True nếu GPIO ở mức CAO, false nếu THẤP. Trả về false nếu chỉ số nằm ngoài giới hạn.
     */
    bool getGpioState(uint8_t gpio_index) {
        if (gpio_index >= 1 && gpio_index <= 6) {
            return _current_gpio_states[gpio_index - 1];
        }
        return false;
    }

    // Hàm để khởi tạo các cluster và gán cho endpoint.
    void initClusters();

private:
    /**
     * @brief Ghi đè hàm xử lý thiết lập thuộc tính của lớp cơ sở.
     * Hàm này được gọi bởi Zigbee stack khi một thuộc tính trên endpoint này được cập nhật.
     * @param message Con trỏ tới thông điệp giá trị thuộc tính.
     */
    void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) override;

    // --- Các biến trạng thái nội bộ ---
    bool _current_on_off_state;
    uint8_t _current_rotation_mode;
    uint8_t _current_pwm_speed;
    bool _current_gpio_states[6];

    // --- Con trỏ hàm Callback ---
    void (*_on_main_on_off_change_cb)(bool);
    void (*_on_rotation_mode_change_cb)(uint8_t);
    void (*_on_pwm_speed_change_cb)(uint8_t);
    void (*_on_gpio_change_cb)(uint8_t, bool);
};

#endif // CONFIG_ZB_ENABLED
#include "ZigbeeCar.h"
#if CONFIG_ZB_ENABLED

// Bao gồm các header cần thiết từ ESP-IDF Zigbee SDK
#include "esp_log.h"                  // Để ghi log (ESP_LOGI, ESP_LOGW, v.v.)
#include <cstring>                    // Để sử dụng memset, memcpy

// Thay vì esp_zigbee_core.h, chúng ta sẽ thêm các header cụ thể hơn
#include "ha/esp_zigbee_ha_standard.h"   // Đã có, cho các định nghĩa HA cluster/attribute
#include "zcl/esp_zigbee_zcl_cluster.h"   // Rất quan trọng cho esp_zb_zcl_attr_append_a_value và esp_zb_cluster_list_add_cluster
#include "zcl/esp_zigbee_zcl_common.h"    // Có thể cần cho một số định nghĩa ZCL chung khác

// Định nghĩa Tag cho Log
static const char *TAG = "ZigbeeCar";

// --- KHỞI TẠO CÁC BIẾN STATIC CHO THUỘC TÍNH BASIC CLUSTER ---
// Những biến này sẽ lưu trữ giá trị của các thuộc tính Basic Cluster.
// Chuỗi ZCL: byte đầu tiên là độ dài, theo sau là dữ liệu chuỗi.
static uint8_t g_basic_zcl_version = 0x01;
static uint8_t g_basic_power_source = 0x01; // 0x01 = Mains (hoặc 0x03 cho Battery)
static char g_basic_manufacturer_name[] = {0x09, 'M', 'y', 'C', 'o', 'm', 'p', 'a', 'n', 'y'}; // Length 9 + "MyCompany"
static char g_basic_model_id[] = {0x0E, 'Z', 'B', '_', 'C', 'a', 'r', '_', 'C', 'o', 'n', 't', 'r', 'o', 'l'}; // Length 14 + "ZB_Car_Control"
static char g_basic_date_code[] = {0x08, '2', '0', '2', '4', '0', '7', '1', '5'}; // Length 8 + "20240715"
static char g_basic_hw_version[] = {0x03, '1', '.', '0'}; // Length 3 + "1.0"
static char g_basic_sw_version[] = {0x05, '1', '.', '0', '.', '0'}; // Length 5 + "1.0.0"

// --- KHAI BÁO CÁC BIẾN STATIC CHO THUỘC TÍNH IDENTIFY CLUSTER ---
static uint16_t g_identify_time = 0; // Thời gian nhận diện ban đầu

// --- HÀM TẠO CỦA LỚP ZIGBEECAR ---
ZigbeeCar::ZigbeeCar(uint8_t endpoint) :
    ZigbeeEP(endpoint),
    // Khởi tạo các biến thành viên không tĩnh trong danh sách khởi tạo
    _current_on_off_state(false),
    _current_rotation_mode(0),
    _current_pwm_speed(0),
    _on_main_on_off_change_cb(nullptr),
    _on_rotation_mode_change_cb(nullptr),
    _on_pwm_speed_change_cb(nullptr),
    _on_gpio_change_cb(nullptr)
{
    // Khởi tạo mảng _current_gpio_states
    memset(_current_gpio_states, 0, sizeof(_current_gpio_states));

    // Thiết bị ID có thể là ON_OFF_LIGHT hoặc một ID gần nhất nếu không có ID cụ thể cho xe
    _device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID; 

    // Khởi tạo cluster list, sẽ thêm các cluster vào trong initClusters()
    _cluster_list = esp_zb_zcl_cluster_list_create(); 

    // Cấu hình endpoint cơ bản
    _ep_config = {.endpoint = _endpoint, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = _device_id, .app_device_version = 0};
    ESP_LOGV(TAG, "ZigbeeCar endpoint created with ID %d", _endpoint);
}

// --- HÀM KHỞI TẠO CÁC CLUSTER VÀ GÁN CHO ENDPOINT ---
void ZigbeeCar::initClusters() {
    // 1. Thêm Basic Cluster
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    if (!basic_cluster) {
        ESP_LOGE(TAG, "Failed to create Basic Cluster attribute list");
        return;
    }
    // Thêm các thuộc tính Basic vào cluster
    esp_err_t ret = esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &g_basic_zcl_version);
    if (ret == ESP_OK) ret = esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &g_basic_power_source);
    if (ret == ESP_OK) ret = esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, g_basic_manufacturer_name);
    if (ret == ESP_OK) ret = esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, g_basic_model_id);
    if (ret == ESP_OK) ret = esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, g_basic_date_code);
    if (ret == ESP_OK) ret = esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, g_basic_hw_version);
    if (ret == ESP_OK) ret = esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, g_basic_sw_version);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Basic Cluster attributes: %s", esp_err_to_name(ret));
        return;
    }
    esp_zb_cluster_list_add_basic_cluster(_cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // 2. Thêm Identify Cluster
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(NULL);
    if (!identify_cluster) {
        ESP_LOGE(TAG, "Failed to create Identify Cluster attribute list");
        return;
    }
    ret = esp_zb_identify_cluster_add_attr(identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &g_identify_time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Identify Cluster attributes: %s", esp_err_to_name(ret));
        return;
    }
    esp_zb_cluster_list_add_identify_cluster(_cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // 3. Thêm On/Off Cluster
    esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(NULL);
    if (!on_off_cluster) {
        ESP_LOGE(TAG, "Failed to create On/Off Cluster attribute list");
        return;
    }
    ret = esp_zb_on_off_cluster_add_attr(on_off_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &_current_on_off_state);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add On/Off Cluster attributes: %s", esp_err_to_name(ret));
        return;
    }
    esp_zb_cluster_list_add_on_off_cluster(_cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // 4. Thêm Motor Control Cluster (tùy chỉnh)
    esp_zb_attribute_list_t *motor_control_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_MOTOR_CONTROL);
    if (!motor_control_cluster) {
        ESP_LOGE(TAG, "Failed to create Motor Control Cluster attribute list");
        return;
    }
    // Thêm thuộc tính Rotation Mode
    ret = esp_zb_zcl_attr_append_a_value(
        motor_control_cluster,
        ZB_ZCL_ATTR_MOTOR_ROTATION_MODE_ID,
        ZB_ZCL_ATTR_TYPE_MOTOR_ROTATION_MODE, // Type từ header của bạn
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,    // Có thể đọc và ghi
        &_current_rotation_mode               // Con trỏ tới biến lưu trữ giá trị
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Motor Rotation Mode attribute: %s", esp_err_to_name(ret));
        return;
    }
    // Thêm thuộc tính PWM Speed
    ret = esp_zb_zcl_attr_append_a_value(
        motor_control_cluster,
        ZB_ZCL_ATTR_MOTOR_PWM_SPEED_ID,
        ZB_ZCL_ATTR_TYPE_MOTOR_PWM_SPEED,     // Type từ header của bạn
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
        &_current_pwm_speed                   // Con trỏ tới biến lưu trữ giá trị
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Motor PWM Speed attribute: %s", esp_err_to_name(ret));
        return;
    }
    esp_zb_cluster_list_add_cluster(_cluster_list, motor_control_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


    // 5. Thêm GPIO Control Cluster (tùy chỉnh)
    esp_zb_attribute_list_t *gpio_control_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_GPIO_CONTROL);
    if (!gpio_control_cluster) {
        ESP_LOGE(TAG, "Failed to create GPIO Control Cluster attribute list");
        return;
    }
    // Thêm các thuộc tính GPIO State (GPIO1 đến GPIO6)
    for (uint8_t i = 0; i < 6; ++i) {
        ret = esp_zb_zcl_attr_append_a_value(
            gpio_control_cluster,
            (ZB_ZCL_ATTR_GPIO1_STATE_ID + i), // Sử dụng ID từ header của bạn và tăng dần
            ZB_ZCL_ATTR_TYPE_GPIO_STATE,      // Type từ header của bạn (BOOL)
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &_current_gpio_states[i]          // Con trỏ tới biến lưu trữ trạng thái của từng GPIO
        );
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add GPIO%d State attribute: %s", i + 1, esp_err_to_name(ret));
            return;
        }
    }
    esp_zb_cluster_list_add_cluster(_cluster_list, gpio_control_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


    // Đăng ký Endpoint vào Zigbee Stack
    // Với mô hình của bạn, việc đăng ký endpoint có thể được thực hiện trong lớp Zigbee toàn cục
    // sau khi gọi Zigbee.addEndpoint(&zbCar);
    // Hàm này (initClusters) chỉ chịu trách nhiệm tạo và thêm các cluster vào _cluster_list.
    ESP_LOGI(TAG, "ZigbeeCar endpoint %d clusters initialized with HA Device ID 0x%04X", _endpoint, _device_id);
}

// --- HÀM XỬ LÝ SỰ KIỆN THAY ĐỔI THUỘC TÍNH ZCL ---
void ZigbeeCar::zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) {
    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Received attribute set for Cluster ID 0x%04X, Attribute ID 0x%04X",
                 message->info.cluster, message->attribute.id);

        // Xử lý cluster On/Off
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
                message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                _current_on_off_state = *(bool *)message->attribute.data.value;
                ESP_LOGI(TAG, "Main On/Off state updated to: %s", _current_on_off_state ? "ON" : "OFF");
                if (_on_main_on_off_change_cb) {
                    _on_main_on_off_change_cb(_current_on_off_state);
                }
            } else {
                ESP_LOGW(TAG, "Unhandled On/Off attribute ID 0x%04X or invalid type %d", message->attribute.id, message->attribute.data.type);
            }
        }
        // Xử lý Cluster Motor Control tùy chỉnh
        else if (message->info.cluster == ZB_ZCL_CLUSTER_ID_MOTOR_CONTROL) {
            if (message->attribute.id == ZB_ZCL_ATTR_MOTOR_ROTATION_MODE_ID &&
                message->attribute.data.type == ZB_ZCL_ATTR_TYPE_MOTOR_ROTATION_MODE) {
                _current_rotation_mode = *(uint8_t *)message->attribute.data.value;
                ESP_LOGI(TAG, "Motor Rotation Mode updated to: %u", _current_rotation_mode);
                if (_on_rotation_mode_change_cb) {
                    _on_rotation_mode_change_cb(_current_rotation_mode);
                }
            } else if (message->attribute.id == ZB_ZCL_ATTR_MOTOR_PWM_SPEED_ID &&
                       message->attribute.data.type == ZB_ZCL_ATTR_TYPE_MOTOR_PWM_SPEED) {
                _current_pwm_speed = *(uint8_t *)message->attribute.data.value;
                ESP_LOGI(TAG, "Motor PWM Speed updated to: %u", _current_pwm_speed);
                if (_on_pwm_speed_change_cb) {
                    _on_pwm_speed_change_cb(_current_pwm_speed);
                }
            } else {
                ESP_LOGW(TAG, "Unhandled Motor Control attribute ID 0x%04X or invalid type %d", message->attribute.id, message->attribute.data.type);
            }
        }
        // Xử lý Cluster GPIO Control tùy chỉnh
        else if (message->info.cluster == ZB_ZCL_CLUSTER_ID_GPIO_CONTROL) {
            // Duyệt qua các thuộc tính GPIO (ZB_ZCL_ATTR_GPIO1_STATE_ID đến ZB_ZCL_ATTR_GPIO6_STATE_ID)
            if (message->attribute.id >= ZB_ZCL_ATTR_GPIO1_STATE_ID &&
                message->attribute.id <= ZB_ZCL_ATTR_GPIO6_STATE_ID && // So sánh với ID cuối cùng
                message->attribute.data.type == ZB_ZCL_ATTR_TYPE_GPIO_STATE) {
                uint8_t gpio_index = message->attribute.id - ZB_ZCL_ATTR_GPIO1_STATE_ID; // Tính index từ ID (0-5)
                _current_gpio_states[gpio_index] = *(bool *)message->attribute.data.value;
                ESP_LOGI(TAG, "GPIO%d state updated to: %s", gpio_index + 1, _current_gpio_states[gpio_index] ? "HIGH" : "LOW");
                if (_on_gpio_change_cb) {
                    _on_gpio_change_cb(gpio_index + 1, _current_gpio_states[gpio_index]); // Truyền index từ 1-6
                }
            } else {
                ESP_LOGW(TAG, "Unhandled GPIO Control attribute ID 0x%04X or invalid type %d", message->attribute.id, message->attribute.data.type);
            }
        }
        else {
            ESP_LOGW(TAG, "Received message ignored. Cluster ID 0x%04X not supported for this device", message->info.cluster);
        }
    } else {
        ESP_LOGW(TAG, "Attribute set failed with status: 0x%02X for cluster 0x%04X, attr 0x%04X",
                 message->info.status, message->info.cluster, message->attribute.id);
    }
}

#endif // CONFIG_ZB_ENABLED
#include "ZigbeeFanRemote.h"
#if CONFIG_ZB_ENABLED

ZigbeeFanRemote::ZigbeeFanRemote(uint8_t endpoint) : ZigbeeEP(endpoint) {
    // Không có DEVICE_ID cụ thể cho Fan trong HA Profile, sử dụng ID chung hoặc tương tự.
    // ESP_ZB_HA_THERMOSTAT_DEVICE_ID thường được dùng cho các thiết bị HVAC.
    _device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID;

    // Tạo danh sách cluster cho thiết bị quạt (Fan Control Server)
    _cluster_list = esp_zb_zcl_cluster_list_create();
    // Thêm Basic Cluster (Server Role) - thông tin cơ bản về thiết bị
    esp_zb_cluster_list_add_basic_cluster(_cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Thêm Identify Cluster (Server Role) - để thiết bị có thể được nhận diện (nhấp nháy LED)
    esp_zb_cluster_list_add_identify_cluster(_cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Thêm Fan Control Cluster (Server Role) - đây là cluster chính để nhận lệnh điều khiển quạt
    esp_zb_cluster_list_add_fan_control_cluster(_cluster_list, esp_zb_fan_control_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Cấu hình Endpoint
    _ep_config = {
        .endpoint = _endpoint,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        // Dùng Heating/Cooling Unit ID vì nó liên quan đến HVAC và không có Fan ID cụ thể.
        .app_device_id = ESP_ZB_HA_HEATING_COOLING_UNIT_DEVICE_ID,
        .app_device_version = 0
    };

    // Khởi tạo chế độ quạt ban đầu
    _current_fan_mode = FAN_MODE_OFF;
    _current_fan_mode_sequence = FAN_MODE_SEQUENCE_LOW_MED_HIGH; // Mặc định một chuỗi
    _on_fan_mode_change = nullptr; // Khởi tạo callback là null
}

// Thiết lập chuỗi chế độ quạt cho Fan Control Server
bool ZigbeeFanRemote::setFanModeSequence(ZigbeeFanModeSequence sequence) {
    esp_zb_attribute_list_t *fan_control_cluster =
        esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    if (!fan_control_cluster) {
        log_e("Không tìm thấy Fan Control Cluster Server!");
        return false;
    }

    esp_err_t ret = esp_zb_cluster_update_attr(fan_control_cluster, ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID, (void *)&sequence);
    if (ret != ESP_OK) {
        log_e("Failed to set fan mode sequence: 0x%x: %s", ret, esp_err_to_name(ret));
        return false;
    }
    _current_fan_mode_sequence = sequence; // Cập nhật biến thành viên

    // Đặt lại chế độ quạt về OFF khi thay đổi chuỗi chế độ
    _current_fan_mode = FAN_MODE_OFF;
    ret = esp_zb_cluster_update_attr(fan_control_cluster, ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID, (void *)&_current_fan_mode);
    if (ret != ESP_OK) {
        log_e("Failed to set initial fan mode to OFF: 0x%x: %s", ret, esp_err_to_name(ret));
        return false;
    }
    log_i("Fan Mode Sequence set to: %d", (uint8_t)sequence);
    return true;
}

// Xử lý khi một Client ghi (set) thuộc tính trên Fan Control Server
void ZigbeeFanRemote::zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) {
    // Kiểm tra xem tin nhắn có phải cho Fan Control Cluster và thuộc tính Fan Mode không
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID &&
            message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {

            // Lấy giá trị chế độ quạt từ tin nhắn và ép kiểu về ZigbeeFanMode
            _current_fan_mode = *(ZigbeeFanMode *)message->attribute.data.value;
            log_i("Đã nhận lệnh Fan Mode từ Zigbee: %d", (uint8_t)_current_fan_mode);
            fanModeChanged(); // Kích hoạt callback để xử lý thay đổi chế độ quạt

        } else {
            log_w("Nhận tin nhắn ZCL bị bỏ qua. Thuộc tính ID: %d không được hỗ trợ cho Fan Control.", message->attribute.id);
        }
    } else {
        log_w("Nhận tin nhắn ZCL bị bỏ qua. Cluster ID: %d không được hỗ trợ bởi Fan Control.", message->info.cluster);
    }
}

// Kích hoạt hàm callback khi chế độ quạt thay đổi
void ZigbeeFanRemote::fanModeChanged() {
    if (_on_fan_mode_change) {
        _on_fan_mode_change(_current_fan_mode); // Gọi hàm callback với chế độ quạt mới
    } else {
        log_w("Không có hàm callback nào được đặt cho sự thay đổi chế độ quạt.");
    }
}

#endif // CONFIG_ZB_ENABLED
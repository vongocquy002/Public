#include "ZigbeeCarDevice.h"
#include "esp_log.h"
#include "Zigbee.h"

#if CONFIG_ZB_ENABLED

static const char *TAG = "ZIGBEE_CAR_DEVICE";

// Constructor
ZigbeeCarDevice::ZigbeeCarDevice(uint8_t endpoint) : ZigbeeEP(endpoint) {
    _device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID; // Sử dụng Device ID tương thích

    // Khởi tạo trạng thái ban đầu
    _state.on_off_state = false;
    _state.level = 0;
    _state.fan_mode = FAN_MODE_OFF;
    _state.endpoint = endpoint;
    esp_zb_get_long_address(_state.ieee_addr);

    // Tạo danh sách cluster
    _cluster_list = esp_zb_zcl_cluster_list_create();
    if (!_cluster_list) {
        ESP_LOGE(TAG, "Khong the tao cluster list!");
        return;
    }

    // Thêm Server Clusters
    esp_zb_cluster_list_add_basic_cluster(_cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(_cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(_cluster_list, esp_zb_on_off_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_level_cluster(_cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_fan_control_cluster(_cluster_list, esp_zb_fan_control_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Thêm Client Cluster cho Identify
    esp_zb_cluster_list_add_identify_cluster(_cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    _ep_config = {
        .endpoint = _endpoint,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = _device_id,
        .app_device_version = 0
    };

    ESP_LOGI(TAG, "ZigbeeCarDevice khoi tao thanh cong tren Endpoint %d", endpoint);
}

// Xử lý cập nhật thuộc tính ZCL
void ZigbeeCarDevice::zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) {
    if (!message) {
        ESP_LOGE(TAG, "Thong diep ZCL la null!");
        return;
    }

    uint16_t cluster_id = message->info.cluster;
    uint16_t attr_id = message->attribute.id;
    void *value = message->attribute.data.value;

    ESP_LOGI(TAG, "Nhan cap nhat thuoc tinh: Cluster 0x%04x, Attribute 0x%04x", cluster_id, attr_id);

    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
            _state.on_off_state = *(bool *)value;
            ESP_LOGI(TAG, "Nhan lenh On/Off: %s", _state.on_off_state ? "ON" : "OFF");
            updateStateAndReport(cluster_id, attr_id, value);
        } else {
            ESP_LOGW(TAG, "Thuoc tinh khong duoc ho tro: Cluster 0x%04x, Attr 0x%04x", cluster_id, attr_id);
        }
    } else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
        if (attr_id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
            _state.level = *(uint8_t *)value;
            ESP_LOGI(TAG, "Nhan lenh Level Control: Level %d", _state.level);
            updateStateAndReport(cluster_id, attr_id, value);
            // Cập nhật trạng thái On/Off nếu Level > 0
            if (_state.level > 0) {
                _state.on_off_state = true;
                updateStateAndReport(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &_state.on_off_state);
            }
        } else {
            ESP_LOGW(TAG, "Thuoc tinh khong duoc ho tro: Cluster 0x%04x, Attr 0x%04x", cluster_id, attr_id);
        }
    } else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL) {
        if (attr_id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
            _state.fan_mode = *(ZigbeeFanMode *)value;
            ESP_LOGI(TAG, "Nhan lenh Fan Mode: %d", _state.fan_mode);
            updateStateAndReport(cluster_id, attr_id, value);
        } else {
            ESP_LOGW(TAG, "Thuoc tinh khong duoc ho tro: Cluster 0x%04x, Attr 0x%04x", cluster_id, attr_id);
        }
    } else {
        ESP_LOGW(TAG, "Cluster khong duoc ho tro: 0x%04x", cluster_id);
    }
}

// Cập nhật trạng thái và gửi báo cáo
void ZigbeeCarDevice::updateStateAndReport(uint16_t cluster_id, uint16_t attr_id, void *value) {
    // Gửi báo cáo
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u = { .addr_short = 0x0000 }, // Gửi đến Coordinator
            .dst_endpoint = 1,                      // Endpoint của Coordinator
            .src_endpoint = _endpoint
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster_id,
        .attributeID = attr_id
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t status = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    esp_zb_lock_release();

    if (status == ESP_OK) {
        ESP_LOGI(TAG, "Gui bao cao thanh cong: Cluster 0x%04x, Attr 0x%04x", cluster_id, attr_id);
    } else {
        ESP_LOGE(TAG, "Loi khi gui bao cao: Cluster 0x%04x, Attr 0x%04x, Status %d", cluster_id, attr_id, status);
    }
}

// Gửi báo cáo trạng thái đầy đủ
bool ZigbeeCarDevice::sendStateReport() {
    bool success = true;
    esp_zb_lock_acquire(portMAX_DELAY);

    // Báo cáo On/Off
    esp_zb_zcl_report_attr_cmd_t on_off_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u = { .addr_short = 0x0000 },
            .dst_endpoint = 1,
            .src_endpoint = _endpoint
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        .attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID
    };
    if (esp_zb_zcl_report_attr_cmd_req(&on_off_cmd) != ESP_OK) success = false;

    // Báo cáo Level
    esp_zb_zcl_report_attr_cmd_t level_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u = { .addr_short = 0x0000 },
            .dst_endpoint = 1,
            .src_endpoint = _endpoint
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
        .attributeID = ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID
    };
    if (esp_zb_zcl_report_attr_cmd_req(&level_cmd) != ESP_OK) success = false;

    // Báo cáo Fan Mode
    esp_zb_zcl_report_attr_cmd_t fan_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u = { .addr_short = 0x0000 },
            .dst_endpoint = 1,
            .src_endpoint = _endpoint
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
        .attributeID = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID
    };
    if (esp_zb_zcl_report_attr_cmd_req(&fan_cmd) != ESP_OK) success = false;

    esp_zb_lock_release();
    return success;
}

#endif // CONFIG_ZB_ENABLED
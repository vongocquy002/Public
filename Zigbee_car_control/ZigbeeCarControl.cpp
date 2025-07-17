#include "ZigbeeCarControl.h"
#include "Zigbee.h"
#if CONFIG_ZB_ENABLED

#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_command.h"
// Khởi tạo con trỏ tĩnh của lớp
ZigbeeCarControl *ZigbeeCarControl::_instance = nullptr;

/**
 * @brief Hàm tạo cho ZigbeeCarControl.
 * Cấu hình Endpoint và các Cluster mà ZigbeeCarControl sẽ hỗ trợ (với vai trò Client hoặc Server).
 * @param endpoint Số endpoint Zigbee cho thiết bị này.
 */
ZigbeeCarControl::ZigbeeCarControl(uint8_t endpoint) : ZigbeeEP(endpoint) {
    _device_id = ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID;
    _instance = this;

    // _cluster_list = esp_zb_zcl_cluster_list_create();
    // if (!_cluster_list) {
    //     log_e("Không thể tạo cluster list!");
    //     return;
    // }
    // // 1. Thêm Basic Cluster (SERVER_ROLE)
    // esp_zb_cluster_list_add_basic_cluster(_cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // // 2. Thêm Identify Cluster (SERVER_ROLE và CLIENT_ROLE) Để thiết bị này có thể được nhận dạng
    // esp_zb_cluster_list_add_identify_cluster(_cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // // 3. On/Off Cluster: Nhận báo cáo trạng thái BẬT/TẮT từ motor hoặc GPIO của CarDevice.
    // // esp_zb_on_off_cluster_cfg_t on_off_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    // // on_off_cfg.on_off = false; // Khởi tạo trạng thái ban đầu là TẮT (OFF)
    // // esp_zb_cluster_list_add_on_off_cluster(_cluster_list, esp_zb_on_off_cluster_create(&on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // esp_zb_on_off_cluster_cfg_t on_off_cfg = {
    //     .on_off = false // Trạng thái ban đầu là OFF (TẮT)
    // };
    // esp_zb_cluster_list_add_on_off_cluster(
    //     _cluster_list,
    //     esp_zb_on_off_cluster_create(&on_off_cfg),
    //     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    // );

    // // 4. Fan Control Cluster: Đại diện cho tốc độ (speed) của motor
    // // esp_zb_color_dimmable_switch_cfg_t level_control_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_SWITCH_CONFIG();
    // // esp_zb_cluster_list_add_level_control_cluster(_cluster_list, esp_zb_color_dimmable_switch_clusters_create(&level_control_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE)) {
    //     esp_zb_fan_control_cluster_cfg_t fan_control_cfg = {
    //         .fan_mode = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_OFF,
    //         .fan_mode_sequence = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH
    //     };
    //     esp_zb_cluster_list_add_fan_control_cluster(
    //         _cluster_list,
    //         esp_zb_fan_control_cluster_create(&fan_control_cfg),
    //         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    //     );
    // }

    // // Xác nhận Client Clusters (On/Off, Level Control, Identify) đã có trong _cluster_list
    // // (chúng được thêm tự động bởi esp_zb_color_dimmable_switch_clusters_create)
    // if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE)) {
    //     ESP_LOGW(TAG, "Identify Client Cluster không tìm thấy sau khi tạo! Có thể có lỗi cấu hình.");
    // }
    // if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE)) {
    //     ESP_LOGW(TAG, "On/Off Client Cluster không tìm thấy sau khi tạo! Có thể có lỗi cấu hình.");
    // }
    // if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE)) {
    //     ESP_LOGW(TAG, "Level Control Client Cluster không tìm thấy sau khi tạo! Có thể có lỗi cấu hình.");
    // }
    // // void app_report_handler(esp_zb_zcl_report_attr_message_t *message)
    // // {
    // //     if (message->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL &&
    // //         message->attribute_id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID) {

    // //         uint8_t speed = *(uint8_t *)message->attribute_data;
    // //         ESP_LOGI(TAG, "Tốc độ nhận được từ End Device (fan_mode): %d", speed);
    // //     }
    // // }
    // // // }//code callback cấu hình
    // // 5. Fan Control Cluster: Nhận báo cáo chế độ hoạt động từ GPIO của CarDevice.
    // esp_zb_cluster_list_add_fan_control_cluster(
    //     _cluster_list,
    //     esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL), // Tạo danh sách thuộc tính trống cho client
    //     ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE
    // );
    // // 6. Identify Cluster: Để ZigbeeCarControl có thể gửi lệnh yêu cầu thiết bị khác nhận dạng.
    // //esp_zb_cluster_list_add_identify_cluster(_cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    // esp_zb_cluster_list_add_identify_cluster(
    //     _cluster_list,
    //     esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY),
    //     ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE
    // );

    // // 7. On/Off Cluster: Gửi lệnh BẬT/TẮT motor hoặc GPIO. CLIENT_ROLE
    // //esp_zb_cluster_list_add_on_off_cluster(_cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    // esp_zb_cluster_list_add_on_off_cluster(
    //     _cluster_list,
    //     esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF),
    //     ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE
    // );
    // // 8. Level Control Cluster: Gửi lệnh điều khiển tốc độ motor.
    // //esp_zb_cluster_list_add_level_control_cluster(_cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    // esp_zb_color_dimmable_switch_cfg_t pwm_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_SWITCH_CONFIG();
    // _cluster_list = esp_zb_color_dimmable_switch_clusters_create(&pwm_cfg);
    // // 9. Fan Control Cluster: Gửi lệnh điều khiển chế độ hoạt động của GPIO.
    // //esp_zb_cluster_list_add_fan_control_cluster(_cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    // esp_zb_cluster_list_add_fan_control_cluster(
    //     _cluster_list,
    //     esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL),
    //     ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE
    // );

    esp_zb_color_dimmable_switch_cfg_t switch_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_SWITCH_CONFIG();
    _cluster_list = esp_zb_color_dimmable_switch_clusters_create(&switch_cfg);

    if (!_cluster_list) {
        ESP_LOGE(TAG, "Không thể tạo cluster list từ Color Dimmable Switch cấu hình!");
        return;
    }
    
    // BÂY GIỜ, HÃY THÊM (HOẶC ĐẢM BẢO ĐÃ CÓ) CÁC SERVER CLUSTER MÀ BẠN MUỐN THIẾT BỊ NÀY CUNG CẤP.
    // Các hàm esp_zb_cluster_list_add_... sẽ thêm cluster nếu nó chưa tồn tại ở vai trò cụ thể đó.

    // 1. Basic Cluster (SERVER_ROLE)
    if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE)) {
        esp_zb_cluster_list_add_basic_cluster(_cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }

    // 2. Identify Cluster (SERVER_ROLE)
    if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE)) {
        esp_zb_cluster_list_add_identify_cluster(_cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }

    // 3. On/Off Cluster (SERVER_ROLE) - Cần để nhận báo cáo trạng thái BẬT/TẮT
    if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE)) {
        esp_zb_on_off_cluster_cfg_t on_off_cfg = { .on_off = false }; // Trạng thái ban đầu là OFF
        esp_zb_cluster_list_add_on_off_cluster(
            _cluster_list,
            esp_zb_on_off_cluster_create(&on_off_cfg),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        );
    }

    // 4. Fan Control Cluster (SERVER_ROLE) - Cần để nhận báo cáo chế độ hoạt động
    if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE)) {
        esp_zb_fan_control_cluster_cfg_t fan_control_cfg = {
            .fan_mode = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_OFF,
            .fan_mode_sequence = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH
        };
        esp_zb_cluster_list_add_fan_control_cluster(
            _cluster_list,
            esp_zb_fan_control_cluster_create(&fan_control_cfg),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        );
    }

    // BƯỚC CUỐI CÙNG: Thêm các CLIENT CLUSTER KHÁC mà esp_zb_color_dimmable_switch_clusters_create KHÔNG TẠO.
    // Ví dụ: Fan Control Client.
    // Các Client Cluster như On/Off, Level Control, Identify đã được tạo bởi esp_zb_color_dimmable_switch_clusters_create ở trên.
    if (!esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE)) {
        esp_zb_cluster_list_add_fan_control_cluster(
            _cluster_list,
            esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL),
            ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE
        );
    }
    
    _ep_config = {
        .endpoint = _endpoint,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, // Hồ sơ Home Automation
        .app_device_id = _device_id,                // Remote Control Device ID
        .app_device_version = 0                     // Phiên bản thiết bị
    };

    ESP_LOGI(TAG, "ZigbeeCarControl đã được khởi tạo cho Endpoint %d (Device ID: 0x%04x) với khả năng nhận báo cáo trạng thái.", endpoint, _device_id);
}


/**
 * @brief Hàm wrapper tĩnh cho bindCb.
 * Được sử dụng bởi stack Zigbee để gọi hàm thành viên bindCb.
 * @param zdo_status Trạng thái của hoạt động ZDO.
 * @param user_ctx Con trỏ ngữ cảnh người dùng.
 */
void ZigbeeCarControl::bindCbWrapper(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
    if (ZigbeeCarControl::_instance) {
        ZigbeeCarControl::_instance->bindCb(zdo_status, user_ctx);
    } else {
        log_e("ZigbeeCarControl instance không được đặt cho bindCbWrapper! user_ctx có thể bị rò rỉ bộ nhớ.");
        if (user_ctx) {
            free(user_ctx); // Đảm bảo giải phóng bộ nhớ ngay cả khi instance không tồn tại
        }
    }
}

/**
 * @brief Hàm callback nội bộ (non-static) để xử lý kết quả yêu cầu liên kết ZDO.
 * Được gọi sau khi một yêu cầu bind được gửi đi và nhận được phản hồi.
 * @param zdo_status Trạng thái của hoạt động ZDO.
 * @param user_ctx Con trỏ tới ZigbeeCarDeviceState của thiết bị vừa được liên kết.
 */
void ZigbeeCarControl::bindCb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
    // Kiểm tra user_ctx hợp lệ
    if (!user_ctx) {
        log_e("User context (device info) là null!");
        return;
    }
    // Ép kiểu ngữ cảnh người dùng về loại ZigbeeCarDeviceState
    ZigbeeCarDeviceState *device_info_to_bind = static_cast<ZigbeeCarDeviceState *>(user_ctx);

    if (zdo_status != ESP_ZB_ZDP_STATUS_SUCCESS) {
        log_e("Liên kết với thiết bị thất bại: 0x%04x (EP %d), mã lỗi: %d",
              device_info_to_bind->short_addr, device_info_to_bind->endpoint, zdo_status);
        free(device_info_to_bind);  // Giải phóng bộ nhớ nếu thất bại
        return;
    }
    log_i("Đã liên kết với Fan Control Server: 0x%04x, EP %d",
          device_info_to_bind->short_addr, device_info_to_bind->endpoint);

    // Tránh trùng lặp
    bool found = std::any_of(_bound_fan_servers.begin(), _bound_fan_servers.end(),
                             [&](const ZigbeeCarDeviceState &dev) {
                                 return dev.short_addr == device_info_to_bind->short_addr &&
                                        dev.endpoint == device_info_to_bind->endpoint;
                             });

    if (!found) {
        _bound_fan_servers.push_back(*device_info_to_bind);
        log_i("Đã thêm thiết bị 0x%04x (EP %d) vào danh sách.", device_info_to_bind->short_addr, device_info_to_bind->endpoint);

        // Cấu hình báo cáo cho các cluster cần thiết
        configureOnOffReporting(device_info_to_bind->short_addr, device_info_to_bind->endpoint);
        configureLevelControlReporting(device_info_to_bind->short_addr, device_info_to_bind->endpoint);
        configureFanModeReporting(device_info_to_bind->short_addr, device_info_to_bind->endpoint);
    } else {
        log_w("Thiết bị 0x%04x (EP %d) đã có trong danh sách, bỏ qua.",
              device_info_to_bind->short_addr, device_info_to_bind->endpoint);
    }

    // Giải phóng bộ nhớ động (nếu dùng malloc)
    free(device_info_to_bind);
}


/**
 * @brief Hàm callback nội bộ (non-static) để xử lý kết quả yêu cầu tìm kiếm ZDO (Match Descriptor).
 * Được gọi khi một thiết bị phù hợp với mô tả tìm kiếm được tìm thấy.
 * @param zdo_status Trạng thái của hoạt động ZDO.
 * @param addr Địa chỉ ngắn của thiết bị tìm thấy.
 * @param endpoint Số endpoint của thiết bị tìm thấy.
 * @param user_ctx Con trỏ ngữ cảnh người dùng (trong trường hợp này là con trỏ tới instance của ZigbeeCarControl).
 */
void ZigbeeCarControl::findCb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx) {
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        log_i("Tìm thấy ZigbeeCarDevice Fan Control Server: địa chỉ 0x%04x, endpoint %d", addr, endpoint);

        // Kiểm tra xem thiết bị đã được thêm vào danh sách hoặc đang trong quá trình xử lý liên kết chưa.
        bool already_processed = false;
        for (const auto& device : _bound_fan_servers) {
            if (device.short_addr == addr && device.endpoint == endpoint) {
                already_processed = true;
                break;
            }
        }

        if (already_processed) {
            log_d("Thiết bị ZigbeeCarDevice 0x%04x (EP %d) đã được biết. Bỏ qua bước liên kết.", addr, endpoint);
            return;
        }

        // Cấp phát bộ nhớ cho thông tin thiết bị để truyền vào hàm bindCb.
        // Đây là thông tin mà bindCb sẽ sử dụng để thêm thiết bị vào _bound_fan_servers.
        ZigbeeCarDeviceState *device_info_to_bind = (ZigbeeCarDeviceState *)malloc(sizeof(ZigbeeCarDeviceState));
        if (!device_info_to_bind) {
            log_e("Không đủ bộ nhớ để cấp phát device_info_to_bind trong findCb");
            return;
        }
        memset(device_info_to_bind, 0, sizeof(ZigbeeCarDeviceState)); // Khởi tạo bằng 0

        device_info_to_bind->endpoint = endpoint;
        device_info_to_bind->short_addr = addr;
        // Lấy địa chỉ IEEE (địa chỉ dài) của thiết bị từ địa chỉ ngắn
        esp_zb_ieee_address_by_short(device_info_to_bind->short_addr, device_info_to_bind->ieee_addr);
        log_d("Đã cấp phát params cho thiết bị: địa chỉ 0x%04x, endpoint %d", device_info_to_bind->short_addr, device_info_to_bind->endpoint);

        // Chuẩn bị yêu cầu liên kết ZDO (ZDO Bind Request).
        // Yêu cầu thiết bị Fan Control Server từ xa liên kết Client của nó tới
        // Fan Control Server của chúng ta (Coordinator).
        esp_zb_zdo_bind_req_param_t bind_req = {0};
        esp_zb_ieee_addr_t ieee_addr_our_device; // Địa chỉ IEEE của thiết bị Coordinator này
        esp_zb_get_long_address(ieee_addr_our_device);

        bind_req.req_dst_addr = addr; // Địa chỉ ngắn của thiết bị từ xa
        memcpy(bind_req.src_address, device_info_to_bind->ieee_addr, ZB_IEEE_ADDR_SIZE); // Địa chỉ IEEE của thiết bị từ xa
        bind_req.src_endp = endpoint; // Endpoint của Fan Control Server trên thiết bị từ xa
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL; // Cluster mà chúng ta muốn liên kết
        bind_req.dst_addr_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT; // Chế độ địa chỉ đích (dài + endpoint)
        memcpy(bind_req.dst_address_u.addr_long, ieee_addr_our_device, ZB_IEEE_ADDR_SIZE); // Địa chỉ IEEE của Coordinator
        bind_req.dst_endp = _endpoint; // Endpoint của ZigbeeCarControl (nơi nhận báo cáo)

        log_i("Đang yêu cầu ZigbeeCarDevice (0x%04x, EP %d) liên kết Fan Control Client của nó tới chúng tôi (EP %d)...",
              addr, endpoint, _endpoint);
        // Gửi yêu cầu bind và truyền device_info_to_bind làm user_ctx cho hàm bindCbWrapper
        esp_zb_zdo_device_bind_req(&bind_req, ZigbeeCarControl::bindCbWrapper, device_info_to_bind);

    } else {
        log_d("Không tìm thấy ZigbeeCarDevice Fan Control Server hoặc có lỗi trong quá trình tìm kiếm, trạng thái: %d", zdo_status);
        // Không giải phóng user_ctx ở đây vì nó không được cấp phát bởi findCbWrapper
    }
}

/**
 * @brief Hàm wrapper tĩnh cho findCb.
 * Được sử dụng bởi stack Zigbee để gọi hàm thành viên findCb.
 * @param zdo_status Trạng thái của hoạt động ZDO.
 * @param addr Địa chỉ ngắn của thiết bị tìm thấy.
 * @param endpoint Số endpoint của thiết bị tìm thấy.
 * @param user_ctx Con trỏ ngữ cảnh người dùng.
 */
void ZigbeeCarControl::findCbWrapper(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx) {
    if (ZigbeeCarControl::_instance) {
        ZigbeeCarControl::_instance->findCb(zdo_status, addr, endpoint, user_ctx);
    } else {
        log_e("ZigbeeCarControl instance không được đặt cho findCbWrapper!");
    }
}

/**
 * @brief Bắt đầu quá trình tìm kiếm các Fan Control Server trong mạng.
 * Sau khi tìm thấy, quá trình liên kết sẽ được bắt đầu tự động thông qua các callback.
 */
void ZigbeeCarControl::findCarControlServer() {
    esp_zb_zdo_match_desc_req_param_t param = {0};
    // Tìm kiếm các thiết bị có Fan Control Cluster (làm Server)
    uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL};

    param.profile_id = ESP_ZB_AF_HA_PROFILE_ID; // Hồ sơ Home Automation
    param.num_in_clusters = 1;                  // Chúng ta đang tìm 1 Cluster (Fan Control) mà thiết bị khác là Server
    param.cluster_list = cluster_list;
    param.num_out_clusters = 0;                 // Không quan tâm Out-Clusters của thiết bị đích trong tìm kiếm này

    log_i("Đang tìm kiếm ZigbeeCarDevice có Fan Control Servers trên mạng...");
    // Gửi yêu cầu Match Descriptor và truyền 'this' làm user_ctx để findCbWrapper có thể gọi lại findCb của instance này.
    esp_zb_zdo_match_cluster(&param, ZigbeeCarControl::findCbWrapper, this);
}

/**
 * @brief Đặt chế độ hoạt động cho motor hoặc các GPIO thông qua Fan Control Cluster.
 * Hàm này sẽ gửi một lệnh Write Attribute đến Fan Control Server trên ZigbeeCarDevice.
 * @param mode Chế độ mong muốn (FanMode).
 * @param target_addr Địa chỉ ngắn của thiết bị mục tiêu (0xFFFF cho tất cả đã liên kết).
 * @param target_ep Endpoint của thiết bị mục tiêu (chỉ dùng khi target_addr không phải 0xFFFF).
 * @return True nếu lệnh được gửi thành công ít nhất một lần, false nếu không gửi được lệnh nào.
 */
bool ZigbeeCarControl::setCarMode(ZigbeeFanMode mode, uint16_t target_addr, uint8_t target_ep) {
    if (_bound_fan_servers.empty() && target_addr == 0xFFFF) {
        log_e("Chưa có ZigbeeCarDevice nào được liên kết. Không thể gửi lệnh Fan Mode.");
        return false;
    }

    uint8_t fan_mode_val = (uint8_t)mode;
    bool command_sent_at_least_one = false;
    // Duyệt qua tất cả các thiết bị đã liên kết để gửi lệnh unicast
    for (const auto& server_params : _bound_fan_servers) {
        if (target_addr != 0xFFFF && server_params.short_addr != target_addr) {
            continue;
        }
        if (target_ep != 0xFF && server_params.endpoint != target_ep) {
            continue;
        }

        log_i("Đang gửi lệnh Fan Mode: %d tới 0x%04x/EP%d",
              mode, server_params.short_addr, server_params.endpoint);

        esp_zb_lock_acquire(portMAX_DELAY); // Khóa Zigbee stack trước khi gửi lệnh

        // 1. Chuẩn bị dữ liệu thuộc tính
        esp_zb_zcl_attribute_data_t attr_data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, // Kiểu dữ liệu của thuộc tính Fan Mode
            .size = sizeof(uint8_t),                // Kích thước của giá trị
            .value = &fan_mode_val,                 // Con trỏ tới giá trị fan_mode_val
        };

        // 2. Chuẩn bị thuộc tính để ghi
        esp_zb_zcl_attribute_t attr_field = {
            .id = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID, // ID của thuộc tính Fan Mode
            .data = attr_data,                             // Dữ liệu thuộc tính
        };

        // 3. Chuẩn bị cấu trúc lệnh ghi thuộc tính
        esp_zb_zcl_write_attr_cmd_t write_cmd = {
            .zcl_basic_cmd = {
                
                .dst_addr_u = { .addr_short = server_params.short_addr }, // Địa chỉ ngắn đích
                .dst_endpoint = server_params.endpoint,                   // Endpoint đích
                .src_endpoint = _endpoint,                                // Endpoint nguồn (của Coordinator)
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .clusterID = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
            .manuf_specific = 0,
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .dis_defalut_resp = 0,
            .manuf_code = 0,
            .attr_number = 1,
            .attr_field = &attr_field,
        };

        // 4. Gọi hàm gửi lệnh ZCL
        esp_err_t api_status = esp_zb_zcl_write_attr_cmd_req(&write_cmd);

        esp_zb_lock_release(); // Giải phóng khóa

        if (api_status != ESP_OK) {
            log_e("Lỗi API khi gửi lệnh Fan Mode cho 0x%04x/EP%d, trạng thái API: %d",
                  server_params.short_addr, server_params.endpoint, api_status);
        } else {
            log_i("Yêu cầu gửi lệnh Fan Mode: %d đến 0x%04x/EP %d đã được gửi thành công (chờ phản hồi ZCL).",
                  mode, server_params.short_addr, server_params.endpoint);
            command_sent_at_least_one = true;
        }

        if (target_addr != 0xFFFF && server_params.short_addr == target_addr &&
            target_ep != 0xFF && server_params.endpoint == target_ep) {
            break;
        }
    }
    return command_sent_at_least_one;
}
/**
 * @brief Gửi lệnh On hoặc Off tới một thiết bị/motor/GPIO cụ thể hoặc tất cả các thiết bị đã liên kết
 * thông qua On/Off Cluster.
 * @param on True để bật (On), false để tắt (Off).
 * @param target_addr Địa chỉ ngắn của thiết bị mục tiêu (0xFFFF để gửi broadcast cho tất cả đã liên kết).
 * @param target_ep Endpoint của thiết bị mục tiêu chứa On/Off Server (chỉ dùng khi target_addr không phải 0xFFFF).
 * @return True nếu lệnh được gửi thành công ít nhất một lần, false nếu không gửi được lệnh nào.
 */
bool ZigbeeCarControl::sendOnOffCommand(bool on, uint16_t target_addr, uint8_t target_ep) {
    // Nếu không có thiết bị nào được liên kết VÀ lệnh không phải broadcast, thì không thể gửi.
    if (_bound_fan_servers.empty() && target_addr == 0xFFFF) {
        log_e("Chua co thiet bi nao duoc lien ket. Khong the gui lenh On/Off.");
        return false;
    }

    bool command_sent_at_least_one = false;
    
    // 1. Chuẩn bị cấu trúc lệnh On/Off.
    esp_zb_zcl_on_off_cmd_t cmd = {0}; 
    
    // Gán ID lệnh (On/Off)
    cmd.on_off_cmd_id = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID; 

    // Quyết định gửi broadcast hay unicast
    if (target_addr == 0xFFFF) { // Gửi lệnh broadcast cho tất cả các thiết bị trong mạng
        log_i("Dang gui lenh On/Off (broadcast): %s", on ? "BAT" : "TAT");

        // THAY ĐỔI Ở ĐÂY: Sử dụng esp_err_t thay vì esp_zb_zcl_status_t
        esp_err_t api_status; 
        esp_zb_lock_acquire(portMAX_DELAY);
        
        // Cấu hình thông tin cơ bản cho lệnh broadcast
        cmd.zcl_basic_cmd = {
            .dst_addr_u = { .addr_short = 0xFFFD }, 
            .dst_endpoint = 0xFF,                   
            .src_endpoint = _endpoint,              
        };
        cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT; 
        
        // Gọi hàm gửi lệnh On/Off, truyền con trỏ tới cấu trúc cmd đã điền đầy đủ
        api_status = esp_zb_zcl_on_off_cmd_req(&cmd); 
        esp_zb_lock_release();

        // THAY ĐỔI Ở ĐÂY: Kiểm tra với ESP_OK
        if (api_status == ESP_OK) { 
            log_i("Da gui lenh On/Off broadcast thanh cong.");
            command_sent_at_least_one = true;
        } else {
            // Hiển thị mã lỗi API nếu gửi thất bại
            log_e("Khong gui duoc lenh On/Off broadcast, trang thai API: %d", api_status);
        }
    } else { // Gửi lệnh unicast đến thiết bị/endpoint cu the
        // Duyệt qua danh sách các thiết bị đã liên kết để tìm thiết bị mục tiêu
        for (const auto& server_params : _bound_fan_servers) {
            // Kiểm tra xem địa chỉ và endpoint có khớp với mục tiêu không
            if ((server_params.short_addr == target_addr) &&
                (target_ep == 0xFF || server_params.endpoint == target_ep)) { 
                
                log_i("Dang gui lenh On/Off toi 0x%04x/EP%d: %s", server_params.short_addr, server_params.endpoint, on ? "BAT" : "TAT");

                // THAY ĐỔI Ở ĐÂY: Sử dụng esp_err_t thay vì esp_zb_zcl_status_t
                esp_err_t api_status;
                esp_zb_lock_acquire(portMAX_DELAY);
                
                // Cấu hình thông tin cơ bản cho lệnh unicast
                cmd.zcl_basic_cmd = {
                    .dst_addr_u = { .addr_short = server_params.short_addr }, 
                    .dst_endpoint = server_params.endpoint,                   
                    .src_endpoint = _endpoint,                                
                };
                cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT; 

                // Gọi hàm gửi lệnh On/Off, truyền con trỏ tới cấu trúc cmd đã điền đầy đủ
                api_status = esp_zb_zcl_on_off_cmd_req(&cmd); 
                esp_zb_lock_release();

                // THAY ĐỔI Ở ĐÂY: Kiểm tra với ESP_OK
                if (api_status != ESP_OK) { 
                    log_e("Khong gui duoc lenh On/Off cho 0x%04x/EP%d, trang thai API: %d",
                          server_params.short_addr, server_params.endpoint, api_status);
                } else {
                    log_i("Da gui lenh On/Off thanh cong toi 0x%04x/EP%d.", server_params.short_addr, server_params.endpoint);
                    command_sent_at_least_one = true;
                }
                // Nếu target_ep được chỉ định cụ thể, sau khi gửi xong thì thoát vòng lặp
                if (target_ep != 0xFF) break; 
            }
        }
    }
    return command_sent_at_least_one;
}

/**
 * @brief Đặt mức độ (ví dụ: tốc độ motor) cho một thiết bị cụ thể hoặc tất cả các thiết bị đã liên kết
 * thông qua Level Control Cluster.
 * @param level Giá trị mức mong muốn (0-254).
 * @param transition_time Thời gian chuyển đổi đến mức mới theo đơn vị 1/10 giây (0 để chuyển ngay lập tức).
 * @param target_addr Địa chỉ ngắn của thiết bị mục tiêu (0xFFFF cho tất cả đã liên kết).
 * @param target_ep Endpoint của thiết bị mục tiêu chứa Level Control Server (chỉ dùng khi target_addr không phải 0xFFFF).
 * @return True nếu lệnh được gửi thành công ít nhất một lần, false nếu không gửi được lệnh nào.
 */
bool ZigbeeCarControl::sendLevelControlCommand(uint8_t level, uint16_t transition_time, uint16_t target_addr, uint8_t target_ep) {
    if (_bound_fan_servers.empty() && target_addr == 0xFFFF) {
        log_e("Chua co thiet bi nao duoc lien ket. Khong the gui lenh Level Control.");
        return false;
    }

    bool command_sent_at_least_one = false;

    // Xử lý trường hợp BROADCAST
    if (target_addr == 0xFFFF) {
        log_i("Dang gui lenh Level Control (broadcast): Level %d, Thoi gian %d (x0.1s)", level, transition_time);

        esp_zb_zcl_move_to_level_cmd_t cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u = { .addr_short = 0xFFFD }, // Broadcast address
                .dst_endpoint = 0xFF,                  // Broadcast endpoint
                .src_endpoint = _endpoint,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT, // Broadcast mode
            .level = level,
            .transition_time = transition_time
        };

        esp_zb_lock_acquire(portMAX_DELAY);
        uint8_t api_status = esp_zb_zcl_level_move_to_level_cmd_req(&cmd);
        esp_zb_lock_release();

        if (api_status == ESP_OK) {
            log_i("Da gui lenh Level Control broadcast thanh cong.");
            command_sent_at_least_one = true;
        } else {
            log_e("Khong gui duoc lenh Level Control broadcast, trang thai API: %d", api_status);
        }
    } else { // Xử lý trường hợp UNICAST (gửi đến thiết bị cụ thể)
        // Duyệt qua tất cả các thiết bị trong vector để tìm thiết bị mục tiêu
        for (const auto& server_params : _bound_fan_servers) {
            // Kiểm tra xem đây có phải là thiết bị mục tiêu hay không
            if ((server_params.short_addr == target_addr) &&
                (target_ep == 0xFF || server_params.endpoint == target_ep)) {

                log_i("Dang gui lenh Level Control toi 0x%04x/EP%d: Level %d, Thoi gian %d (x0.1s)",
                            server_params.short_addr, server_params.endpoint, level, transition_time);

                esp_zb_zcl_move_to_level_cmd_t cmd = {
                    .zcl_basic_cmd = {
                        .dst_addr_u = { .addr_short = server_params.short_addr },
                        .dst_endpoint = server_params.endpoint,
                        .src_endpoint = _endpoint,
                    },
                    .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                    .level = level,
                    .transition_time = transition_time
                };

                uint8_t api_status;
                esp_zb_lock_acquire(portMAX_DELAY);
                api_status = esp_zb_zcl_level_move_to_level_cmd_req(&cmd);
                esp_zb_lock_release();

                if (api_status == ESP_OK) {
                    log_i("Da gui lenh Level Control thanh cong toi 0x%04x/EP%d.", server_params.short_addr, server_params.endpoint);
                    command_sent_at_least_one = true;
                } else {
                    log_e("Khong gui duoc lenh Level Control cho 0x%04x/EP%d, trang thai API: %d",
                            server_params.short_addr, server_params.endpoint, api_status);
                }

                // Nếu chỉ điều khiển một thiết bị cụ thể, thoát vòng lặp
                if (target_ep != 0xFF) break;
            }
        }
    }
    return command_sent_at_least_one;
}
/**
 * @brief Cấu hình báo cáo thuộc tính On/Off cho một thiết bị cụ thể.
 * Yêu cầu thiết bị ZigbeeCarDevice gửi báo cáo về thuộc tính On/Off của nó.
 * @param target_addr Địa chỉ ngắn của thiết bị ZigbeeCarDevice.
 * @param target_ep Endpoint của ZigbeeCarDevice chứa On/Off Cluster cần báo cáo.
 * @return True nếu yêu cầu cấu hình được gửi thành công, false nếu ngược lại.
 */
bool ZigbeeCarControl::configureOnOffReporting(uint16_t target_addr, uint8_t target_ep) {
    log_i("Dang cau hinh bao cao On/Off cho thiet bi 0x%04x/EP%d...", target_addr, target_ep);

    esp_zb_zcl_config_report_cmd_t cmd_req = {0};

    // Điền thông tin cơ bản của lệnh ZCL
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = target_addr;
    cmd_req.zcl_basic_cmd.dst_endpoint = target_ep;
    cmd_req.zcl_basic_cmd.src_endpoint = _endpoint;
    
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT; // Chế độ địa chỉ cho unicast
    cmd_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;             // Cluster On/Off

    cmd_req.manuf_specific = 0; 
    cmd_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV; // Hướng của lệnh (Client -> Server)
    cmd_req.dis_defalut_resp = 0; 
    cmd_req.manuf_code = 0;       

    esp_zb_zcl_config_report_record_t record;

    // Sửa lỗi: Sử dụng enum đúng cho hướng báo cáo
    record.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND; // Client yêu cầu Server GỬI báo cáo
    
    // Sửa lỗi: Thuộc tính ID là thành viên trực tiếp
    record.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;  // Thuộc tính On/Off (current state)
    
    // Sửa lỗi: Các trường này nằm trong struct ẩn danh đầu tiên của union
    // Bạn truy cập chúng trực tiếp
    record.attrType = ESP_ZB_ZCL_ATTR_TYPE_BOOL;       // Kiểu dữ liệu Boolean
    record.min_interval = 1;                           // Báo cáo tối thiểu mỗi 1 giây
    record.max_interval = 60;                          // Báo cáo tối đa mỗi 60 giây (nếu không thay đổi)
    
    // Sửa lỗi: reportable_change là void*, không có .delta.u8
    // Đối với boolean, thay đổi nhỏ nhất là 1 (từ 0 sang 1 hoặc ngược lại)
    // Bạn có thể trỏ nó tới một biến uint8_t = 1, hoặc NULL nếu SDK xử lý đúng.
    // Cách an toàn hơn là tạo một biến static/global để trỏ đến.
    // Tuy nhiên, thường thì đối với boolean, giá trị này không được sử dụng
    // hoặc có thể đặt là 0 (hoặc NULL) để báo cáo mọi thay đổi.
    // Giả sử 0 sẽ là giá trị "thay đổi bất kỳ" cho boolean.
    static uint8_t reportable_bool_change = 0; // Thay đổi = 0 nghĩa là báo cáo mọi thay đổi
                                                // hoặc 1 nếu bạn muốn đảm bảo giá trị khác 0
    record.reportable_change = &reportable_bool_change; 


    cmd_req.record_field = &record; 
    cmd_req.record_number = 1;      

    esp_zb_lock_acquire(portMAX_DELAY);
    // Sửa lỗi: Tên hàm API
    uint8_t api_status = esp_zb_zcl_config_report_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (api_status == ESP_OK) { 
        log_i("Da gui yeu cau cau hinh bao cao On/Off thanh cong cho 0x%04x/EP%d", target_addr, target_ep);
        return true;
    } else {
        log_e("Khong gui duoc yeu cau cau hinh bao cao On/Off cho 0x%04x/EP%d, trang thai API: %d", target_addr, target_ep, api_status);
        return false;
    }
}

/**
 * @brief Cấu hình báo cáo thuộc tính Level Control cho một thiết bị cụ thể.
 * Yêu cầu thiết bị ZigbeeCarDevice gửi báo cáo về mức độ hiện tại của nó.
 * @param target_addr Địa chỉ ngắn của thiết bị ZigbeeCarDevice.
 * @param target_ep Endpoint của ZigbeeCarDevice chứa Level Control Cluster cần báo cáo.
 * @return True nếu yêu cầu cấu hình được gửi thành công, false nếu ngược lại.
 */
bool ZigbeeCarControl::configureLevelControlReporting(uint16_t target_addr, uint8_t target_ep) {
    log_i("Dang cau hinh bao cao Level Control cho thiet bi 0x%04x/EP%d...", target_addr, target_ep);

    // Sửa lỗi: Tên kiểu cấu trúc lệnh cấu hình báo cáo
    esp_zb_zcl_config_report_cmd_t cmd_req = {0};

    // Điền thông tin cơ bản của lệnh ZCL
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = target_addr;
    cmd_req.zcl_basic_cmd.dst_endpoint = target_ep;
    cmd_req.zcl_basic_cmd.src_endpoint = _endpoint;
    
    // Điền các trường khác của esp_zb_zcl_config_report_cmd_t
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT; // Chế độ địa chỉ cho unicast
    cmd_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;     // Cluster Level Control

    // Khởi tạo các cờ bit-field của lệnh
    cmd_req.manuf_specific = 0; 
    cmd_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV; // Hướng của lệnh (Client -> Server)
    cmd_req.dis_defalut_resp = 0; 
    cmd_req.manuf_code = 0;       

    // Sửa lỗi: Tên kiểu cấu trúc bản ghi báo cáo
    esp_zb_zcl_config_report_record_t record;

    // Sửa lỗi: Sử dụng enum đúng cho hướng báo cáo
    record.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND; // Client yêu cầu Server GỬI báo cáo
    
    // Sửa lỗi: Thuộc tính ID là thành viên trực tiếp
    record.attributeID = ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID; // Thuộc tính Current Level
    
    // Sửa lỗi: Các trường này nằm trong struct ẩn danh đầu tiên của union
    record.attrType = ESP_ZB_ZCL_ATTR_TYPE_U8;       // Kiểu dữ liệu U8
    record.min_interval = 1;                         // Báo cáo tối thiểu mỗi 1 giây
    record.max_interval = 60;                        // Báo cáo tối đa mỗi 60 giây (nếu không thay đổi)
    
    // reportable_change là void*, cần trỏ tới một biến chứa giá trị thay đổi.
    // Đối với U8, bạn có thể tạo một biến static uint8_t để lưu giá trị delta.
    static uint8_t reportable_level_change = 1; // Báo cáo nếu mức thay đổi ít nhất 1 đơn vị
    record.reportable_change = &reportable_level_change; 

    // Gán bản ghi và số lượng bản ghi vào cmd_req
    cmd_req.record_field = &record; 
    cmd_req.record_number = 1;      

    esp_zb_lock_acquire(portMAX_DELAY);
    // Sửa lỗi: Tên hàm API
    uint8_t api_status = esp_zb_zcl_config_report_cmd_req(&cmd_req);
    esp_zb_lock_release();

    // Kiểm tra trạng thái API (uint8_t, so sánh với ESP_OK)
    if (api_status == ESP_OK) { 
        log_i("Da gui yeu cau cau hinh bao cao Level Control thanh cong cho 0x%04x/EP%d", target_addr, target_ep);
        return true;
    } else {
        log_e("Khong gui duoc yeu cau cau hinh bao cao Level Control cho 0x%04x/EP%d, trang thai API: %d", target_addr, target_ep, api_status);
        return false;
    }
}

/**
 * @brief Cấu hình báo cáo thuộc tính Fan Mode cho một thiết bị cụ thể.
 * Yêu cầu thiết bị ZigbeeCarDevice gửi báo cáo về chế độ hoạt động hiện tại của GPIO.
 * @param target_addr Địa chỉ ngắn của thiết bị ZigbeeCarDevice.
 * @param target_ep Endpoint của ZigbeeCarDevice chứa Fan Control Cluster cần báo cáo.
 * @return True nếu yêu cầu cấu hình được gửi thành công, false nếu ngược lại.
 */
bool ZigbeeCarControl::configureFanModeReporting(uint16_t target_addr, uint8_t target_ep) {
    log_i("Dang cau hinh bao cao Fan Mode cho thiet bi 0x%04x/EP%d...", target_addr, target_ep);

    // Sửa lỗi: Tên kiểu cấu trúc lệnh cấu hình báo cáo
    esp_zb_zcl_config_report_cmd_t cmd_req = {0};

    // Điền thông tin cơ bản của lệnh ZCL
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = target_addr;
    cmd_req.zcl_basic_cmd.dst_endpoint = target_ep;
    cmd_req.zcl_basic_cmd.src_endpoint = _endpoint;
    
    // Điền các trường khác của esp_zb_zcl_config_report_cmd_t
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT; // Chế độ địa chỉ cho unicast
    cmd_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;       // Cluster Fan Control

    // Khởi tạo các cờ bit-field của lệnh
    cmd_req.manuf_specific = 0; 
    cmd_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV; // Hướng của lệnh (Client -> Server)
    cmd_req.dis_defalut_resp = 0; 
    cmd_req.manuf_code = 0;       

    // Sửa lỗi: Tên kiểu cấu trúc bản ghi báo cáo
    esp_zb_zcl_config_report_record_t record;

    // Sửa lỗi: Sử dụng enum đúng cho hướng báo cáo
    record.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND; // Client yêu cầu Server GỬI báo cáo
    
    // Sửa lỗi: Thuộc tính ID là thành viên trực tiếp
    record.attributeID = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID; // Thuộc tính Fan Mode
    
    // Sửa lỗi: Các trường này nằm trong struct ẩn danh đầu tiên của union
    record.attrType = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM;        // Kiểu dữ liệu Enum 8-bit
    record.min_interval = 1;                                 // Báo cáo tối thiểu mỗi 1 giây
    record.max_interval = 60;                                // Báo cáo tối đa mỗi 60 giây (nếu không thay đổi)
    
    // reportable_change là void*, cần trỏ tới một biến chứa giá trị thay đổi.
    // Đối với Enum (giống như boolean), mọi thay đổi thường được báo cáo.
    // Đặt delta = 0 hoặc trỏ tới một biến có giá trị 0 thường có nghĩa là "báo cáo mọi thay đổi".
    static uint8_t reportable_enum_change = 0; 
    record.reportable_change = &reportable_enum_change; 

    // Gán bản ghi và số lượng bản ghi vào cmd_req
    cmd_req.record_field = &record; 
    cmd_req.record_number = 1;      

    esp_zb_lock_acquire(portMAX_DELAY);
    // Sửa lỗi: Tên hàm API
    uint8_t api_status = esp_zb_zcl_config_report_cmd_req(&cmd_req);
    esp_zb_lock_release();

    // Kiểm tra trạng thái API (uint8_t, so sánh với ESP_OK)
    if (api_status == ESP_OK) { 
        log_i("Da gui yeu cau cau hinh bao cao Fan Mode thanh cong cho 0x%04x/EP%d", target_addr, target_ep);
        return true;
    } else {
        log_e("Khong gui duoc yeu cau cau hinh bao cao Fan Mode cho 0x%04x/EP%d, trang thai API: %d", target_addr, target_ep, api_status);
        return false;
    }
}


#endif // CONFIG_ZB_ENABLED
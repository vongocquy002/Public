#include <Arduino.h>
#include <WiFi.h>
#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "lvgl_v8_port.h"
//#include <demos/lv_demos.h>
#include "ui.h"
using namespace esp_panel::drivers;
using namespace esp_panel::board;

/**
 * To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 */
 // #include <demos/lv_demos.h>
 // #include <examples/lv_examples.h>

void setup()
{
    String title = "LVGL porting example";

    Serial.begin(115200);

    Serial.println("Initializing board");
    Board *board = new Board();
    board->init();

    #if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    // When avoid tearing function is enabled, the frame buffer number should be set in the board driver
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd->getBus();
    /**
     * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the
     * "bounce buffer" functionality to enhance the RGB data bandwidth.
     * This feature will consume `bounce_buffer_size * bytes_per_pixel * 2` of SRAM memory.
     */
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
    }
#endif
#endif
    assert(board->begin());

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    Serial.println("Creating UI");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    /**
     * Create the simple labels
     */
    // lv_obj_t *label_1 = lv_label_create(lv_scr_act());
    // lv_label_set_text(label_1, "Hello World!");
    // lv_obj_set_style_text_font(label_1, &lv_font_montserrat_30, 0);
    // lv_obj_align(label_1, LV_ALIGN_CENTER, 0, -20);
    // lv_obj_t *label_2 = lv_label_create(lv_scr_act());
    // lv_label_set_text_fmt(
    //     label_2, "ESP32_Display_Panel (%d.%d.%d)",
    //     ESP_PANEL_VERSION_MAJOR, ESP_PANEL_VERSION_MINOR, ESP_PANEL_VERSION_PATCH
    // );
    // lv_obj_set_style_text_font(label_2, &lv_font_montserrat_16, 0);
    // lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    // lv_obj_t *label_3 = lv_label_create(lv_scr_act());
    // lv_label_set_text_fmt(label_3, "LVGL (%d.%d.%d)", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    // lv_obj_set_style_text_font(label_3, &lv_font_montserrat_16, 0);
    // lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /**
     * Try an example. Don't forget to uncomment header.
     * See all the examples online: https://docs.lvgl.io/master/examples.html
     * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples
     */
    //  lv_example_btn_1();

    /**
     * Or try out a demo.
     * Don't forget to uncomment header and enable the demos in `lv_conf.h`. E.g. `LV_USE_DEMO_WIDGETS`
     */
    //lv_demo_widgets();
    // lv_demo_benchmark();
    // lv_demo_music();
    // lv_demo_stress();

    /* Release the mutex */
    lvgl_port_unlock();
    ui_init();
    Serial.println(title + " end");
}

void wifiscan(lv_event_t * e)
{
	Serial.println("Nut Scan Wi-Fi da duoc nhan!"); 

    // 1. Hiển thị trạng thái "Đang quét..." trên màn hình
    if (ui_TextArea1 != NULL) { // Luôn kiểm tra NULL để tránh lỗi
        lv_textarea_set_text(ui_TextArea1, "Dang quet Wi-Fi...");
        lv_timer_handler(); // Cập nhật UI ngay lập tức
    }
    // 2. Bắt đầu quét Wi-Fi
    int n = WiFi.scanNetworks(false, true); // false: không ẩn SSID, true: quét đồng bộ (blocking)
    Serial.printf("%d mang Wi-Fi tim thay.\n", n);
    // 3. Chuẩn bị chuỗi kết quả
    String wifiListString = "Danh sach Wi-Fi:\n";
    if (n == 0) {
        wifiListString += "Khong tim thay mang nao.";
    } else {
        for (int i = 0; i < n; ++i) {
            // Thêm SSID (tên mạng) và RSSI (cường độ tín hiệu)
            wifiListString += String(i + 1) + ". " + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)\n";
        }
    }

    // 4. Cập nhật lại Text Area với danh sách Wi-Fi
    if (ui_TextArea1 != NULL) {
        lv_textarea_set_text(ui_TextArea1, wifiListString.c_str());
        // Cuộn về đầu hoặc cuối tùy ý muốn, ví dụ cuộn về đầu:
        lv_textarea_set_cursor_pos(ui_TextArea1, 0); 
        lv_timer_handler(); // Cập nhật UI ngay lập tức
    }
}

void loop()
{
    Serial.println("IDLE loop");
    delay(1000);
}

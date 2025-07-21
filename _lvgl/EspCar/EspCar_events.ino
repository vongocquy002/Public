// Biến tĩnh để theo dõi trạng thái dropdown
static bool isDropdownOpen = false;
static int lastSelectedValue = -1; // Lưu giá trị trước đó

void Car1Mode(lv_event_t * e) {
  Serial.println("Car1Mode function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  lv_event_code_t event_code = lv_event_get_code(e);

  // Chỉ xử lý khi giá trị của dropdown thay đổi
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    Serial.println("LV_EVENT_VALUE_CHANGED detected!");
    int selectedIndex = lv_dropdown_get_selected(dropdown);
    String command; // Dùng String để dễ dàng thao tác

    // Lấy chuỗi từ lựa chọn dropdown (tùy chọn, để debug hoặc log)
    char buf[32];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    Serial.printf("Dropdown CAR1 Mode changed to: %s (Index: %d)\n", buf, selectedIndex);

    // Ánh xạ selectedIndex tới lệnh cho CAR1
    switch (selectedIndex) {
        case 0:
            command = "CAR1:mode:forward"; 
            break;
        case 1:
            command = "CAR1:mode:reverse";  
            break;
        case 2:
            command = "CAR1:mode:line_follow";
            break;
        case 3:
            command = "CAR1:mode:obstacle_avoid";
            break;
        // Thêm các case khác nếu có nhiều chế độ
        default:
            Serial.println("Lựa chọn chế độ CAR1 không xác định.");
            return; // Thoát nếu không phải lựa chọn hợp lệ
      }

      // --- Logic gửi lệnh tới CAR1 (tương tự như hàm sendCommandTask của bạn) ---
      if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
          bool sent = false;
          // Duyệt qua tất cả các client đã kết nối để tìm CAR1 (IP 192.168.4.2)
          for (int i = 0; i < maxClients; i++) {
              // Kiểm tra nếu client kết nối và IP của nó là của CAR1
              if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
                  clients[i].println(command);
                  String msg = String("Đã gửi lệnh ") + command + " đến Car1 (" + clients[i].remoteIP().toString() + ")";
                  Serial.println(msg); // In ra Serial của Server
                  if (dataQueue != NULL) {
                      xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY); // Gửi vào hàng đợi log
                  }
                  sent = true;
                  break; // Tìm thấy và gửi, thoát vòng lặp
              }
          }
          if (!sent) {
              String msg = "Không tìm thấy Car1 hoặc chưa kết nối để gửi lệnh: " + command;
              Serial.println(msg); // In ra Serial của Server
              if (dataQueue != NULL) {
                  xQueueSend(dataQueue, msg.c_str(), portMAX_DELAY); // Gửi vào hàng đợi log
              }
          }
          xSemaphoreGive(clientMutex); // Nhả semaphore
      } else {
          Serial.println("Không thể lấy semaphore để gửi lệnh từ UI!");
      }
  }
}

void Car1OnOff(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED); // Kiểm tra trạng thái ON
  const char* command = isOn ? "CAR1:on" : "CAR1:off";

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        clients[i].println(command);
        Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
        break;
      }
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Không thể lấy semaphore để gửi " + String(command));
  }
}

void Car1Speed(lv_event_t * e) {
  lv_obj_t* slider = lv_event_get_target(e);
  int speed = lv_slider_get_value(slider);
  char command[16];
  snprintf(command, sizeof(command), "CAR1:speed:%d", speed);
  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        clients[i].println(command);
        Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
        break;
      }
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Không thể lấy semaphore để gửi " + String(command));
  }
}

void Car1Io1Func(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED); // Kiểm tra trạng thái ON
  const char* command = isOn ? "CAR1:io1:on" : "CAR1:io1:off";

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        clients[i].println(command);
        Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
        break;
      }
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Không thể lấy semaphore để gửi " + String(command));
  }
}

void Car1Io2Func(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED); // Kiểm tra trạng thái ON
  const char* command = isOn ? "CAR1:io2:on" : "CAR1:io2:off";

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        clients[i].println(command);
        Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
        break;
      }
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Không thể lấy semaphore để gửi " + String(command));
  }
}

void Car1Io3Func(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED); // Kiểm tra trạng thái ON
  const char* command = isOn ? "CAR1:io3:on" : "CAR1:io3:off";

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        clients[i].println(command);
        Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
        break;
      }
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Không thể lấy semaphore để gửi " + String(command));
  }
}

void Car1Io4Func(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED); // Kiểm tra trạng thái ON
  const char* command = isOn ? "CAR1:io4:on" : "CAR1:io4:off";

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        clients[i].println(command);
        Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
        break;
      }
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Không thể lấy semaphore để gửi " + String(command));
  }
}

void Car1Io5Func(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED); // Kiểm tra trạng thái ON
  const char* command = isOn ? "CAR1:io5:on" : "CAR1:io5:off";

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        clients[i].println(command);
        Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
        break;
      }
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Không thể lấy semaphore để gửi " + String(command));
  }
}

void Car1Io6Func(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED); // Kiểm tra trạng thái ON
  const char* command = isOn ? "CAR1:io6:on" : "CAR1:io6:off";

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        clients[i].println(command);
        Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
        break;
      }
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Không thể lấy semaphore để gửi " + String(command));
  }
}

void OnOffAllCar(lv_event_t * e)
{
    lv_obj_t* sw = lv_event_get_target(e);
    bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED); // Kiểm tra trạng thái ON
    const char* command = isOn ? "ALL:on" : "ALL:off";

    if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 5; i++) {
            if (clients[i].connected()) {
                clients[i].println(command);
                Serial.println("Gửi lệnh " + String(command) + " thành công đến " + clients[i].remoteIP().toString());
            }
        }
        xSemaphoreGive(clientMutex);
    } else {
        Serial.println("Không thể lấy semaphore để gửi " + String(command));
    }
}
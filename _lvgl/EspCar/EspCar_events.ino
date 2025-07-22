// Sự kiện nút nhấn cho ON/OFF tất cả xe
void OnOffAllCar(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in OnOffAllCar!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[1] = {(uint8_t)(isOn ? 1 : 0)};

  Serial.printf("OnOffAllCar: %s\n", isOn ? "ON" : "OFF");
  sendCommandToAll(ON_OFF_ALL, payload, 1);
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent ON_OFF_ALL: %s to all cars", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, portMAX_DELAY);

  for (int i = 0; i < 5; i++) {
    carStatuses[i].isOn = isOn;
    saveCarStatus((CarID)i);
  }
}

// Sự kiện nút nhấn cho Speed All (Dropdown)
static bool isSpeedAllDropdownOpen = false;
static int lastSpeedAllValue = -1;

void SpeedAll(lv_event_t * e) {
  Serial.println("SpeedAll function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  if (!dropdown) return;
  lv_event_code_t event_code = lv_event_get_code(e);

  if (event_code == LV_EVENT_VALUE_CHANGED) {
    Serial.println("LV_EVENT_VALUE_CHANGED detected for SpeedAll!");
    int selectedIndex = lv_dropdown_get_selected(dropdown);
    uint8_t speed;

    char buf[32];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    Serial.printf("Dropdown SpeedAll changed to: %s (Index: %d)\n", buf, selectedIndex);

    switch (selectedIndex) {
      case 0: speed = 10; break;
      case 1: speed = 20; break;
      case 2: speed = 30; break;
      case 3: speed = 40; break;
      case 4: speed = 50; break;
      case 5: speed = 60; break;
      case 6: speed = 70; break;
      case 7: speed = 80; break;
      case 8: speed = 90; break;
      case 9: speed = 100; break;
      default:
        Serial.println("Invalid SpeedAll selection.");
        return;
    }

    uint8_t payload[1] = {speed}; // Define payload here
    sendCommandToAll(SPEED_ALL, payload, 1);
    char msg[64];
    snprintf(msg, sizeof(msg), "Sent SPEED_ALL: %d%% to all cars", speed);
    xQueueSend(dataQueue, msg, portMAX_DELAY);

    for (int i = 0; i < maxClients; i++) {
      carStatuses[i].speed = speed;
      saveCarStatus((CarID)i);
    }
  }
}

//----------------------------------------------------------------------------------------------------
// Biến tĩnh để theo dõi trạng thái dropdown Speed CAR1 (dropdown)
static bool isCar1ModeDropdownOpen = false;
static int lastCar1ModeValue = -1;

void Car1Mode(lv_event_t * e) {
  Serial.println("Car1Mode function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  if (!dropdown) return;
  lv_event_code_t event_code = lv_event_get_code(e);

  if (event_code == LV_EVENT_VALUE_CHANGED) {
    Serial.println("LV_EVENT_VALUE_CHANGED detected for Car1Mode!");
    int selectedIndex = lv_dropdown_get_selected(dropdown);
    uint8_t mode;

    char buf[32];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    Serial.printf("Dropdown CAR1 Mode changed to: %s (Index: %d)\n", buf, selectedIndex);

    switch (selectedIndex) {
      case 0: mode = 1; break; // forward
      case 1: mode = 2; break; // reverse
      case 2: mode = 3; break; // line_follow
      case 3: mode = 4; break; // obstacle_avoid
      default:
        Serial.println("Invalid Car1Mode selection.");
        return;
    }

    if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
      bool sent = false;
      for (int i = 0; i < maxClients; i++) {
        if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
          uint8_t payload[1] = {mode};
          sendCommand(clients[i], MODE, CAR1, payload, 1);
          char msg[64];
          snprintf(msg, sizeof(msg), "Sent MODE: %d to Car1", mode);
          xQueueSend(dataQueue, msg, portMAX_DELAY);
          sent = true;
          break;
        }
      }
      if (!sent) {
        char msg[64] = "Car1 not found for MODE";
        xQueueSend(dataQueue, msg, portMAX_DELAY);
      }
      xSemaphoreGive(clientMutex);
    } else {
      Serial.println("Cannot take semaphore for MODE!");
    }

    carStatuses[CAR1].mode = mode;
    saveCarStatus(CAR1);
  }
}

// Sự kiện nút nhấn cho ON/OFF CAR1
void Car1OnOff(lv_event_t * e) {
  Serial.println("Car1OnOff function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) return;
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[1] = {isOn ? 1 : 0};

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    bool sent = false;
    for (int i = 0; i < maxClients; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        sendCommand(clients[i], ON_OFF_CAR, CAR1, payload, 1);
        char msg[64];
        snprintf(msg, sizeof(msg), "Sent ON_OFF_CAR: %s to Car1", isOn ? "ON" : "OFF");
        xQueueSend(dataQueue, msg, portMAX_DELAY);
        sent = true;
        break;
      }
    }
    if (!sent) {
      char msg[64] = "Car1 not found for ON_OFF_CAR";
      xQueueSend(dataQueue, msg, portMAX_DELAY);
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Cannot take semaphore for ON_OFF_CAR!");
  }

  carStatuses[CAR1].isOn = isOn;
  saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho Speed CAR1 (Dropdown)
static bool isCar1SpeedDropdownOpen = false;
static int lastCar1SpeedValue = -1;

void Car1Speed(lv_event_t * e) {
  Serial.println("Car1Speed function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  if (!dropdown) return;
  lv_event_code_t event_code = lv_event_get_code(e);

  if (event_code == LV_EVENT_VALUE_CHANGED) {
    Serial.println("LV_EVENT_VALUE_CHANGED detected for Car1Speed!");
    int selectedIndex = lv_dropdown_get_selected(dropdown);
    uint8_t speed;

    char buf[32];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    Serial.printf("Dropdown Car1Speed changed to: %s (Index: %d)\n", buf, selectedIndex);

    switch (selectedIndex) {
      case 0: speed = 10; break;
      case 1: speed = 20; break;
      case 2: speed = 30; break;
      case 3: speed = 40; break;
      case 4: speed = 50; break;
      case 5: speed = 60; break;
      case 6: speed = 70; break;
      case 7: speed = 80; break;
      case 8: speed = 90; break;
      case 9: speed = 100; break;
      default:
        Serial.println("Invalid Car1Speed selection.");
        return;
    }

    if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
      bool sent = false;
      for (int i = 0; i < maxClients; i++) {
        if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
          uint8_t payload[1] = {speed};
          sendCommand(clients[i], SPEED_CAR, CAR1, payload, 1);
          char msg[64];
          snprintf(msg, sizeof(msg), "Sent SPEED_CAR: %d%% to Car1", speed);
          xQueueSend(dataQueue, msg, portMAX_DELAY);
          sent = true;
          break;
        }
      }
      if (!sent) {
        char msg[64] = "Car1 not found for SPEED_CAR";
        xQueueSend(dataQueue, msg, portMAX_DELAY);
      }
      xSemaphoreGive(clientMutex);
    } else {
      Serial.println("Cannot take semaphore for SPEED_CAR!");
    }

    carStatuses[CAR1].speed = speed;
    saveCarStatus(CAR1);
  }
}

// Sự kiện nút nhấn cho IO1 CAR1
void Car1Io1Func(lv_event_t * e) {
  Serial.println("Car1Io1Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) return;
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {isOn ? 1 : 0, carStatuses[CAR1].gpio[1], carStatuses[CAR1].gpio[2], carStatuses[CAR1].gpio[3], carStatuses[CAR1].gpio[4], carStatuses[CAR1].gpio[5]};

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    bool sent = false;
    for (int i = 0; i < maxClients; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        sendCommand(clients[i], GPIO_CONTROL, CAR1, payload, 6);
        char msg[64];
        snprintf(msg, sizeof(msg), "Sent IO1: %s to Car1", isOn ? "ON" : "OFF");
        xQueueSend(dataQueue, msg, portMAX_DELAY);
        sent = true;
        break;
      }
    }
    if (!sent) {
      char msg[64] = "Car1 not found for IO1";
      xQueueSend(dataQueue, msg, portMAX_DELAY);
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Cannot take semaphore for IO1!");
  }

  carStatuses[CAR1].gpio[0] = isOn ? 1 : 0;
  saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO2 CAR1
void Car1Io2Func(lv_event_t * e) {
  Serial.println("Car1Io2Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) return;
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], isOn ? 1 : 0, carStatuses[CAR1].gpio[2], carStatuses[CAR1].gpio[3], carStatuses[CAR1].gpio[4], carStatuses[CAR1].gpio[5]};

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    bool sent = false;
    for (int i = 0; i < maxClients; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        sendCommand(clients[i], GPIO_CONTROL, CAR1, payload, 6);
        char msg[64];
        snprintf(msg, sizeof(msg), "Sent IO2: %s to Car1", isOn ? "ON" : "OFF");
        xQueueSend(dataQueue, msg, portMAX_DELAY);
        sent = true;
        break;
      }
    }
    if (!sent) {
      char msg[64] = "Car1 not found for IO2";
      xQueueSend(dataQueue, msg, portMAX_DELAY);
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Cannot take semaphore for IO2!");
  }

  carStatuses[CAR1].gpio[1] = isOn ? 1 : 0;
  saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO3 CAR1
void Car1Io3Func(lv_event_t * e) {
  Serial.println("Car1Io3Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) return;
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], carStatuses[CAR1].gpio[1], isOn ? 1 : 0, carStatuses[CAR1].gpio[3], carStatuses[CAR1].gpio[4], carStatuses[CAR1].gpio[5]};

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    bool sent = false;
    for (int i = 0; i < maxClients; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        sendCommand(clients[i], GPIO_CONTROL, CAR1, payload, 6);
        char msg[64];
        snprintf(msg, sizeof(msg), "Sent IO3: %s to Car1", isOn ? "ON" : "OFF");
        xQueueSend(dataQueue, msg, portMAX_DELAY);
        sent = true;
        break;
      }
    }
    if (!sent) {
      char msg[64] = "Car1 not found for IO3";
      xQueueSend(dataQueue, msg, portMAX_DELAY);
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Cannot take semaphore for IO3!");
  }

  carStatuses[CAR1].gpio[2] = isOn ? 1 : 0;
  saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO4 CAR1
void Car1Io4Func(lv_event_t * e) {
  Serial.println("Car1Io4Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) return;
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], carStatuses[CAR1].gpio[1], carStatuses[CAR1].gpio[2], isOn ? 1 : 0, carStatuses[CAR1].gpio[4], carStatuses[CAR1].gpio[5]};

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    bool sent = false;
    for (int i = 0; i < maxClients; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        sendCommand(clients[i], GPIO_CONTROL, CAR1, payload, 6);
        char msg[64];
        snprintf(msg, sizeof(msg), "Sent IO4: %s to Car1", isOn ? "ON" : "OFF");
        xQueueSend(dataQueue, msg, portMAX_DELAY);
        sent = true;
        break;
      }
    }
    if (!sent) {
      char msg[64] = "Car1 not found for IO4";
      xQueueSend(dataQueue, msg, portMAX_DELAY);
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Cannot take semaphore for IO4!");
  }

  carStatuses[CAR1].gpio[3] = isOn ? 1 : 0;
  saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO5 CAR1
void Car1Io5Func(lv_event_t * e) {
  Serial.println("Car1Io5Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) return;
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], carStatuses[CAR1].gpio[1], carStatuses[CAR1].gpio[2], carStatuses[CAR1].gpio[3], isOn ? 1 : 0, carStatuses[CAR1].gpio[5]};

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    bool sent = false;
    for (int i = 0; i < maxClients; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        sendCommand(clients[i], GPIO_CONTROL, CAR1, payload, 6);
        char msg[64];
        snprintf(msg, sizeof(msg), "Sent IO5: %s to Car1", isOn ? "ON" : "OFF");
        xQueueSend(dataQueue, msg, portMAX_DELAY);
        sent = true;
        break;
      }
    }
    if (!sent) {
      char msg[64] = "Car1 not found for IO5";
      xQueueSend(dataQueue, msg, portMAX_DELAY);
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Cannot take semaphore for IO5!");
  }

  carStatuses[CAR1].gpio[4] = isOn ? 1 : 0;
  saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO6 CAR1
void Car1Io6Func(lv_event_t * e) {
  Serial.println("Car1Io6Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) return;
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], carStatuses[CAR1].gpio[1], carStatuses[CAR1].gpio[2], carStatuses[CAR1].gpio[3], carStatuses[CAR1].gpio[4], isOn ? 1 : 0};

  if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) {
    bool sent = false;
    for (int i = 0; i < maxClients; i++) {
      if (clients[i].connected() && clients[i].remoteIP() == IPAddress(192, 168, 4, 2)) {
        sendCommand(clients[i], GPIO_CONTROL, CAR1, payload, 6);
        char msg[64];
        snprintf(msg, sizeof(msg), "Sent IO6: %s to Car1", isOn ? "ON" : "OFF");
        xQueueSend(dataQueue, msg, portMAX_DELAY);
        sent = true;
        break;
      }
    }
    if (!sent) {
      char msg[64] = "Car1 not found for IO6";
      xQueueSend(dataQueue, msg, portMAX_DELAY);
    }
    xSemaphoreGive(clientMutex);
  } else {
    Serial.println("Cannot take semaphore for IO6!");
  }

  carStatuses[CAR1].gpio[5] = isOn ? 1 : 0;
  saveCarStatus(CAR1);
}


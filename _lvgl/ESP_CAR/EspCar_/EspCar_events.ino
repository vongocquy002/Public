// Sự kiện nút nhấn cho ON/OFF tất cả xe
void OnOffAllCar(lv_event_t * e) {
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in OnOffAllCar!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[1] = {(uint8_t)(isOn ? 1 : 0)};
  CommandData cmdData = {ON_OFF_ALL, ALL_CARS, {0}, 1};
  memcpy(cmdData.payload, payload, 1);
  Serial.printf(">>>>OnOffAllCar: %s\n", isOn ? "ON" : "OFF");
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent ON_OFF_ALL: %s to all cars", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  for (int i = 0; i < maxClients; i++) {
    carStatuses[i].isOn = isOn;
    // saveCarStatus((CarID)i);
  }
}

// Sự kiện nút nhấn cho Speed All (Dropdown)
static bool isSpeedAllDropdownOpen = false;
static int lastSpeedAllValue = -1;

void SpeedAll(lv_event_t * e) {
  Serial.println("SpeedAll function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  if (!dropdown) {
    return;
  }
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
    uint8_t payload[1] = {speed};
    CommandData cmdData = {SPEED_ALL, ALL_CARS, {0}, 1};
    memcpy(cmdData.payload, payload, 1);
    xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
    char msg[64];
    snprintf(msg, sizeof(msg), "Sent SPEED_ALL: %d%% to all cars", speed);
    xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
    for (int i = 0; i < maxClients; i++) {
      carStatuses[i].speed = speed;
      // saveCarStatus((CarID)i);
    }
  }
}

//----------------------------------------------------------------------------------------------------
//CAR1
// Biến tĩnh để theo dõi trạng thái dropdown Speed CAR1 (dropdown)
static bool isCar1ModeDropdownOpen = false;
static int lastCar1ModeValue = -1;

void Car1Mode(lv_event_t * e) {
  Serial.println("Car1Mode function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  if (!dropdown) {
    Serial.println("Error: Dropdown is NULL!");
    return;
  }
  lv_event_code_t event_code = lv_event_get_code(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    Serial.println("LV_EVENT_VALUE_CHANGED detected for Car1Mode!");
    int selectedIndex = lv_dropdown_get_selected(dropdown);
    uint8_t mode;
    char buf[32];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    Serial.printf("Dropdown CAR1 Mode changed to: %s (Index: %d)\n", buf, selectedIndex);
    switch (selectedIndex) {
      case 0: mode = 0; break; // Forward (Mode 1)
      case 1: mode = 1; break; // Reverse (Mode 2)
      default:
        Serial.println("Invalid Car1Mode selection.");
        return;
    }
    uint8_t payload[1] = {mode};
    CommandData cmdData = {MODE, CAR1, {0}, 1};
    memcpy(cmdData.payload, payload, 1);
    if (xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100)) != pdPASS) {
      Serial.println("Error: Failed to send MODE command to commandQueue!");
    } else {
      char msg[64];
      snprintf(msg, sizeof(msg), "Sent MODE: %d to Car1", mode);
      xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
      carStatuses[CAR1].mode = mode;
      saveCarStatus(CAR1);
      Serial.printf("Updated carStatuses[CAR1].mode: %d\n", mode);
    }
  }
}

// Sự kiện nút nhấn cho ON/OFF CAR1
void Car1OnOff(lv_event_t * e) {
  Serial.println("Car1OnOff function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car1OnOff!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[1] = {(uint8_t)(isOn ? 1 : 0)};
  CommandData cmdData = {ON_OFF_CAR, CAR1, {0}, 1};
  memcpy(cmdData.payload, payload, 1);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent ON_OFF_CAR: %s to Car1", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR1].isOn = isOn;
  // saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho Speed CAR1 (Dropdown)
static bool isCar1SpeedDropdownOpen = false;
static int lastCar1SpeedValue = -1;

void Car1Speed(lv_event_t * e) {
  Serial.println("Car1Speed function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  if (!dropdown) {
    return;
  }
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
    uint8_t payload[1] = {speed};
    CommandData cmdData = {SPEED_CAR, CAR1, {0}, 1};
    memcpy(cmdData.payload, payload, 1);
    xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
    char msg[64];
    snprintf(msg, sizeof(msg), "Sent SPEED_CAR: %d%% to Car1", speed);
    xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
    carStatuses[CAR1].speed = speed;
    // saveCarStatus(CAR1);
  }
}

// Sự kiện nút nhấn cho IO1 CAR1
void Car1Io1Func(lv_event_t * e) {
  Serial.println("Car1Io1Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car1Io1Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {(uint8_t)(isOn ? 1 : 0), carStatuses[CAR1].gpio[1], carStatuses[CAR1].gpio[2], 
                        carStatuses[CAR1].gpio[3], carStatuses[CAR1].gpio[4], carStatuses[CAR1].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR1, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO1: %s to Car1", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR1].gpio[0] = isOn ? 1 : 0;
  // saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO2 CAR1
void Car1Io2Func(lv_event_t * e) {
  Serial.println("Car1Io2Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car1Io2Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], (uint8_t)(isOn ? 1 : 0), carStatuses[CAR1].gpio[2], 
                        carStatuses[CAR1].gpio[3], carStatuses[CAR1].gpio[4], carStatuses[CAR1].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR1, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO2: %s to Car1", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR1].gpio[1] = isOn ? 1 : 0;
  // saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO3 CAR1
void Car1Io3Func(lv_event_t * e) {
  Serial.println("Car1Io3Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car1Io3Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], carStatuses[CAR1].gpio[1], (uint8_t)(isOn ? 1 : 0), 
                        carStatuses[CAR1].gpio[3], carStatuses[CAR1].gpio[4], carStatuses[CAR1].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR1, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO3: %s to Car1", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR1].gpio[2] = isOn ? 1 : 0;
  // saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO4 CAR1
void Car1Io4Func(lv_event_t * e) {
  Serial.println("Car1Io4Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car1Io4Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], carStatuses[CAR1].gpio[1], carStatuses[CAR1].gpio[2], 
                        (uint8_t)(isOn ? 1 : 0), carStatuses[CAR1].gpio[4], carStatuses[CAR1].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR1, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO4: %s to Car1", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR1].gpio[3] = isOn ? 1 : 0;
  // saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO5 CAR1
void Car1Io5Func(lv_event_t * e) {
  Serial.println("Car1Io5Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car1Io5Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], carStatuses[CAR1].gpio[1], carStatuses[CAR1].gpio[2], 
                        carStatuses[CAR1].gpio[3], (uint8_t)(isOn ? 1 : 0), carStatuses[CAR1].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR1, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO5: %s to Car1", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR1].gpio[4] = isOn ? 1 : 0;
  // saveCarStatus(CAR1);
}

// Sự kiện nút nhấn cho IO6 CAR1
void Car1Io6Func(lv_event_t * e) {
  Serial.println("Car1Io6Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car1Io6Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR1].gpio[0], carStatuses[CAR1].gpio[1], carStatuses[CAR1].gpio[2], 
                        carStatuses[CAR1].gpio[3], carStatuses[CAR1].gpio[4], (uint8_t)(isOn ? 1 : 0)};
  CommandData cmdData = {GPIO_CONTROL, CAR1, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO6: %s to Car1", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR1].gpio[5] = isOn ? 1 : 0;
  // saveCarStatus(CAR1);
}

//----------------------------------------------------------------------------------------------------
//CAR2
static bool isCar2ModeDropdownOpen = false;
static int lastCar2ModeValue = -1;

void Car2Mode(lv_event_t * e) {
  Serial.println("Car2Mode function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  if (!dropdown) {
    Serial.println("Error: Dropdown is NULL!");
    return;
  }
  lv_event_code_t event_code = lv_event_get_code(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    Serial.println("LV_EVENT_VALUE_CHANGED detected for Car2Mode!");
    int selectedIndex = lv_dropdown_get_selected(dropdown);
    uint8_t mode;
    char buf[32];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    Serial.printf("Dropdown CAR2 Mode changed to: %s (Index: %d)\n", buf, selectedIndex);
    switch (selectedIndex) {
      case 0: mode = 0; break; // Forward (Mode 1)
      case 1: mode = 1; break; // Reverse (Mode 2)
      default:
        Serial.println("Invalid Car2Mode selection.");
        return;
    }
    uint8_t payload[1] = {mode};
    CommandData cmdData = {MODE, CAR2, {0}, 1};
    memcpy(cmdData.payload, payload, 1);
    if (xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100)) != pdPASS) {
      Serial.println("Error: Failed to send MODE command to commandQueue!");
    } else {
      char msg[64];
      snprintf(msg, sizeof(msg), "Sent MODE: %d to Car2", mode);
      xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
      carStatuses[CAR2].mode = mode;
      saveCarStatus(CAR2);
      Serial.printf("Updated carStatuses[CAR2].mode: %d\n", mode);
    }
  }
}

// Sự kiện nút nhấn cho ON/OFF CAR2
void Car2OnOff(lv_event_t * e) {
  Serial.println("Car2OnOff function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car2OnOff!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[1] = {(uint8_t)(isOn ? 1 : 0)};
  CommandData cmdData = {ON_OFF_CAR, CAR2, {0}, 1};
  memcpy(cmdData.payload, payload, 1);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent ON_OFF_CAR: %s to Car2", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR2].isOn = isOn;
  // saveCarStatus(CAR2);
}

// Sự kiện nút nhấn cho Speed CAR2 (Dropdown)
static bool isCar2SpeedDropdownOpen = false;
static int lastCar2SpeedValue = -1;

void Car2Speed(lv_event_t * e) {
  Serial.println("Car2Speed function called!");
  lv_obj_t* dropdown = lv_event_get_target(e);
  if (!dropdown) {
    return;
  }
  lv_event_code_t event_code = lv_event_get_code(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    Serial.println("LV_EVENT_VALUE_CHANGED detected for Car2Speed!");
    int selectedIndex = lv_dropdown_get_selected(dropdown);
    uint8_t speed;
    char buf[32];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    Serial.printf("Dropdown Car2Speed changed to: %s (Index: %d)\n", buf, selectedIndex);
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
        Serial.println("Invalid Car2Speed selection.");
        return;
    }
    uint8_t payload[1] = {speed};
    CommandData cmdData = {SPEED_CAR, CAR2, {0}, 1};
    memcpy(cmdData.payload, payload, 1);
    xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
    char msg[64];
    snprintf(msg, sizeof(msg), "Sent SPEED_CAR: %d%% to Car2", speed);
    xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
    carStatuses[CAR2].speed = speed;
    // saveCarStatus(2);
  }
}

// Sự kiện nút nhấn cho IO1 CAR2
void Car2Io1Func(lv_event_t * e) {
  Serial.println("Car2Io1Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car2Io1Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {(uint8_t)(isOn ? 1 : 0), carStatuses[CAR2].gpio[1], carStatuses[CAR2].gpio[2], 
                        carStatuses[CAR2].gpio[3], carStatuses[CAR2].gpio[4], carStatuses[CAR2].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR2, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO1: %s to Car2", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR2].gpio[0] = isOn ? 1 : 0;
  // saveCarStatus(CAR2);
}

// Sự kiện nút nhấn cho IO2 CAR2
void Car2Io2Func(lv_event_t * e) {
  Serial.println("Car2Io2Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car2Io2Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR2].gpio[0], (uint8_t)(isOn ? 1 : 0), carStatuses[CAR2].gpio[2], 
                        carStatuses[CAR2].gpio[3], carStatuses[CAR2].gpio[4], carStatuses[CAR2].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR2, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO2: %s to Car2", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR2].gpio[1] = isOn ? 1 : 0;
  // saveCarStatus(CAR2);
}

// Sự kiện nút nhấn cho IO3 CAR2
void Car2Io3Func(lv_event_t * e) {
  Serial.println("Car2Io3Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car2Io3Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR2].gpio[0], carStatuses[CAR2].gpio[1], (uint8_t)(isOn ? 1 : 0), 
                        carStatuses[CAR2].gpio[3], carStatuses[CAR2].gpio[4], carStatuses[CAR2].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR2, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO3: %s to Car2", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR2].gpio[2] = isOn ? 1 : 0;
  // saveCarStatus(CAR2);
}

// Sự kiện nút nhấn cho IO4 CAR2
void Car2Io4Func(lv_event_t * e) {
  Serial.println("Car2Io4Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car2Io4Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR2].gpio[0], carStatuses[CAR2].gpio[1], carStatuses[CAR2].gpio[2], 
                        (uint8_t)(isOn ? 1 : 0), carStatuses[CAR2].gpio[4], carStatuses[CAR2].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR2, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO4: %s to Car2", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR2].gpio[3] = isOn ? 1 : 0;
  // saveCarStatus(CAR2);
}

// Sự kiện nút nhấn cho IO5 CAR2
void Car2Io5Func(lv_event_t * e) {
  Serial.println("Car2Io5Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car2Io5Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR2].gpio[0], carStatuses[CAR2].gpio[1], carStatuses[CAR2].gpio[2], 
                        carStatuses[CAR2].gpio[3], (uint8_t)(isOn ? 1 : 0), carStatuses[CAR2].gpio[5]};
  CommandData cmdData = {GPIO_CONTROL, CAR2, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO5: %s to Car2", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR2].gpio[4] = isOn ? 1 : 0;
  // saveCarStatus(CAR2);
}

// Sự kiện nút nhấn cho IO6 CAR2
void Car2Io6Func(lv_event_t * e) {
  Serial.println("Car2Io6Func function called!");
  lv_obj_t* sw = lv_event_get_target(e);
  if (!sw) {
    Serial.println("Invalid switch in Car2Io6Func!");
    return;
  }
  bool isOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
  uint8_t payload[6] = {carStatuses[CAR2].gpio[0], carStatuses[CAR2].gpio[1], carStatuses[CAR2].gpio[2], 
                        carStatuses[CAR2].gpio[3], carStatuses[CAR2].gpio[4], (uint8_t)(isOn ? 1 : 0)};
  CommandData cmdData = {GPIO_CONTROL, CAR2, {0}, 6};
  memcpy(cmdData.payload, payload, 6);
  xQueueSend(commandQueue, &cmdData, pdMS_TO_TICKS(100));
  char msg[64];
  snprintf(msg, sizeof(msg), "Sent IO6: %s to Car2", isOn ? "ON" : "OFF");
  xQueueSend(dataQueue, msg, pdMS_TO_TICKS(100));
  carStatuses[CAR2].gpio[5] = isOn ? 1 : 0;
  // saveCarStatus(CAR2);
}

//----------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------

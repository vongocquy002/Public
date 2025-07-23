#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <driver/ledc.h>

// WiFi configuration
const char* ssid = "ESP32_AP";
const char* password = "12345678";
const char* server_ip = "192.168.4.1";
const uint16_t server_port = 12345;

// Car configuration
enum CarID { CAR1, CAR2, CAR3, CAR4, CAR5, ALL_CARS = 0xFF };
const CarID car_id = CAR1;

// Command Types
enum CommandType {
  ON_OFF_ALL = 0x01,
  SPEED_ALL = 0x02,
  ON_OFF_CAR = 0x03,
  SPEED_CAR = 0x04,
  MODE = 0x05,
  GPIO_CONTROL = 0x06,
  STATUS_REPORT = 0x07,
  BAT_REPORT = 0x08,
  DIRECTION = 0x09 // New command for direction control
};

// GPIO and PWM configuration
const int motor_pwm_a = 20; // Matches test code
const int motor_pwm_b = 21; // Matches test code
const int gpio_pins[6] = {5, 6, 7, 8, 9, 10};
const int pwm_freq = 5000; // Matches test code (5 kHz)
const int pwm_resolution = 8;
const int pwm_channel_a = 0;
const int pwm_channel_b = 1;

// Car status
struct CarStatus {
  bool isOn;
  uint8_t speed;
  uint8_t mode;
  uint8_t gpio[6];
  bool all_on;
  uint8_t speed_all;
  uint8_t direction; // 0: Stop, 1: Forward, 2: Reverse
};

CarStatus car_status = {false, 0, 1, {0, 0, 0, 0, 0, 0}, true, 100, 1}; // Default direction: Forward
Preferences preferences;
WiFiClient client;

// Save car status to NVS
void saveCarStatus() {
  preferences.begin("car_status", false);
  preferences.putBool("isOn", car_status.isOn);
  preferences.putUChar("speed", car_status.speed);
  preferences.putUChar("mode", car_status.mode);
  for (int i = 0; i < 6; i++) {
    preferences.putUChar(("gpio" + String(i)).c_str(), car_status.gpio[i]);
  }
  preferences.putBool("all_on", car_status.all_on);
  preferences.putUChar("speed_all", car_status.speed_all);
  preferences.putUChar("direction", car_status.direction);
  preferences.end();
  Serial.println("Car status saved to NVS");
}

// Restore car status from NVS
void restoreCarStatus() {
  preferences.begin("car_status", true);
  car_status.isOn = preferences.getBool("isOn", false);
  car_status.speed = preferences.getUChar("speed", 0);
  car_status.mode = preferences.getUChar("mode", 1);
  for (int i = 0; i < 6; i++) {
    car_status.gpio[i] = preferences.getUChar(("gpio" + String(i)).c_str(), 0);
  }
  car_status.all_on = preferences.getBool("all_on", true);
  car_status.speed_all = preferences.getUChar("speed_all", 100);
  car_status.direction = preferences.getUChar("direction", 1); // Default: Forward
  preferences.end();
  Serial.println("Car status restored from NVS");
}

// Update motor based on isOn, all_on, speed, and direction
void updateMotor() {
  uint32_t duty = (car_status.isOn && car_status.all_on) ? (car_status.speed * 255) / 100 : 0;
  uint32_t final_pwm_a = 0;
  uint32_t final_pwm_b = 0;

  if (car_status.isOn && car_status.all_on) {
    if (car_status.direction == 1) { // Forward
      final_pwm_a = duty;
      Serial.println("Motor: FORWARD");
    } else if (car_status.direction == 2) { // Reverse
      final_pwm_b = duty;
      Serial.println("Motor: REVERSE");
    } else { // Stop
      Serial.println("Motor: STOP");
    }
  } else {
    Serial.println("Motor: STOP (isOn or all_on is false)");
  }

  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_a, final_pwm_a);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_b, final_pwm_b);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_a);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_b);
  Serial.printf("Motor updated: isOn=%d, all_on=%d, speed=%d, direction=%d, duty_a=%d, duty_b=%d\n", 
                car_status.isOn, car_status.all_on, car_status.speed, car_status.direction, final_pwm_a, final_pwm_b);
}

// Send command to server
void sendCommand(CommandType cmd, uint8_t* payload, uint8_t payloadLen) {
  if (!client.connected()) {
    Serial.println("Client not connected, cannot send command!");
    return;
  }
  while (client.available()) {
    client.read();
  }
  uint8_t frame[32];
  frame[0] = 0xAA;
  frame[1] = 0x01;
  frame[2] = cmd;
  frame[3] = car_id;
  frame[4] = payloadLen;
  memcpy(frame + 5, payload, payloadLen);
  uint8_t checksum = 0;
  for (int i = 0; i < 5 + payloadLen; i++) checksum += frame[i];
  frame[5 + payloadLen] = checksum;
  frame[6 + payloadLen] = 0xFF;
  Serial.printf("Sending command: Type=0x%02X, CarID=%d, Len=%d\n", cmd, car_id, payloadLen);
  Serial.print("Packet: ");
  for (int i = 0; i < 7 + payloadLen; i++) {
    if (frame[i] < 0x10) Serial.print("0");
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  if (client.write(frame, 7 + payloadLen) != 7 + payloadLen) {
    Serial.println("Failed to send command to server!");
  }
  client.clear();
}

// Send battery report
void sendBatteryReport() {
  uint8_t battery_level = random(0, 101);
  uint8_t payload[1] = {battery_level};
  sendCommand(BAT_REPORT, payload, 1);
  Serial.printf("Sent BAT_REPORT: %d%%\n", battery_level);
}

// Send status report
void sendStatusReport() {
  uint8_t payload[10] = {
    (uint8_t)car_status.isOn,
    car_status.speed,
    car_status.mode,
    car_status.gpio[0], car_status.gpio[1], car_status.gpio[2],
    car_status.gpio[3], car_status.gpio[4], car_status.gpio[5],
    car_status.direction
  };
  sendCommand(STATUS_REPORT, payload, 10);
  Serial.println("Sent STATUS_REPORT");
}

// Battery task
void batteryTask(void *pvParameters) {
  while (1) {
    if (client.connected()) {
      sendBatteryReport();
    } else {
      Serial.println("BatteryTask: Client not connected, skipping BAT_REPORT");
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// WiFi task
void wifiTask(void *pvParameters) {
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        Serial.print(".");
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nReconnected to WiFi");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("\nFailed to reconnect to WiFi");
      }
    }
    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
      Serial.println("Server disconnected, reconnecting...");
      client.stop();
      if (client.connect(server_ip, server_port)) {
        Serial.println("Reconnected to server");
      } else {
        Serial.println("Failed to reconnect to server");
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// Client task
void clientTask(void *pvParameters) {
  while (1) {
    if (!client.connected()) {
      Serial.println("ClientTask: Client not connected, waiting...");
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    if (client.available() >= 7) {
      uint8_t buffer[32];
      client.setTimeout(100);
      int len = client.readBytes(buffer, sizeof(buffer));
      Serial.printf("Received packet: len=%d\n", len);
      Serial.print("Packet: ");
      for (int i = 0; i < len; i++) {
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      // Sync frame, find AA 01
      int frameStart = -1;
      for (int i = 0; i < len - 1; i++) {
        if (buffer[i] == 0xAA && buffer[i+1] == 0x01) {
          frameStart = i;
          break;
        }
      }
      if (frameStart == -1 || frameStart + 7 > len) {
        Serial.println("No valid frame found!");
        while (client.available()) {
          client.read();
        }
        continue;
      }
      // Check frame length and footer
      int frameLen = buffer[frameStart + 4] + 7;
      if (frameStart + frameLen > len || buffer[frameStart + frameLen - 1] != 0xFF) {
        Serial.println("Invalid packet header or footer!");
        while (client.available()) {
          client.read();
        }
        continue;
      }
      uint8_t cmd = buffer[frameStart + 2];
      uint8_t carId = buffer[frameStart + 3];
      uint8_t dataLen = buffer[frameStart + 4];
      Serial.printf("Packet info: cmd=0x%02X, car_id=%d, dataLen=%d\n", cmd, carId, dataLen);
      if (frameLen >= 7 && frameLen <= len && (carId == car_id || carId == ALL_CARS)) {
        uint8_t checksum = 0;
        for (int i = frameStart; i < frameStart + frameLen - 2; i++) checksum += buffer[i];
        if (checksum == buffer[frameStart + frameLen - 2]) {
          Serial.print("Payload: ");
          for (int i = 0; i < dataLen; i++) {
            if (buffer[frameStart + 5 + i] < 0x10) Serial.print("0");
            Serial.print(buffer[frameStart + 5 + i], HEX);
            Serial.print(" ");
          }
          Serial.println();
          switch (cmd) {
            case ON_OFF_ALL:
              car_status.all_on = buffer[frameStart + 5];
              saveCarStatus();
              updateMotor();
              Serial.printf("Received ON_OFF_ALL: %s\n", car_status.all_on ? "ON" : "OFF");
              break;
            case ON_OFF_CAR:
              car_status.isOn = buffer[frameStart + 5];
              saveCarStatus();
              updateMotor();
              Serial.printf("Received ON_OFF_CAR: %s\n", car_status.isOn ? "ON" : "OFF");
              break;
            case SPEED_ALL:
              car_status.speed_all = buffer[frameStart + 5];
              saveCarStatus();
              updateMotor();
              Serial.printf("Received SPEED_ALL: %d%%\n", car_status.speed_all);
              break;
            case SPEED_CAR:
              car_status.speed = buffer[frameStart + 5];
              saveCarStatus();
              updateMotor();
              Serial.printf("Received SPEED_CAR: %d%%\n", car_status.speed);
              break;
            case MODE:
              car_status.mode = buffer[frameStart + 5];
              saveCarStatus();
              Serial.printf("Received MODE: %d\n", car_status.mode);
              break;
            case GPIO_CONTROL:
              for (int i = 0; i < 6; i++) car_status.gpio[i] = buffer[frameStart + 5 + i];
              saveCarStatus();
              Serial.printf("Received GPIO_CONTROL: %d %d %d %d %d %d\n", 
                            car_status.gpio[0], car_status.gpio[1], car_status.gpio[2],
                            car_status.gpio[3], car_status.gpio[4], car_status.gpio[5]);
              break;
            case DIRECTION:
              car_status.direction = buffer[frameStart + 5]; // 0: Stop, 1: Forward, 2: Reverse
              saveCarStatus();
              updateMotor();
              Serial.printf("Received DIRECTION: %d\n", car_status.direction);
              break;
          }
          client.write("OK", 2);
          client.clear();
          vTaskDelay(10 / portTICK_PERIOD_MS);
          sendStatusReport();
        } else {
          Serial.println("Checksum error!");
        }
      } else {
        Serial.println("Invalid packet format or car ID!");
        while (client.available()) {
          client.read();
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting client setup");

  // Configure GPIO and PWM
  for (int i = 0; i < 6; i++) {
    pinMode(gpio_pins[i], OUTPUT);
    digitalWrite(gpio_pins[i], LOW);
  }
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = (ledc_timer_bit_t)pwm_resolution,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = pwm_freq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  if (ledc_timer_config(&ledc_timer) != ESP_OK) {
    Serial.println("Failed to configure LEDC timer");
  }
  ledc_channel_config_t ledc_channel_a = {
    .gpio_num = motor_pwm_a,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)pwm_channel_a,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config_t ledc_channel_b = {
    .gpio_num = motor_pwm_b,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)pwm_channel_b,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  if (ledc_channel_config(&ledc_channel_a) != ESP_OK || ledc_channel_config(&ledc_channel_b) != ESP_OK) {
    Serial.println("Failed to configure LEDC channels");
  }

  // Test motor for 5 seconds
  Serial.println("Testing motor for 5 seconds...");
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_a, 128); // 50% duty, forward
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_a);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_b, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_b);
  delay(5000);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_a, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_b, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_a);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwm_channel_b);
  Serial.println("Motor test complete");

  // Restore car status and update motor
  Serial.println("Restoring car status...");
  restoreCarStatus();
  updateMotor();

  // Connect to WiFi with timeout
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi");
    while (1) { delay(1000); }
  }
  Serial.println("\nConnected to WiFi");
  Serial.println(WiFi.localIP());

  // Connect to server with timeout
  start = millis();
  while (!client.connect(server_ip, server_port) && millis() - start < 15000) {
    Serial.println("Connecting to server...");
    delay(1000);
  }
  if (client.connected()) {
    Serial.println("Connected to server");
  } else {
    Serial.println("Failed to connect to server");
    while (1) { delay(1000); }
  }

  // Create tasks
  Serial.println("Creating tasks...");
  if (xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 2, NULL) != pdPASS) {
    Serial.println("Failed to create WiFiTask");
  }
  if (xTaskCreate(clientTask, "ClientTask", 4096, NULL, 1, NULL) != pdPASS) {
    Serial.println("Failed to create ClientTask");
  }
  if (xTaskCreate(batteryTask, "BatteryTask", 2048, NULL, 1, NULL) != pdPASS) {
    Serial.println("Failed to create BatteryTask");
  }

  Serial.printf("Setup complete. Free heap: %d bytes\n", ESP.getFreeHeap());
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last > 5000) {
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    last = millis();
  }
  delay(100);
}
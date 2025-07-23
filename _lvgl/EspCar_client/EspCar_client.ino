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
  BAT_REPORT = 0x08
};

// GPIO and PWM configuration
const int motor_pwm_a = 18; // Thay GPIO 26 bằng 18 (an toàn trên ESP32-C6)
const int motor_pwm_b = 19; // Thay GPIO 27 bằng 19
const int gpio_pins[6] = {5, 6, 7, 8, 9, 10};
const int pwm_freq = 1000;
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
};

CarStatus car_status = {false, 0, 1, {0, 0, 0, 0, 0, 0}, true, 100};
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
  preferences.end();
  Serial.println("Car status restored from NVS");
}

// Send command to server
void sendCommand(CommandType cmd, uint8_t* payload, uint8_t payloadLen) {
  if (!client.connected()) {
    Serial.println("Client not connected, cannot send command!");
    return;
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
}

// Send battery report
void sendBatteryReport() {
  uint8_t battery_level = random(0, 101); // Giả lập pin
  uint8_t payload[1] = {battery_level};
  sendCommand(BAT_REPORT, payload, 1);
  Serial.printf("Sent BAT_REPORT: %d%%\n", battery_level);
}

// Battery task
void batteryTask(void *pvParameters) {
  while (1) {
    if (client.connected()) {
      sendBatteryReport();
    } else {
      Serial.println("BatteryTask: Client not connected, skipping BAT_REPORT");
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Gửi mỗi 5 giây
  }
}

// WiFi task
void wifiTask(void *pvParameters) {
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.reconnect();
    } else if (!client.connected()) {
      Serial.println("Server disconnected, reconnecting...");
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
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    if (client.available() >= 7) {
      uint8_t buffer[32];
      int len = client.readBytes(buffer, sizeof(buffer));
      Serial.printf("Received packet: len=%d\n", len);
      Serial.print("Packet: ");
      for (int i = 0; i < len; i++) {
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      if (len >= 7 && buffer[0] == 0xAA && buffer[1] == 0x01 && buffer[len-1] == 0xFF) {
        uint8_t cmd = buffer[2];
        uint8_t carId = buffer[3];
        uint8_t dataLen = buffer[4];
        Serial.printf("Packet info: cmd=0x%02X, car_id=%d, dataLen=%d\n", cmd, carId, dataLen);
        if (len >= 6 + dataLen && (carId == car_id || carId == ALL_CARS)) {
          uint8_t checksum = 0;
          for (int i = 0; i < len-2; i++) checksum += buffer[i];
          if (checksum == buffer[len-2]) {
            Serial.print("Payload: ");
            for (int i = 0; i < dataLen; i++) {
              if (buffer[5+i] < 0x10) Serial.print("0");
              Serial.print(buffer[5+i], HEX);
              Serial.print(" ");
            }
            Serial.println();
            switch (cmd) {
              case ON_OFF_ALL:
              case ON_OFF_CAR:
                car_status.isOn = buffer[5];
                saveCarStatus();
                Serial.printf("Received ON_OFF_%s: %s\n", cmd == ON_OFF_ALL ? "ALL" : "CAR", car_status.isOn ? "ON" : "OFF");
                break;
              case SPEED_ALL:
              case SPEED_CAR:
                car_status.speed = buffer[5];
                saveCarStatus();
                Serial.printf("Received SPEED_%s: %d%%\n", cmd == SPEED_ALL ? "ALL" : "CAR", car_status.speed);
                break;
              case MODE:
                car_status.mode = buffer[5];
                saveCarStatus();
                Serial.printf("Received MODE: %d\n", car_status.mode);
                break;
              case GPIO_CONTROL:
                for (int i = 0; i < 6; i++) car_status.gpio[i] = buffer[5 + i];
                saveCarStatus();
                Serial.printf("Received GPIO_CONTROL: %d %d %d %d %d %d\n", 
                              car_status.gpio[0], car_status.gpio[1], car_status.gpio[2],
                              car_status.gpio[3], car_status.gpio[4], car_status.gpio[5]);
                break;
            }
            client.write("OK", 2);
            // Send STATUS_REPORT after updating status
            uint8_t payload[9] = {
              (uint8_t)car_status.isOn, car_status.speed, car_status.mode,
              car_status.gpio[0], car_status.gpio[1], car_status.gpio[2],
              car_status.gpio[3], car_status.gpio[4], car_status.gpio[5]
            };
            sendCommand(STATUS_REPORT, payload, 9);
          } else {
            Serial.println("Checksum error!");
          }
        } else {
          Serial.println("Invalid packet format or car ID!");
        }
      } else {
        Serial.println("Invalid packet header or footer!");
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Tăng delay để nhường CPU
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

  // Restore car status
  Serial.println("Restoring car status...");
  restoreCarStatus();

  // Connect to WiFi with timeout
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi");
    return;
  }
  Serial.println("\nConnected to WiFi");
  Serial.println(WiFi.localIP());

  // Connect to server with timeout
  start = millis();
  while (!client.connect(server_ip, server_port) && millis() - start < 10000) {
    Serial.println("Connecting to server...");
    delay(500);
  }
  if (client.connected()) {
    Serial.println("Connected to server");
  } else {
    Serial.println("Failed to connect to server");
    return;
  }

  // Create tasks
  Serial.println("Creating tasks...");
  if (xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 2, NULL) != pdPASS) {
    Serial.println("Failed to create WiFiTask");
  }
  if (xTaskCreate(clientTask, "ClientTask", 4096, NULL, 1, NULL) != pdPASS) {
    Serial.println("Failed to create ClientTask");
  }
  // if (xTaskCreate(batteryTask, "BatteryTask", 2048, NULL, 1, NULL) != pdPASS) {
  //   Serial.println("Failed to create BatteryTask");
  // }

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
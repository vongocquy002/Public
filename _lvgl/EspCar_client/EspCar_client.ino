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
enum CarID { CAR1, CAR2, CAR3, CAR4, CAR5, ALL_CARS = 0xFF }; // Thêm ALL_CARS
const CarID car_id = CAR1; // Cấu hình tĩnh cho CAR1

// Command Types
enum CommandType {
  ON_OFF_ALL = 0x01,
  SPEED_ALL = 0x02,
  ON_OFF_CAR = 0x03,
  SPEED_CAR = 0x04,
  MODE = 0x05,
  GPIO_CONTROL = 0x06,
  STATUS_REPORT = 0x07
};

// GPIO configuration
const int motor_pwm_a = 20; // GPIO 26 for motor forward
const int motor_pwm_b = 21; // GPIO 27 for motor reverse
const int gpio_pins[6] = {5, 6, 7, 8, 9, 10}; // GPIO 5-10

// PWM configuration
const int pwm_freq = 1000; // 1kHz
const int pwm_resolution = 8; // 8-bit
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

// In gói tin dưới dạng hex
void printHexPacket(const uint8_t* buffer, int len) {
  Serial.print("Packet: ");
  for (int i = 0; i < len; i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// Tạo và gửi gói tin
void sendCommand(CommandType cmd, uint8_t* payload, uint8_t payloadLen) {
  if (!client.connected()) {
    Serial.println("Client not connected in sendCommand!");
    return;
  }
  uint8_t frame[64];
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
  printHexPacket(frame, 7 + payloadLen);
  client.write(frame, 7 + payloadLen);
}

// Gửi STATUS_REPORT
void sendStatusReport() {
  uint8_t payload[9];
  payload[0] = car_status.isOn ? 1 : 0;
  payload[1] = car_status.speed;
  payload[2] = car_status.mode;
  memcpy(payload + 3, car_status.gpio, 6);
  sendCommand(STATUS_REPORT, payload, 9);
}

// Điều khiển motor
void controlMotor() {
  if (!car_status.all_on || !car_status.isOn) {
    ledcWrite(pwm_channel_a, 0);
    ledcWrite(pwm_channel_b, 0);
    Serial.println("Motor stopped (all_on or isOn is false)");
    return;
  }

  uint8_t effective_speed = car_status.speed * car_status.speed_all / 100;
  uint8_t pwm_value = effective_speed * 255 / 100;

  if (car_status.mode == 1) {
    ledcWrite(pwm_channel_a, pwm_value);
    ledcWrite(pwm_channel_b, 0);
    Serial.printf("Motor forward, speed=%d%%\n", effective_speed);
  } else if (car_status.mode == 2) {
    ledcWrite(pwm_channel_a, 0);
    ledcWrite(pwm_channel_b, pwm_value);
    Serial.printf("Motor reverse, speed=%d%%\n", effective_speed);
  } else {
    ledcWrite(pwm_channel_a, 0);
    ledcWrite(pwm_channel_b, 0);
    Serial.println("Invalid mode, motor stopped");
  }
}

// Lưu trạng thái vào NVS
void saveCarStatus() {
  preferences.begin("car_status", false);
  char key[16];
  snprintf(key, sizeof(key), "car%d", car_id);
  preferences.putBool((String(key) + "_on").c_str(), car_status.isOn);
  preferences.putUChar((String(key) + "_speed").c_str(), car_status.speed);
  preferences.putUChar((String(key) + "_mode").c_str(), car_status.mode);
  for (int i = 0; i < 6; i++) {
    snprintf(key, sizeof(key), "car%d_gpio%d", car_id, i);
    preferences.putUChar(key, car_status.gpio[i]);
  }
  preferences.putBool((String(key) + "_all_on").c_str(), car_status.all_on);
  preferences.putUChar((String(key) + "_speed_all").c_str(), car_status.speed_all);
  preferences.end();
  Serial.printf("Car %d status saved to NVS\n", car_id + 1);
}

// Khôi phục trạng thái từ NVS
void restoreCarStatus() {
  preferences.begin("car_status", true);
  char key[16];
  snprintf(key, sizeof(key), "car%d", car_id);
  car_status.isOn = preferences.getBool((String(key) + "_on").c_str(), false);
  car_status.speed = preferences.getUChar((String(key) + "_speed").c_str(), 0);
  car_status.mode = preferences.getUChar((String(key) + "_mode").c_str(), 1);
  for (int i = 0; i < 6; i++) {
    snprintf(key, sizeof(key), "car%d_gpio%d", car_id, i);
    car_status.gpio[i] = preferences.getUChar(key, 0);
  }
  car_status.all_on = preferences.getBool((String(key) + "_all_on").c_str(), true);
  car_status.speed_all = preferences.getUChar((String(key) + "_speed_all").c_str(), 100);
  preferences.end();
  Serial.println("Car status restored from NVS");
}

// Task xử lý kết nối WiFi và server
void wifiTask(void *pvParameters) {
  WiFi.mode(WIFI_STA); // Đảm bảo chế độ Station
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  const int max_attempts = 20; // Timeout sau 10 giây (20 * 500ms)
  while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
    vTaskDelay(500 / portTICK_PERIOD_MS); // Nhường CPU
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi, retrying in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    WiFi.disconnect();
    WiFi.begin(ssid, password); // Thử lại
    return;
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("Client IP: ");
  Serial.println(WiFi.localIP());

  while (1) {
    if (!client.connected()) {
      if (client.connect(server_ip, server_port)) {
        Serial.println("Connected to server");
        sendStatusReport(); // Gửi trạng thái ban đầu
      } else {
        Serial.println("Failed to connect to server, retrying...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task xử lý gói tin từ server
void clientTask(void *pvParameters) {
  while (1) {
    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    if (stackHighWaterMark < 512) {
      Serial.printf("Warning: Low stack in ClientTask: %d bytes\n", stackHighWaterMark);
    }

    if (client.connected()) {
      // Xử lý phản hồi "OK" (2 bytes) từ server
      while (client.available() >= 2 && client.peek() != 0xAA) {
        char ok[2];
        if (client.readBytes(ok, 2) == 2 && ok[0] == 'O' && ok[1] == 'K') {
          Serial.println("Received OK response from server");
        } else {
          Serial.println("Discarded invalid data");
          char buf[2];
          client.readBytes(buf, 2);
          Serial.printf("Invalid data: %02X %02X\n", buf[0], buf[1]);
          client.flush();
        }
      }

      // Xử lý gói tin định dạng chuẩn
      if (client.available() >= 7) {
        uint8_t buffer[64];
        int len = client.readBytes(buffer, sizeof(buffer));
        Serial.printf("Received packet: len=%d\n", len);
        printHexPacket(buffer, len);
        if (len >= 7 && buffer[0] == 0xAA && buffer[1] == 0x01 && buffer[len-1] == 0xFF) {
          uint8_t cmd = buffer[2];
          uint8_t received_car_id = buffer[3];
          uint8_t dataLen = buffer[4];
          Serial.printf("Packet info: cmd=0x%02X, car_id=%d, dataLen=%d\n", cmd, received_car_id, dataLen);
          if (len >= 6 + dataLen && (received_car_id == car_id || received_car_id == ALL_CARS)) {
            uint8_t checksum = 0;
            for (int i = 0; i < len-2; i++) checksum += buffer[i];
            if (checksum == buffer[len-2]) {
              Serial.print("Payload: ");
              for (int i = 0; i < dataLen; i++) {
                if (buffer[5 + i] < 0x10) Serial.print("0");
                Serial.print(buffer[5 + i], HEX);
                Serial.print(" ");
              }
              Serial.println();
              switch (cmd) {
                case ON_OFF_ALL:
                  if (dataLen == 1) {
                    car_status.all_on = buffer[5];
                    Serial.printf("Received ON_OFF_ALL: %s\n", car_status.all_on ? "ON" : "OFF");
                    controlMotor();
                    saveCarStatus();
                    sendStatusReport();
                  } else {
                    Serial.println("Invalid ON_OFF_ALL dataLen");
                  }
                  break;
                case SPEED_ALL:
                  if (dataLen == 1) {
                    car_status.speed_all = buffer[5];
                    Serial.printf("Received SPEED_ALL: %d%%\n", car_status.speed_all);
                    controlMotor();
                    saveCarStatus();
                    sendStatusReport();
                  } else {
                    Serial.println("Invalid SPEED_ALL dataLen");
                  }
                  break;
                case ON_OFF_CAR:
                  if (dataLen == 1) {
                    car_status.isOn = buffer[5];
                    Serial.printf("Received ON_OFF_CAR: %s\n", car_status.isOn ? "ON" : "OFF");
                    controlMotor();
                    saveCarStatus();
                    sendStatusReport();
                  } else {
                    Serial.println("Invalid ON_OFF_CAR dataLen");
                  }
                  break;
                case SPEED_CAR:
                  if (dataLen == 1) {
                    car_status.speed = buffer[5];
                    Serial.printf("Received SPEED_CAR: %d%%\n", car_status.speed);
                    controlMotor();
                    saveCarStatus();
                    sendStatusReport();
                  } else {
                    Serial.println("Invalid SPEED_CAR dataLen");
                  }
                  break;
                case MODE:
                  if (dataLen == 1) {
                    car_status.mode = buffer[5];
                    Serial.printf("Received MODE: %d\n", car_status.mode);
                    controlMotor();
                    saveCarStatus();
                    sendStatusReport();
                  } else {
                    Serial.println("Invalid MODE dataLen");
                  }
                  break;
                case GPIO_CONTROL:
                  if (dataLen == 6) {
                    for (int i = 0; i < 6; i++) {
                      car_status.gpio[i] = buffer[5 + i];
                      digitalWrite(gpio_pins[i], car_status.gpio[i]);
                    }
                    Serial.println("Received GPIO_CONTROL");
                    saveCarStatus();
                    sendStatusReport();
                  } else {
                    Serial.println("Invalid GPIO_CONTROL dataLen");
                  }
                  break;
              }
              client.write("OK", 2); // Gửi phản hồi OK cho server
            } else {
              Serial.printf("Checksum error: calculated=0x%02X, received=0x%02X\n", checksum, buffer[len-2]);
              client.flush();
            }
          } else {
            Serial.println("Invalid packet format or car ID");
            client.flush();
          }
        } else {
          Serial.println("Invalid packet format");
          client.flush();
        }
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting client setup");

  // Kiểm tra GPIO trước khi cấu hình
  Serial.println("Configuring GPIOs...");
  for (int i = 0; i < 6; i++) {
    Serial.printf("Setting GPIO %d as OUTPUT\n", gpio_pins[i]);
    pinMode(gpio_pins[i], OUTPUT);
    digitalWrite(gpio_pins[i], LOW);
  }

  // Thiết lập PWM cho motor
  Serial.println("Configuring PWM...");
  ledc_timer_config_t ledc_timer = {
    LEDC_LOW_SPEED_MODE,           // speed_mode
    (ledc_timer_bit_t)pwm_resolution, // duty_resolution
    LEDC_TIMER_0,                 // timer_num
    pwm_freq,                     // freq_hz
    LEDC_AUTO_CLK                 // clk_cfg
  };
  if (ledc_timer_config(&ledc_timer) != ESP_OK) {
    Serial.println("Failed to configure LEDC timer!");
    return;
  }

  ledc_channel_config_t ledc_channel_a = {
    motor_pwm_a,                  // gpio_num
    LEDC_LOW_SPEED_MODE,          // speed_mode
    (ledc_channel_t)pwm_channel_a, // channel
    LEDC_INTR_DISABLE,            // intr_type
    LEDC_TIMER_0,                 // timer_sel
    0,                            // duty
    0                             // hpoint
  };
  if (ledc_channel_config(&ledc_channel_a) != ESP_OK) {
    Serial.println("Failed to configure LEDC channel A!");
    return;
  }

  ledc_channel_config_t ledc_channel_b = {
    motor_pwm_b,                  // gpio_num
    LEDC_LOW_SPEED_MODE,          // speed_mode
    (ledc_channel_t)pwm_channel_b, // channel
    LEDC_INTR_DISABLE,            // intr_type
    LEDC_TIMER_0,                 // timer_sel
    0,                            // duty
    0                             // hpoint
  };
  if (ledc_channel_config(&ledc_channel_b) != ESP_OK) {
    Serial.println("Failed to configure LEDC channel B!");
    return;
  }

  // Khôi phục trạng thái từ NVS
  Serial.println("Restoring car status...");
  restoreCarStatus();

  // Áp dụng trạng thái GPIO
  Serial.println("Applying GPIO states...");
  for (int i = 0; i < 6; i++) {
    digitalWrite(gpio_pins[i], car_status.gpio[i]);
  }

  // Áp dụng trạng thái motor
  Serial.println("Applying motor state...");
  controlMotor();

  // Tăng stack size và tạo task
  Serial.println("Creating tasks...");
  xTaskCreate(wifiTask, "WiFiTask", 8192, NULL, 2, NULL); // Tăng stack size
  xTaskCreate(clientTask, "ClientTask", 8192, NULL, 1, NULL); // Tăng stack size

  Serial.printf("Setup complete. Free heap: %d bytes\n", ESP.getFreeHeap());
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last > 5000) {
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    last = millis();
  }
}
// Blinks an LED attached to a MCP23XXX pin.

// ok to include only the one needed
// both included here to make things simple for example
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include <Wire.h> // Bao gồm thư viện Wire cho I2C

#define LED_PIN 4     // gpa4

// Định nghĩa chân SDA và SCL tùy chỉnh
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 32

// only used for SPI - KHÔNG DÙNG CHO I2C
// #define CS_PIN 6

// uncomment appropriate line
// Adafruit_MCP23X08 mcp;
Adafruit_MCP23X17 mcp; // Sử dụng MCP23X17 (ví dụ: MCP23017)

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  Serial.println("MCP23xxx Blink Test!");

  // Khởi tạo giao tiếp I2C với các chân tùy chỉnh
  // Đảm bảo không có dòng Wire.begin() nào khác trước dòng này
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Bắt đầu giao tiếp với MCP23X17 qua I2C
  // Nếu MCP23X17 có địa chỉ I2C khác 0x20 (địa chỉ mặc định), bạn cần truyền địa chỉ vào đây.
  // Ví dụ: mcp.begin_I2C(0x21); nếu địa chỉ là 0x21
  if (!mcp.begin_I2C()) {
    Serial.println("Error. Please check wiring, power, and I2C address (default is 0x20)!");
    while (1);
  }

  // configure pin for output
  mcp.pinMode(LED_PIN, OUTPUT);

  Serial.println("Looping...");
}

void loop() {
  mcp.digitalWrite(LED_PIN, HIGH);
  delay(500);
  mcp.digitalWrite(LED_PIN, LOW);
  delay(500);
}
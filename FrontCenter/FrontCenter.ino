#include <Adafruit_MCP2515.h>
#include <Servo.h>
#ifdef ESP8266
   #define CS_PIN    2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
   #define CS_PIN    14
#elif defined(TEENSYDUINO)
   #define CS_PIN    8
#elif defined(ARDUINO_STM32_FEATHER)
   #define CS_PIN    PC5
#elif defined(ARDUINO_NRF52832_FEATHER)
   #define CS_PIN    27
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define CS_PIN    P3_2
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
   #define CS_PIN    7
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN)
   #define CS_PIN    PIN_CAN_CS
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W) // PiCowbell CAN Bus
   #define CS_PIN    20
#else
   #define CS_PIN    5
#endif
#define CAN_BAUDRATE (250000)
Adafruit_MCP2515 mcp(CS_PIN);
Servo wiperservo;  // create servo object to control a servo
int pulseWidth;    // variable to store the servo position

void setup() {
  wiperservo.attach(26);  // attaches the servo on pin 9 to the servo object
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Sender test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void loop() {
  if mcp.parsePacket() {
    if (mcp.packetId() = 0x05) {
      while (mcp.available()) {
        message += (char)mcp.read();
      }

      if (message == "wiper") {
        for (pulseWidth = 1000; pulseWidth <= 2000; pulseWidth += 10) { // Increment by 10us for smoother motion
          wiperservo.writeMicroseconds(pulseWidth); // Set the pulse width directly
          delay(5); // Short delay to allow the servo to catch up
        }
        delay(500); // Wait for half a second at the end of the sweep

        for (int pulseWidth = 2000; pulseWidth >= 1000; pulseWidth -= 10) { // Decrement by 10us for smoother motion
          wiperservo.writeMicroseconds(pulseWidth); // Set the pulse width directly
          delay(5); // Short delay to allow the servo to catch up
        }
        delay(100);
    }
  }


}
//This is all of our include stuff
//Feel free to ignore this! It's not important for us
#include <Adafruit_MCP2515.h>
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
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
   #define CS_PIN    20
#else
   #define CS_PIN    5
#endif
#define CAN_BAUDRATE (250000)
Adafruit_MCP2515 mcp(CS_PIN);
//


//CODE BEGINS HERE
//Setup, also ignore!
void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Reciever test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");

  pinMode(runLight, INPUT);
  pinMode(brakeLight, INPUT);
  pinMode(hazardLight, INPUT);
  
}


const int runLight= 9;
const int brakeLight = 10;
const in hazardLight = 11;
string run;
string brake;
string hazard;
//Continuously loop and search for packets
void loop() {
  //If we find a packet.

}
  digitalWrite(runLight, HIGH);
   if(mcp.parsePacket()) {
    //And that packet has the ID we're looking for..
    if(mcp.packetId() == 0x00) {
      //Loop over every byte in the packet till the end
      while(mcp.available()) {
        //Then print the character in sequence!
        //Interpret chars
        message += (char)mcp.read();
        if (message == brake) {
          digitalWrite(brakeLight, HIGH);
        }
        if (message == hazard) {
          digitalWrite(hazardLight, HIGH);
        }
      }
      //Add a newline once the packet is finished
      Serial.println();
    }
  }

}
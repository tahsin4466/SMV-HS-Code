//This is all of our include stuff
//Feel free to ignore this! It's not important for us
#include <Adafruit_MCP2515.h>
#include <SPI.h>

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

//initialize pins
#define CS_LIS3 3
#define INT_LIS3 1
#define DRDY 10
#define runLight 6
#define brakeLight 5
#define hazardLight 9

//define settings
#define TEMP_ENABLE  0b1 //0 for disable, 1 for enable
#define XY_PERFORMANCE  0b10 //00 for low, 01 for medium, 10 for high, 11 for ultrahigh
#define SCALE  0b11 //00 for 4 gauss, 01 for 8, 10 for 12, 11 for 16
#define MODE  0b00 //00 for continuous, 01 for single, 10 or 11 for power down                                                                   
#define Z_PERFORMANCE  0b10 //00 for low, 01 for medium, 10 for high, 11 for ultrahigh
#define BIG_LITTLE_ENDIAN  0b1 //0 for little endian, 1 for big endian
#define INTERRUPT_CONFIG  0b1 //0 for disable, 1 for enable

//define register addresses
#define CTRL_REG1  0b00100000
#define CTRL_REG2  0b00100001
#define CTRL_REG3  0b00100010
#define CTRL_REG4  0b00100011
#define OUT_X_L  0b00101000
#define OUT_X_H  0b00101001
#define OUT_Y_L  0b00101010
#define OUT_Y_H  0b00101011
#define OUT_Z_L  0b00101100
#define OUT_Z_H  0b00101101
#define TEMP_OUT_L  0b00101110
#define TEMP_OUT_H  0b00101111
#define INT_CFG  0b00110000

#define DUMMY_BYTE 0x00

//define modes
#define READ_SINGLE 0b10000000
#define READ_MULTI 0b11000000
#define WRITE_SINGLE 0b00000000
#define WRITE_MULTI 0b01000000

uint8_t ctrl_reg1() {
  return (uint8_t)((TEMP_ENABLE << 8) | (XY_PERFORMANCE << 7) | 0b00010000);
}

uint8_t ctrl_reg2() {
  return (uint8_t) (SCALE << 7);
}

uint8_t ctrl_reg3() {
 return (uint8_t) MODE;
}

uint8_t ctrl_reg4() {
 return (uint8_t)(BIG_LITTLE_ENDIAN << 1) |(Z_PERFORMANCE << 2); 
}

uint8_t int_cfg() {
  return ((uint8_t) INTERRUPT_CONFIG) | 0b11101000;
}


double twosComplimentMag(int data) {
  double resolution;
  switch(SCALE) { //resolution depends on scale
    case 0b00:
      resolution = 6842.;
      break;
    case 0b01:
      resolution = 3421.;
      break;
    case 0b10:
      resolution = 2281.;
      break;
    case 0b11:
      resolution = 1711.;
      break;
  }

  if (data & 0b1000000000000000) { //if negative
    //Serial.println("before " + String(data));
    data -= 65536;
    //Serial.println("after " + String(data));
  }

  return data/resolution; //resolution given in LSB/gauss
}

double twosComplimentTemp(int data) {
  double resolution = 8;

  if (data & 0b1000000000000000) { //if negative
    //Serial.println("before " + String(data));
    data -= 65536;
    //Serial.println("after " + String(data));
  }

  return data/resolution; //resolution given in LSB/gauss
}

//CODE BEGINS HERE
//Setup, also ignore!
void setup() {
pinMode(CS_LIS3, OUTPUT);
  pinMode(INT_LIS3, OUTPUT);
  pinMode(DRDY, OUTPUT);
  
  Serial.begin(115200);
  digitalWrite(CS_LIS3, HIGH); //set chip selects to inactive
  SPI.begin(); //initialize SPI
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); //SPI settings

  //initialize LIS3
  digitalWrite(CS_LIS3, LOW);
  SPI.transfer(WRITE_MULTI | CTRL_REG1);
  SPI.transfer(ctrl_reg1());
  SPI.transfer(ctrl_reg2());
  SPI.transfer(ctrl_reg3());
  SPI.transfer(ctrl_reg4());
  digitalWrite(CS_LIS3, HIGH);
  digitalWrite(CS_LIS3, LOW);
  SPI.transfer(INT_CFG);
  SPI.transfer(int_cfg());
  digitalWrite(CS_LIS3, HIGH);


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



int previousMillis = 0;
int interval = 0;
int currentMillis = 0;
String run;
String brake;
String hazard;
String blinker;
String message;
//Continuously loop and search for packets
void loop() {
  //If we find a packet.

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
          if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
          }
          if (digitalRead(hazardLight) == LOW) {
          digitalWrite(hazardLight, HIGH);
          interval = 750;
          } else {
            digitalWrite(hazardLight, LOW);
            interval = 750;
          }
        }
          
         if (message == blinker) {
          if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
          }
          if (digitalRead(hazardLight) == LOW) {
          digitalWrite(hazardLight, HIGH);
          interval = 750;
          } else {
            digitalWrite(hazardLight, LOW);
            interval = 750;

          }
         }
      }
      //Add a newline once the packet is finished
      Serial.println();
    }

  }

  digitalWrite(CS_LIS3, LOW);
  SPI.transfer(READ_MULTI | OUT_X_L); //send a single-read command to OUT_X_L
  uint8_t x_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t x_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t y_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t y_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t z_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t z_LSB = SPI.transfer(DUMMY_BYTE);
  //uint8_t temp_MSB = SPI.transfer(DUMMY_BYTE);
  //uint8_t temp_LSB = SPI.transfer(DUMMY_BYTE);
  digitalWrite(CS_LIS3, HIGH);

  int x_raw = ((uint16_t) x_MSB << 8) | x_LSB; //combine both bytes into 1 value
  int y_raw = ((uint16_t) y_MSB << 8) | y_LSB;
  int z_raw = ((uint16_t) z_MSB << 8) | z_LSB;
  //int temp_raw = ((uint16_t) temp_MSB << 8) | temp_LSB;

  //Serial.println(String(x_MSB) + " " + String(x_LSB) + " " + String(x_raw));
  //Serial.println(String(y_MSB) + " " + String(y_LSB) + " " + String(y_raw));
  //Serial.println(String(z_MSB) + " " + String(z_LSB) + " " + String(z_raw));
  //Serial.println(String(temp_MSB) + " " + String(temp_LSB) + " " + String(temp_raw));
  //Serial.println();

  
  double x, y, z, temp;
  x = twosComplimentMag(x_raw); //convert raw data to human readable in gauss
  y = twosComplimentMag(y_raw);
  z = twosComplimentMag(z_raw);
  //temp = twosComplimentTemp(temp_raw);
  
  Serial.println("x: " + String(x) + "; y: " + String(y) + "; z: " + String(z));
  delay(500);

  //Serial.println(ctrl_reg1());
  //Serial.println(ctrl_reg2());
  //Serial.println(ctrl_reg3());
  //Serial.println(ctrl_reg4());
  //Serial.println(int_cfg());
    
}
  


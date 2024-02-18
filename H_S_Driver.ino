#include <SPI.h>

//initialize pins
#define CS_LIS3 12
#define INT_LIS3 10
#define DRDY 11

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

  
  
}

void loop() {
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

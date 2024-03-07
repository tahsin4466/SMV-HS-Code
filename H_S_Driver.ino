#include <SPI.h>

/*
 * H&S Board Driver Code 
 * 
 * UCLA Supermileage
 * 
 * Interfaces with the BMI323 and LIS3MLTR sensors through an Adafruit Feather RP2040 CAN
 * Outputs acceleration, gyroscope, and magnetometer data
 * Data output is given with respect to Global Top, Right, and Up (as defined by "Orientation of Sensors" Google Doc)
 * Before changing settings, make sure to consult datasheet
 * 
 * ONLY code that should be changed every time are BOARD_LOCATION, ROLL, and PITCH
 * Code that could be changed are the LIS3 and BMI SETTINGS (NOT REGISTERS)
 * 
 */

//IMPORTANT!!! MUST CHANGE TO MATCH BOARD
#define BOARD_LOCATION 0
//0 = Back Left/Front Left
//1 = Back Right
//2 = Front Right
//3 = Front Center

//EVEN MORE IMPORTANT!! MUST CHANGE TO MATCH BOARD
//UNITS IN RADIANS
#define ROLL 0.231
#define PITCH 0.647

//LIS3
//define settings
#define TEMP_ENABLE  0b1 //0 for disable, 1 for enable
#define XY_PERFORMANCE  0b10 //00 for low, 01 for medium, 10 for high, 11 for ultrahigh
#define SCALE  0b11 //00 for 4 gauss, 01 for 8, 10 for 12, 11 for 16
#define MODE  0b00 //00 for continuous, 01 for single, 10 or 11 for power down                                                                   
#define Z_PERFORMANCE  0b10 //00 for low, 01 for medium, 10 for high, 11 for ultrahigh
#define BIG_LITTLE_ENDIAN  0b1 //0 for little endian, 1 for big endian
#define INTERRUPT_CONFIG  0b1 //0 for disable, 1 for enable

//define register addresses
//DO NOT CHANGE
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

//BMI323
//define settings
#define ACC_ODR 0x5
#define ACC_SCALE 0x1
#define ACC_PERFORMANCE 0x7
#define GYRO_ODR 0x5
#define GYRO_SCALE 0x1
#define GYRO_PERFORMANCE 0x7

//define register addresses
//DO NOT CHANGE
#define ACC_CONF 0x20
#define GYRO_CONF 0x21
#define ACC_DATA_X 0x03
#define ACC_DATA_Y 0x04
#define ACC_DATA_Z 0x05
#define GYRO_DATA_X 0x06
#define GYRO_DATA_Y 0x07
#define GYRO_DATA_Z 0x08
#define TEMP_DATA 0x09

//pin definitions
//DO NOT CHANGE
#if BOARD_LOCATION == 0
  #define CS_LIS3 12
  #define CS_BMI 25
  #define INT_LIS3 10
  #define DRDY 11
#elif BOARD_LOCATION == 1
  #define CS_LIS3 29
  #define CS_BMI 25
  #define INT_LIS3 13
  #define DRDY 10
#elif BOARD_LOCATION == 2
  #define CS_LIS3 24
  #define CS_BMI 10
  #define INT_LIS3 26
  #define DRDY 29
#elif BOARD_LOCATION == 3
  #define CS_LIS3 12
  #define CS_BMI 25
  #define INT_LIS3 10
  #define DRDY 11
#endif

#define DUMMY_BYTE 0x00

//define modes for LIS3
//DO NOT CHANGE
#define LIS3_READ_SINGLE 0b10000000
#define LIS3_READ_MULTI 0b11000000
#define LIS3_WRITE_SINGLE 0b00000000
#define LIS3_WRITE_MULTI 0b01000000

//define modes for BMI
//DO NOT CHANGE
#define BMI_READ 0b10000000
#define BMI_WRITE 0b00000000

//functions to correctly shift bits of settings in respective registers
//DO NOT CHANGE
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

uint16_t acc_conf() {
  return (uint16_t) ACC_ODR | (uint16_t) (ACC_SCALE << 4) | (uint16_t) (ACC_PERFORMANCE << 12);
}

uint16_t gyro_conf() {
  return (uint16_t) GYRO_ODR | (uint16_t) (GYRO_SCALE << 4) | (uint16_t) (GYRO_PERFORMANCE << 12);
}


void setup() {
  pinMode(CS_LIS3, OUTPUT); //initialize pins
  pinMode(CS_BMI, OUTPUT);
  pinMode(INT_LIS3, OUTPUT);
  pinMode(DRDY, OUTPUT);
  
  Serial.begin(115200);
  digitalWrite(CS_LIS3, HIGH); //set chip selects to inactive
  digitalWrite(CS_BMI, HIGH);
  SPI.begin(); //initialize SPI
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); //SPI settings
  
  //initialize LIS3
  digitalWrite(CS_LIS3, LOW);
  SPI.transfer(LIS3_WRITE_MULTI | CTRL_REG1);
  SPI.transfer(ctrl_reg1());
  SPI.transfer(ctrl_reg2());
  SPI.transfer(ctrl_reg3());
  SPI.transfer(ctrl_reg4());
  digitalWrite(CS_LIS3, HIGH);
  digitalWrite(CS_LIS3, LOW);
  SPI.transfer(INT_CFG);
  SPI.transfer(int_cfg());
  digitalWrite(CS_LIS3, HIGH);
  SPI.endTransaction();

  //initialize BMI
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_BMI, LOW);
  SPI.transfer(BMI_WRITE | ACC_CONF);
  SPI.transfer(lowByte(acc_conf()));
  SPI.transfer(highByte(acc_conf()));
  SPI.transfer(lowByte(gyro_conf()));
  SPI.transfer(highByte(gyro_conf()));
  digitalWrite(CS_BMI, HIGH);
  SPI.endTransaction();
}

void loop() {
  //getting mag data from LIS3
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_LIS3, LOW);
  SPI.transfer(LIS3_READ_MULTI | OUT_X_L); //send a multi-read command to OUT_X_L
  uint8_t x_mag_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t x_mag_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t y_mag_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t y_mag_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t z_mag_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t z_mag_LSB = SPI.transfer(DUMMY_BYTE);
  //uint8_t temp_mag_MSB = SPI.transfer(DUMMY_BYTE);
  //uint8_t temp_mag_LSB = SPI.transfer(DUMMY_BYTE);
  digitalWrite(CS_LIS3, HIGH);
  SPI.endTransaction(); //NOTE: remember to end transaction after de-asserting CS

  //getting acc and gyro data from BMI
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_BMI, LOW);
  SPI.transfer(BMI_READ | ACC_DATA_X);
  uint8_t dummy = SPI.transfer(DUMMY_BYTE); // dummy read must be discarded
  uint8_t x_acc_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t x_acc_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t y_acc_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t y_acc_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t z_acc_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t z_acc_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t x_gyro_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t x_gyro_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t y_gyro_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t y_gyro_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t z_gyro_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t z_gyro_MSB = SPI.transfer(DUMMY_BYTE);
  uint8_t temp_acc_gyro_LSB = SPI.transfer(DUMMY_BYTE);
  uint8_t temp_acc_gyro_MSB = SPI.transfer(DUMMY_BYTE);
  digitalWrite(CS_BMI, HIGH);
  SPI.endTransaction();

  //adding LSB and MSB to make 16 bit int
  int x_mag_raw = ((uint16_t) x_mag_MSB << 8) | x_mag_LSB; //combine both bytes into 1 value
  int y_mag_raw = ((uint16_t) y_mag_MSB << 8) | y_mag_LSB;
  int z_mag_raw = ((uint16_t) z_mag_MSB << 8) | z_mag_LSB;
  //int temp_mag_raw = ((uint16_t) temp_mag_MSB << 8) | temp_mag_LSB;
  int x_acc_raw = ((uint16_t) x_acc_MSB << 8) | x_acc_LSB;
  int y_acc_raw = ((uint16_t) y_acc_MSB << 8) | y_acc_LSB;
  int z_acc_raw = ((uint16_t) z_acc_MSB << 8) | z_acc_LSB;
  int x_gyro_raw = ((uint16_t) x_gyro_MSB << 8) | x_gyro_LSB;
  int y_gyro_raw = ((uint16_t) y_gyro_MSB << 8) | y_gyro_LSB;
  int z_gyro_raw = ((uint16_t) z_gyro_MSB << 8) | z_gyro_LSB;
  int temp_acc_gyro_raw = ((uint16_t) temp_acc_gyro_MSB << 8) | temp_acc_gyro_LSB;

  //defining final variables
  double x_mag, y_mag, z_mag, temp_mag, x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro, temp_acc_gyro;

  //convert raw mag data to human readable in gauss
  x_mag = twosComplimentMag(x_mag_raw);
  y_mag = twosComplimentMag(y_mag_raw);
  z_mag = twosComplimentMag(z_mag_raw);

  //convert raw acc data to human readable in m/s^2
  x_acc = twosComplimentAcc(x_acc_raw);
  y_acc = twosComplimentAcc(y_acc_raw);
  z_acc = twosComplimentAcc(z_acc_raw);

  //convert raw gyro data to human readable in degree/s
  x_gyro = twosComplimentGyro(x_gyro_raw);
  y_gyro = twosComplimentGyro(y_gyro_raw);
  z_gyro = twosComplimentGyro(z_gyro_raw);

  //convert raw temp data to human readable in Fahrenheit
  temp_acc_gyro = twosComplimentTempAccGyro(temp_acc_gyro_raw);

  //changing x, y, z to top, right, up coordinate system
  
  double front_mag, right_mag, up_mag, front_acc, right_acc, up_acc, front_gyro, right_gyro, up_gyro;

  //z is always up
  up_mag = z_mag;
  up_acc = z_acc;
  up_gyro = z_gyro;
  
  //change x and y to top and right based on board location
  switch(BOARD_LOCATION) {
    case 0:
      front_mag = y_mag;
      front_acc = y_acc;
      front_gyro = y_gyro;

      right_mag = x_mag;
      right_acc = x_acc;
      right_gyro = x_gyro;
      
      break;
    case 1:
      front_mag = -x_mag;
      front_acc = -x_acc;
      front_gyro = -x_gyro;

      right_mag = y_mag;
      right_acc = y_acc;
      right_gyro = y_gyro;
      
      break;
    case 2:
      front_mag = x_mag;
      front_acc = x_acc;
      front_gyro = x_gyro;

      right_mag = -y_mag;
      right_acc = -y_acc;
      right_gyro = -y_gyro;
      
      break;
    case 3:
      front_mag = -y_mag;
      front_acc = x_acc;
      front_gyro = x_gyro;

      right_mag = -x_mag;
      right_acc = -y_acc;
      right_gyro = -y_gyro;
      break;
  }

  double global_front_acc, global_right_acc, global_up_acc, global_front_gyro, global_right_gyro, global_up_gyro, global_front_mag, global_right_mag, global_up_mag;

  global_front_acc = toGlobalFront(front_acc, right_acc, up_acc);
  global_front_gyro = toGlobalFront(front_gyro, right_gyro, up_gyro);
  global_front_mag = toGlobalFront(front_mag, right_mag, up_mag);

  global_right_acc = toGlobalRight(front_acc, right_acc, up_acc);
  global_right_gyro = toGlobalRight(front_gyro, right_gyro, up_gyro);
  global_right_mag = toGlobalRight(front_mag, right_mag, up_mag);

  global_up_acc = toGlobalUp(front_acc, right_acc, up_acc);
  global_up_gyro = toGlobalUp(front_gyro, right_gyro, up_gyro);
  global_up_mag = toGlobalUp(front_mag, right_mag, up_mag);
  

  //xyz coordinate print
  //Serial.println("x_mag: " + String(x_mag) + "; y_mag: " + String(y_mag) + "; z_mag: " + String(z_mag));
  //Serial.println("x_acc: " + String(x_acc) + "; y_acc: " + String(y_acc) + "; z_acc: " + String(z_acc));
  //Serial.println("x_gyro: " + String(x_gyro) + "; y_gyro: " + String(y_gyro) + "; z_gyro: " + String(z_gyro));
  //Serial.println("temp: " + String(temp_acc_gyro));

  //board's top right up coordinate print
  //Serial.println("top_mag: " + String(front_mag) + "; right_mag: " + String(right_mag) + "; up_mag: " + String(up_mag));
  //Serial.println("top_acc: " + String(front_acc) + "; right_acc: " + String(right_acc) + "; up_acc: " + String(up_acc));
  //Serial.println("top_gyro: " + String(front_gyro) + "; right_gyro: " + String(right_gyro) + "; up_gyro: " + String(up_gyro));
  //Serial.println("temp: " + String(temp_acc_gyro));

  //global top right up coordinate print
  //Serial.println("global_front_mag: " + String(global_front_mag) + "; global_right_mag: " + String(global_right_mag) + "; global_up_mag: " + String(global_up_mag));
  Serial.println("global_front_acc: " + String(global_front_acc) + "; global_right_acc: " + String(global_right_acc) + "; global_up_acc: " + String(global_up_acc));
  //Serial.println("global_front_gyro: " + String(global_front_gyro) + "; global_right_gyro: " + String(global_right_gyro) + "; global_up_gyro: " + String(global_up_gyro));
  //Serial.println("temp: " + String(temp_acc_gyro));
  
  
  delay(100); 
}


//twos compliment functions to convert data into human-readable
//DO NOT CHANGE

//twos compliment for mag data
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

//twos compliment for mag data
double twosComplimentAcc(int data) {
  double resolution;
  switch(ACC_SCALE) { //resolution depends on scale
    case 0b000:
      resolution = 16.38;
      break;
    case 0b01:
      resolution = 8.19;
      break;
    case 0b10:
      resolution = 4.10;
      break;
    case 0b11:
      resolution = 2.05;
      break;
  }

  if (data & 0b1000000000000000) { //if negative
    //Serial.println("before " + String(data));
    data -= 65536;
    //Serial.println("after " + String(data));
  }

  return (data/resolution)/ 1000 * 9.81; //resolution given in LSB/mg
}

//twos compliment for gyro data
double twosComplimentGyro(int data) {
  double resolution;
  switch(GYRO_SCALE) { //resolution depends on scale
    case 0b000:
      resolution = 262.144;
      break;
    case 0b01:
      resolution = 131.2;
      break;
    case 0b10:
      resolution = 65.6;
      break;
    case 0b11:
      resolution = 32.8;
      break;
    case 0b100:
      resolution = 16.4;
      break;
  }

  if (data & 0b1000000000000000) { //if negative
    //Serial.println("before " + String(data));
    data -= 65536;
    //Serial.println("after " + String(data));
  }

  return data/resolution; //resolution given in LSB/(degrees/s)
}

//twos compliment for mag temp data
double twosComplimentTempMag(int data) {
  double resolution = 8;

  if (data & 0b1000000000000000) { //if negative
    //Serial.println("before " + String(data));
    data -= 65536;
    //Serial.println("after " + String(data));
  }

  return data/resolution; //resolution given in LSB/gauss
}

//twos compliment for acc/gyro temp data
double twosComplimentTempAccGyro(int data) {
  return data/512. + 23;
}

double toGlobalFront(double front, double right, double up) {
  return front*cos(PITCH) - up*sin(PITCH);
}
double toGlobalRight(double front, double right, double up) {
  return front*sin(ROLL)*sin(PITCH) + right*cos(ROLL) + up*sin(ROLL)*cos(PITCH);
}
double toGlobalUp(double front, double right, double up) {
  return front*cos(ROLL)*sin(PITCH) - right*sin(ROLL) + up*cos(PITCH)*cos(ROLL);
}

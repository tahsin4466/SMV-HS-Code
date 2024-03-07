#include <SPI.h>

/*
 * H&S Orientation Testing
 * 
 * Takes samples of accelerometer
 * 
 * Outputs roll and pitch
 * Positive roll = CCW (when looking back to front)
 * Negative roll = CS
 * 
 * Positive pitch = pitch forward
 * Negative pitch = pitch backward
 * 
 * ONLY code that should be changed is BOARD_LOCATION variable
 * 
 */

//IMPORTANT!!! MUST CHANGE TO MATCH BOARD
#define BOARD_LOCATION 0
//0 = Back Left/Front Left
//1 = Back Right
//2 = Front Right
//3 = Front Center

//pin definitions
//DO NOT CHANGE
#if BOARD_LOCATION == 0
  #define CS_BMI 25
#elif BOARD_LOCATION == 1
  #define CS_BMI 25
#elif BOARD_LOCATION == 2
  #define CS_BMI 10
#elif BOARD_LOCATION == 3
  #define CS_BMI 25
#endif

//BMI323
//define settings
//DO NOT CHANGE
#define ACC_ODR 0x5
#define ACC_SCALE 0x0
#define ACC_PERFORMANCE 0x7

//define register addresses
//DO NOT CHANGE
#define ACC_CONF 0x20
#define ACC_DATA_X 0x03
#define ACC_DATA_Y 0x04
#define ACC_DATA_Z 0x05

#define DUMMY_BYTE 0x00

//define modes for BMI
//DO NOT CHANGE
#define BMI_READ 0b10000000
#define BMI_WRITE 0b00000000

double front_sum = 0;
double right_sum = 0;
double up_sum = 0;
int i = 1;

//functions to correctly shift bits of settings in respective registers
//DO NOT CHANGE
uint16_t acc_conf() {
  return (uint16_t) ACC_ODR | (uint16_t) (ACC_SCALE << 4) | (uint16_t) (ACC_PERFORMANCE << 12);
}

void setup() { //entire setup copied from H_S_Driver so data can be taken from accelerometer
  pinMode(CS_BMI, OUTPUT);
  
  Serial.begin(115200);
  digitalWrite(CS_BMI, HIGH);
  SPI.begin(); //initialize SPI
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); //SPI settings

  //initialize BMI
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_BMI, LOW);
  SPI.transfer(BMI_WRITE | ACC_CONF);
  SPI.transfer(lowByte(acc_conf()));
  SPI.transfer(highByte(acc_conf()));
  digitalWrite(CS_BMI, HIGH);
  SPI.endTransaction();
}

void loop() {
  //copied from H_S_Driver
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
  digitalWrite(CS_BMI, HIGH);
  SPI.endTransaction();

  //adding LSB and MSB to make 16 bit int
  int x_acc_raw = ((uint16_t) x_acc_MSB << 8) | x_acc_LSB;
  int y_acc_raw = ((uint16_t) y_acc_MSB << 8) | y_acc_LSB;
  int z_acc_raw = ((uint16_t) z_acc_MSB << 8) | z_acc_LSB;

  //defining final variables
  double x_acc, y_acc, z_acc;

  //convert raw acc data to human readable in m/s^2
  x_acc = twosComplimentAcc(x_acc_raw);
  y_acc = twosComplimentAcc(y_acc_raw);
  z_acc = twosComplimentAcc(z_acc_raw);

  //changing x, y, z to top, right, up coordinate system
  double front_acc, right_acc, up_acc;

  //z is always up
  up_acc = z_acc;
  
  //change x and y to top and right based on board location
  switch(BOARD_LOCATION) {
    case 0:
      front_acc = y_acc;
      right_acc = x_acc;
      break;
    case 1:
      front_acc = -x_acc;
      right_acc = y_acc;
      break;
    case 2:
      front_acc = x_acc;
      right_acc = -y_acc;
      break;
    case 3:
      front_acc = x_acc;
      right_acc = -y_acc;
      break;
  }

  
  //end copy from H_S_Driver
  //averaging samples
  front_sum += front_acc;
  right_sum += right_acc;
  up_sum += up_acc;

  double front_avg = front_sum/i;
  double right_avg = right_sum/i;
  double up_avg = up_sum/i;


  //IN RADIANS
  double roll = -atan(right_avg/up_avg);
  double pitch = atan(front_avg/sqrt(pow(right_avg,2) + pow(up_avg,2)));

  //IN DEGREES
  double roll_degree = roll * 180/3.1415926;
  double pitch_degree = pitch * 180/3.1415926;

  /*
  Serial.println("i: " + String(i));
  Serial.println("front_acc: " + String(front_acc));
  Serial.println("right_acc: " + String(right_acc));
  Serial.println("up_acc: " + String(up_acc));
  Serial.println("front_sum: " + String(front_sum));
  Serial.println("right_sum: " + String(right_sum));
  Serial.println("up_sum: " + String(up_sum));
  Serial.println("front_avg: " + String(front_avg));
  Serial.println("right_avg: " + String(right_avg));
  Serial.println("up_avg: " + String(up_avg));
  */
  
  if (i%100 == 0) {
    Serial.print("Roll (Radians): ");
    Serial.println(roll, 3);
    Serial.print("Pitch (Radians): ");
    Serial.println(pitch, 3);
  
    Serial.print("Roll (Degrees): ");
    Serial.println(roll_degree, 6);
    Serial.print("Pitch (Degrees): ");
    Serial.println(pitch_degree, 6);
  }
  
  i++;
  delay(10); 
}


//twos compliment functions to convert data into human-readable
//DO NOT CHANGE

//twos compliment for acc data
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

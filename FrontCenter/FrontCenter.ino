#include "SMVcanbus.h"
#include <Servo.h>
#define CS_LIS3 12
#define CS_BMI 25
#define INT_LIS3 10
#define DRDY 11
#define BOARD HS_FC
#define MOSFET_1 6
#define PWM_1 26
//GPIO 6

CANBUS can(HS);
Servo wiperservo;
bool wiperToggle = false;
bool hornToggle = false;
unsigned long previousMillis = 0; // will store last time the servo was updated
const long interval = 5; // interval at which to move servo (milliseconds)
int pulseWidth = 1000; // Servo position
bool increasing = true; // Direction of servo movement

void setup() {
  wiperservo.attach(PWM_1); // attaches the servo on pin 26 to the servo object
  pinMode(MOSFET_1, OUTPUT);
  Serial.begin(115200);
  can.begin();
}

void loop() {
  can.looper();
  if (can.isThere()) {
    if (strcmp(can.getDataType(), "Wipers") == 0) {
      double data = can.getData();
      if (data == 1) {
        wiperToggle = true;
      } 
      else if (data == 0) {
        wiperToggle = false;
      }
    }
    else if (strcmp(can.getDataType(), "Horn") == 0) {
      double data = can.getData();
      if (data == 1) {
        hornToggle = true;
      }
      else if (data == 0) {
        hornToggle = false;
      }
    }
  }

  unsigned long currentMillis = millis();

  //Horn check
  if (hornToggle) {
    digitalWrite(MOSFET_1, HIGH);
  }
  else {
    digitalWrite(MOSFET_1, LOW);
  }
  // Check if it's time to move the servo
  if (wiperToggle && currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // save the last time you moved the servo
    // Move Servo
    if (increasing) {
      pulseWidth += 10;
      if (pulseWidth >= 2000) {
        increasing = false; // Change direction
        delay(500); // Wait at the end of sweep
      }
    } else {
      pulseWidth -= 10;
      if (pulseWidth <= 1000) {
        increasing = true; // Change direction
        delay(100); // Short delay at the end of the return sweep
      }
    }
    wiperservo.writeMicroseconds(pulseWidth);
  }
}

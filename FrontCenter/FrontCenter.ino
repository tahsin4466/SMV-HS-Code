#include "SMVcanbus.h"
#include <Servo.h>

CANBUS can(Bear_1);
Servo wiperservo;
bool wiperToggle = false;
unsigned long previousMillis = 0; // will store last time the servo was updated
const long interval = 5; // interval at which to move servo (milliseconds)
int pulseWidth = 1000; // Servo position
bool increasing = true; // Direction of servo movement

void setup() {
  wiperservo.attach(26); // attaches the servo on pin 26 to the servo object
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
      } else if (data == 0) {
        wiperToggle = false;
      }
    }
  }

  unsigned long currentMillis = millis();

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

#include "SMVcanbus.h"
#define CS_LIS3 12
#define CS_BMI 25
#define INT_LIS3 10
#define DRDY 11
#define BOARD HS_FR
#define MOSFET_1 4
#define MOSFET_2 26

CANBUS can(HS);
bool headlightToggle = false;
bool blinkerToggle = false;
bool blinkerState = false;
unsigned long blinkerLastToggled = 0;

// Variables to keep track of the current state to avoid unnecessary toggles
bool currentHeadlightState = false;
bool currentBlinkerState = false;

bool setToggle(double data) {
  return data == 1;
}

//Function to toggle blinker on and off asynchronously
void toggleBlinkerState() {
  if ((millis() - blinkerLastToggled) > 600) {
    blinkerState = !blinkerState;
    blinkerLastToggled = millis();
  }
}

void setup() {
  pinMode(MOSFET_1, OUTPUT);
  pinMode(MOSFET_2, OUTPUT);
  Serial.begin(115200);
  can.begin();
}

void loop() {
  //CAN message checker
  can.looper();
  if (can.isThere()) {
    char* device = can.getDataType();
    double data = can.getData();
    if (strcmp(device, "Headlights") == 0) {
      headlightToggle = setToggle(data);
    }
    else if (strcmp(device, "Blink_Left") == 0) { 
      blinkerToggle = setToggle(data);
    }
  }

  //Headlight toggler - only update if state has changed
  if (headlightToggle != currentHeadlightState) {
    digitalWrite(MOSFET_1, headlightToggle ? HIGH : LOW);
    currentHeadlightState = headlightToggle;
  }

  //Blinker toggler - only update if state has changed
  if (blinkerToggle) {
    toggleBlinkerState(); 
    if (blinkerState != currentBlinkerState) {
      digitalWrite(MOSFET_2, blinkerState ? HIGH : LOW);
      currentBlinkerState = blinkerState;
    }
  }
  else if (currentBlinkerState) { // If blinker is currently on but should be off
    digitalWrite(MOSFET_2, LOW);
    currentBlinkerState = false;
  }
}

#include "SMVcanbus.h"
#define CS_LIS3 24
#define CS_BMI 10
#define INT_LIS3 26
#define DRDY 29
#define BOARD HS_BR
#define MOSFET_1 6
#define MOSFET_2 5
#define MOSFET_3 9

CANBUS can(HS);
bool taillightToggle = false;
bool brakelightToggle = false;
bool blinkerToggle = false;
bool blinkerState = false;
unsigned long blinkerLastToggled = 0;

// Variables to keep track of the current state to avoid unnecessary toggles
bool currentTaillightState = false;
bool currentBrakelightState = false;
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
  pinMode(MOSFET_3, OUTPUT);
  Serial.begin(115200);
  can.begin();
}

void loop() {
  //CAN message checker
  can.looper();
  if (can.isThere()) {
    char* device = can.getDataType();
    double data = can.getData();
    Serial.println(device);
    Serial.println(data);
    if (strcmp(device, "Headlights") == 0) {
      taillightToggle = setToggle(data);
    }
    else if (strcmp(device, "Brake") == 0) {
      brakelightToggle = setToggle(data);
    }
    else if (strcmp(device, "Blink_Right") == 0) { 
      blinkerToggle = setToggle(data);
    }
    else if (strcmp(device, "Hazard") == 0) { 
      blinkerToggle = setToggle(data);
    }
  }

  //Taillight toggler - only update if state has changed
  if (taillightToggle != currentTaillightState) {
    Serial.println("Toggled Taillight");
    digitalWrite(MOSFET_1, taillightToggle ? HIGH : LOW);
    currentTaillightState = taillightToggle;
  }

  //Brakelight toggler
  if (brakelightToggle != currentBrakelightState) {
    Serial.println("Toggled Brakelight");
    digitalWrite(MOSFET_3, brakelightToggle ? HIGH : LOW);
    currentBrakelightState = brakelightToggle;
  }

  

  //Blinker toggler - only update if state has changed
  if (blinkerToggle) {
    Serial.println("Toggled blinker");
    toggleBlinkerState(); 
    if (blinkerState != currentBlinkerState) {
      digitalWrite(MOSFET_2, blinkerState ? HIGH : LOW);
      currentBlinkerState = blinkerState;
    }
  }
  else if (currentBlinkerState) { // If blinker is currently on but should be off
    Serial.println("Turned off blinker");
    digitalWrite(MOSFET_2, LOW);
    currentBlinkerState = false;
  }
  delay(20);
}

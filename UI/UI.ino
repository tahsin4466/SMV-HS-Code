#include "SMVcanbus.h"
#include <string.h>

CANBUS can(UI);
const double ON = 1
const double OFF = 0

const int reverseSwitchPin = 26;
const int headlightSwitchPin = 27;
const int wiperSwitchPin = 28;
const int hazardSwitchPin = 29;
const int spareSwitchPin = 24;

const int leftSwitchPin = 9;
const int rightSwitchPin = 6;

const int hornButtonPin = 10;
const int daqButtonPin = 11;
const int spareButtonPin = 12;

void setup() {
  Serial.begin(115200);
  can.begin();
  delay(400);
}

void loop() {
  //NOTE
  //You MUST only send signals on change, not every clock cycle! Please modify!
  reverseState = digitalRead(reverseSwitchPin);
  headlightState = digitalRead(headlightSwitchPin);
  wiperState = digitalRead(wiperSwitchpin);
  hazardState = digitalRead(hazardSwitchPin);
  
  hornState = digitalRead(hornButtonPin);

  leftState = digitalRead(leftSwitchPin);
  rightState = digitalRead(rightSwitchPin);

  //Back running light is always on when powered on
  can.send(ON, Back_Running_Light)

  //Check reverse state
  if (reverseState == HIGH) {
    can.send(ON, Reverse);
  }
  else {
    can.send(OFF, Reverse);
  }

  //Check headlight state
  if (headlightState == HIGH) {
    can.send(ON, Headlight);
  }
  else {
    can.send(OFF, Reverse);
  }

  //Check wiper state
  if (wiperSate == HIGH) {
    can.send(ON, Wiper);
  }
  else {
    can.send(OFF, Wiper);
  }

  //Check hazard sate
  if (hazardState == HIGH) {
    can.send(ON, Hazard);
  }
  else {
    can.send(OFF, Hazard);
    //Check indicator states only if hazard is off
    //Chech left indicator state
    if (leftState == HIGH) {
      can.send(ON, Left_Blinker);
    }
    else {
      can.send(OFF, Left_Blinker);
    }
    //Check right indicator state
    if (rightState == HIGH) {
      can.send(ON, Right_Blinker);
    }
    else {
      can.send(OFF, Right_Blinker)
    }
  }
};

enum devices {
  Bear_1,
  Bear_2,
  UI,
  HS,
  DAQ,
}

enum motorMessage {
    RPM,
    Motor_State,
    Cruise,
    M_Error_Status,
    Throttle,
    Brake,
    Meter_Count
};

enum UIMessage {
  Left_Blinker,
  Right_Blinker,
  Headlight,
  Back_Running_Light,
  Horn,
  Reverse,
  Wiper,
  UI_Error_Status
}

enum HSMessage {
  Gryo,
  Accel,
  Mag,
  Temp
  HS_Error_Status
}

enum DAQMessage {
    Longitude,
    Latitude,
    Speed
    D_Error_Status
};

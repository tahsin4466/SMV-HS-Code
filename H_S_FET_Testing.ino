#define BOARD 0
//0 = back left/front left
//1 = back right
//2 = front right
//3 = front center

#if BOARD == 0
  #define MOSFET1 4
  #define MOSFET2 26
  #define MOSFET3 1
#elif BOARD == 1
  #define MOSFET1 9
  #define MOSFET2 5
  #define MOSFET3 6
#elif BOARD == 2
  #define MOSFET1 10
  #define MOSFET2 9
#elif BOARD == 3
  #define MOSFET1 6
#endif

void setup() {
  pinMode(MOSFET1, OUTPUT);
  if (BOARD != 3) {
    pinMode(MOSFET2, OUTPUT);
  }
  if (BOARD == 0 || BOARD == 1) {
    pinMode(MOSFET3, OUTPUT);
  }

  digitalWrite(MOSFET1, HIGH);
  if (BOARD != 3) {
    digitalWrite(MOSFET2, HIGH);
  }
  if (BOARD == 0 || BOARD == 1) {
    digitalWrite(MOSFET3, HIGH);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}

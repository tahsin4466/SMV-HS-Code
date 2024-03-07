#define ROLL -3.1415926/3
#define PITCH 0

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  double front = 0;
  double right = 8.496;
  double up = 9.81/2;

  double global_front = toGlobalFront(front, right, up);
  double global_right = toGlobalRight(front, right, up);
  double global_up = toGlobalUp(front, right, up);

  Serial.println("F: " + String(global_front));
  Serial.println("R: " + String(global_right));
  Serial.println("U: " + String(global_up));
  
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

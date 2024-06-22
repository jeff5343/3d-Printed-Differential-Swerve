#include <PID_v1.h>

const int enA = 9;
const int in1 = 4;
const int in2 = 5;
const int c1a = 2;
const int c2a = 3;

const int enB = 10;
const int in3 = 6;
const int in4 = 7;

double percentOuts[] = { 0, 0 };
bool newData = false;
int dataIndex = 0;
String inString = "";

bool pidMode = false;

volatile int ticksA = 0;
volatile double rotationA = 0;
volatile bool updatedA = false;

double rpsA = 0;
unsigned long lastMicrosA;
double lastRotationA = 0;

// PID
double Setpoint, Input, Output;
double Kp = 1, Ki = 0, Kd = 0;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// feedforward
double Ks = 12, Kv = 1.5;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(c1a, INPUT);
  pinMode(c2a, INPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(c1a), updateEncoderA, CHANGE);

  lastMicrosA = micros();

  Input = rpsA;
  Setpoint = 0;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-40, 40);

  Serial.begin(9600);
  Serial.println("<Arduino program starting...>");
}

void loop() {
  readPercentOut();
  if (newData && !pidMode) {
    Serial.println(String(percentOuts[0], 1) + ", " + String(percentOuts[1], 1));
    setA(percentOuts[0]);
    setB(percentOuts[1]);
    newData = false;
  }
  // Serial.println(String(digitalRead(c1a)) + ", " + String(digitalRead(c2a)));

  updateEncoderAVelocity();

  double output = constrain(Output + (Ks + (Kv * Setpoint)), -100, 100);
  if (newData && pidMode) {
    Setpoint = percentOuts[0];
    newData = false;
  }
  if(pidMode) {
    Input = rpsA;
    pid.Compute();
    setA(output);
  }

  // Serial.print(String(rotationA) + ", ");
  Serial.print(String(rpsA) + "r/s , ");
  Serial.print(String(Setpoint) + "r/s , ");
  Serial.println(output);
}

void setA(double percentOut) {
  if (percentOut >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  analogWrite(enA, map(abs(percentOut), 0, 100, 0, 255));
}

void setB(double percentOut) {
  if (percentOut >= 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }

  analogWrite(enB, map(abs(percentOut), 0, 100, 0, 255));
}

double readPercentOut() {
  while (Serial.available() && newData == false) {
    int inChar = Serial.read();

    if (inChar == '!') {
      pidMode = !pidMode;
      break;
    }

    if (isDigit(inChar) || inChar == '.' || inChar == '-') {
      inString += (char)inChar;
    }

    if ((inChar == ',' || inChar == '\n') && dataIndex <= 1) {
      percentOuts[dataIndex] = constrain(inString.toDouble(), -100, 100);
      inString = "";
      dataIndex++;
    }

    if (inChar == '\n') {
      newData = true;
      dataIndex = 0;
    }
  }
}

// called every loop
void updateEncoderAVelocity() {
  unsigned long currMicros = micros();
  if (currMicros > lastMicrosA) {
    double secondsDiff = (currMicros - lastMicrosA) / 1000000.0;
    if (updatedA || secondsDiff > 0.1) {
      double rotationDiff = rotationA - lastRotationA;
      rpsA = rotationDiff / secondsDiff;
      lastRotationA = rotationA;
      updatedA = false;
      lastMicrosA = currMicros;
    }
  } else {
    lastMicrosA = currMicros;
  }
}

void updateEncoderA() {
  int c1aVal = digitalRead(c1a);
  if (c1aVal == digitalRead(c2a)) {
    // CCW
    ticksA += 1;
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    // CW
    ticksA -= 1;
    digitalWrite(LED_BUILTIN, LOW);
  }

  rotationA = ticksA / 100.0;
  updatedA = true;
}
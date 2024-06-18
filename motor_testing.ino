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

volatile int ticksA = 0;
volatile int c1aPrevVal = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(c1a, INPUT);
  pinMode(c2a, INPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(c1a), updateEncoderA, CHANGE);

  c1aPrevVal = digitalRead(c1a);

  Serial.begin(9600);
  Serial.println("<Arduino program starting...>");
}

void loop() {
  readPercentOut();
  if (newData) {
    Serial.println(String(percentOuts[0], 1) + ", " + String(percentOuts[1], 1));
    setA(percentOuts[0]);
    setB(percentOuts[1]);
    newData = false;
  }
  // Serial.println(String(digitalRead(c1a)) + ", " + String(digitalRead(c2a)));
  Serial.println(ticksA / 100.0);
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

void updateEncoderA() {
  int c1aVal = digitalRead(c1a);
  if (c1aVal == 1 && c1aVal != c1aPrevVal) {
    if (c1aVal == digitalRead(c2a)) {
      // CCW
      ticksA += 1;
    } else {
      // CW
      ticksA -= 1;
    }
  }
  c1aPrevVal = c1aVal;
}
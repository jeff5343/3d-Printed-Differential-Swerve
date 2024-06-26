#include <PID_v1.h>

const double tickToRotationRatio = 1.0 / 4.0;
const double motorGearRatio = 1.0 / 100.0;
const double driveGearRatio = 17.0 / 60.0;
const double moduleRotationPerTick = tickToRotationRatio * motorGearRatio * driveGearRatio / 2.0;
double moduleRotation = 0;

bool newData = false;
int dataIndex = 0;
String inString = "";

bool pidMode = false;

struct Motor {
  // pins
  const int en, in1, in2, c1, c2;
  // const
  const bool reversePositive;
  // input/output
  double input;
  double output;
  // data
  volatile bool isUpdated;
  volatile int ticks;
  volatile double rotation, rps;
  unsigned long lastRpsUpdateMicros;
  double lastRotation;
  // pid
  double pidSetpoint, pidInput, pidOutput;
  const double kP, kI, kD, kS, kV;
  PID pid;
};

Motor motorA = Motor{
  // pins
  9, 4, 5, 2, 11, true,
  // input/output
  0, 0,
  // data
  false, 0, 0, 0, 0, 0,
  // pid
  0, 0, 0,
  5, 0, 0,                           // kP, kI, kD
  5, 7,                              // Ks, Kv
  PID(NULL, NULL, NULL, 0, 0, 0, 0)  // PID created later
};

Motor motorB = Motor{
  // pins
  10, 6, 7, 3, 12, false,
  //input
  0, 0,
  // data
  false, 0, 0, 0, 0, 0,
  // pid
  0, 0, 0,
  5, 0, 0,  // kP, kI, kD
  5, 7,     // Ks, Kv
  PID(NULL, NULL, NULL, 0, 0, 0, 0)
};

Motor *motors[2] = { &motorA, &motorB };

// module rotation pid
double rotationPidInput = 0, rotationPidOutput = 0, rotationPidSetpoint = 0;
double kP=30, kI=0, kD=0.5, kS=12;
PID rotationPid(&rotationPidInput, &rotationPidOutput, &rotationPidSetpoint, kP, kI, kD, DIRECT);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < 2; i++) {
    pinMode(motors[i]->en, OUTPUT);
    pinMode(motors[i]->in1, OUTPUT);
    pinMode(motors[i]->in2, OUTPUT);
    pinMode(motors[i]->c1, INPUT);
    pinMode(motors[i]->c2, INPUT);
    attachInterrupt(digitalPinToInterrupt(motors[i]->c1), i == 0 ? updateMotorARotation : updateMotorBRotation, CHANGE);

    motors[i]->lastRpsUpdateMicros = micros();
    motors[i]->pid = PID(
      &motors[i]->pidInput, &motors[i]->pidOutput, &motors[i]->pidSetpoint,
      motors[i]->kP, motors[i]->kI, motors[i]->kD, DIRECT);
    motors[i]->pid.SetMode(AUTOMATIC);
    motors[i]->pid.SetOutputLimits(-100, 100);
  }

  rotationPid.SetMode(AUTOMATIC);
  rotationPid.SetOutputLimits(-100, 100);

  Serial.begin(9600);
  Serial.println("<Arduino program starting...>");
}

void loop() {
  readMotorInputs();
  if (newData && !pidMode) {
    motorA.output = motorA.input;
    motorB.output = motorB.input;
    newData = false;
  }
  // Serial.println(String(digitalRead(motorA.c1)) + ", " + String(digitalRead(motorA.c2)));
  // Serial.println(String(motorA.c1) + ", " + String(motorA.c2));

  updateMotorVelocities();

  if (newData && pidMode) {
    // motorA.pidSetpoint = motorA.input;
    // motorB.pidSetpoint = motorB.input;

    rotationPidSetpoint = motorA.input;

    newData = false;
  }
  if (pidMode) {
    // for (int i = 0; i < 2; i++) {
    //   motors[i]->pidInput = motors[i]->rps;
    //   motors[i]->pid.Compute();
    //   int setpointSign = motors[i]->pidSetpoint > 0 ? 1 : -1;
    //   motors[i]->output = constrain(motors[i]->pidOutput + ((motors[i]->kS * setpointSign) + (motors[i]->kV * motors[i]->pidSetpoint)), -100, 100);
    // }

    rotationPidInput = moduleRotation;
    rotationPid.Compute();
    int pidOutputSign = rotationPidOutput > 0 ? 1 : -1;
    double output = constrain(rotationPidOutput + (kS * pidOutputSign), -100, 100);
    motorA.output = output;
    motorB.output = output;
  }

  setPercentOut(motorA, motorA.output);
  setPercentOut(motorB, motorB.output);

  // Serial.print(String(motorA.input, 1) + "AI , ");
  // Serial.print(String(motorA.rotation, 1) + " AO , ");
  // Serial.print(String(motorA.rps) + "r/s A , ");
  // Serial.print(String(motorB.input, 1) + "BI , ");
  // Serial.print(String(motorB.pidOutput, 1) + " BO , ")
  // Serial.print(String(motorB.rps) + "r/s B , ");
  Serial.print(String(motorA.output) + " O, ");
  Serial.println(String(moduleRotation) + "r M , ");
}

void setPercentOut(Motor motor, double percentOut) {
  bool direction = percentOut >= 0;
  if (motor.reversePositive) {
    direction = !direction;
  }
  if (direction) {
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, HIGH);
  } else {
    digitalWrite(motor.in1, HIGH);
    digitalWrite(motor.in2, LOW);
  }

  analogWrite(motor.en, map(abs(percentOut), 0, 100, 0, 255));
}

double readMotorInputs() {
  while (Serial.available() && newData == false) {
    int inChar = Serial.read();
    if (inChar == '!') { // PID MODE
      pidMode = !pidMode;
      break;
    }
    if (inChar == '=') { // ZERO MODULE
      moduleRotation = 0;
      break;
    }
    if (isDigit(inChar) || inChar == '.' || inChar == '-') {
      inString += (char)inChar;
    }
    if ((inChar == ',' || inChar == '\n') && dataIndex <= 1) {
      motors[dataIndex]->input = constrain(inString.toDouble(), -100, 100);
      Serial.println(motors[dataIndex]->input);
      Serial.println(motorA.input);
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
void updateMotorVelocities() {
  updateMotorVelocity(motorA);
  updateMotorVelocity(motorB);
}

void updateMotorVelocity(Motor &motor) {
  unsigned long currMicros = micros();
  if (currMicros > motor.lastRpsUpdateMicros) {
    double secondsDiff = (currMicros - motor.lastRpsUpdateMicros) / 1000000.0;
    if (motor.isUpdated || secondsDiff > 0.1) {
      double rotationDiff = motor.rotation - motor.lastRotation;
      motor.rps = rotationDiff / secondsDiff;
      motor.lastRotation = motor.rotation;
      motor.isUpdated = false;
      motor.lastRpsUpdateMicros = currMicros;
    }
  } else {
    motor.lastRpsUpdateMicros = currMicros;
  }
}

void updateMotorARotation() {
  updateMotorRotation(motorA);
}

void updateMotorBRotation() {
  updateMotorRotation(motorB);
}

void updateMotorRotation(Motor &motor) {
  int c1Val = digitalRead(motor.c1);

  if (c1Val != digitalRead(motor.c2)) {
    // CCW
    motor.ticks += 1;
    moduleRotation += moduleRotationPerTick;
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    // CW
    motor.ticks -= 1;
    moduleRotation -= moduleRotationPerTick;
    digitalWrite(LED_BUILTIN, LOW);
  }

  motor.rotation = motor.ticks * tickToRotationRatio * motorGearRatio;
  motor.isUpdated = true;
}
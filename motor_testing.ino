#include <ArduPID.h>

const double tickToRotationRatio = 1.0 / 4.0;
const double motorGearRatio = 1.0 / 100.0;
const double driveGearRatio = 17.0 / 60.0;
const double moduleRotationPerTick = tickToRotationRatio * motorGearRatio * driveGearRatio / 2.0;
double moduleRotation = 0;

bool newData = false;
int dataIndex = 0;
String inString = "";

bool pidMode = true;

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
  ArduPID pid;
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
  0, 0, 0,   // kP, kI, kD
  0, 0,      // Ks, Kv
  ArduPID()  // PID created later
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
  5, 0, 0,   // kP, kI, kD
  5, 7,      // Ks, Kv
  ArduPID()  // PID created later
};

Motor *motors[2] = { &motorA, &motorB };

// module rotation pid
double rotationPidInput = 0, rotationPidOutput = 0, rotationPidSetpoint = 0;
double kS = 0;
double kPFast = 20, kIFast = 1, kDFast = 0, kSFast = 0;
double kPSlow = 7, kISlow = 0, kDSlow = 0, kSSlow = .5;
ArduPID rotationPid;
bool fastPidMode = true;

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
    motors[i]->pid.begin(
      &motors[i]->pidInput, &motors[i]->pidOutput, &motors[i]->pidSetpoint,
      motors[i]->kP, motors[i]->kI, motors[i]->kD);
    motors[i]->pid.setOutputLimits(-100, 100);
    motors[i]->pid.start();
  }

  rotationPid.begin(&rotationPidInput, &rotationPidOutput, &rotationPidSetpoint, kPFast, kIFast, kDFast);
  rotationPid.setOutputLimits(-9, 9);
  rotationPid.start();

  Serial.begin(9600);
  Serial.println("<Arduino program starting...>");
}

void loop() {
  readMotorInputs();
  updateMotorVelocities();

  if (newData && !pidMode) {
    motorA.output = motorA.input;
    motorB.output = motorB.input;
    newData = false;
  }
  // Serial.print(String(digitalRead(motorA.c1)) + ", " + String(digitalRead(motorA.c2)));
  // Serial.print(String(motorA.c1) + ", " + String(motorA.c2));

  if (pidMode) {
    if (newData) {
      motorA.pidSetpoint = motorA.input;
      motorB.pidSetpoint = motorB.input;

      // rotationPidSetpoint = motorA.input;

      newData = false;
    }

    // if (abs(rotationPidSetpoint - moduleRotation) > 0.05) {
    //   int direction = rotationPidSetpoint > moduleRotation ? 1 : -1;
    //   motorA.pidSetpoint = 9 * direction;
    //   motorB.pidSetpoint = 9 * direction;
    // } else {
    //   motorA.pidSetpoint = 0;
    //   motorB.pidSetpoint = 0;
    // }

    // double error = abs(rotationPidSetpoint - moduleRotation);
    // double errorPidSlowThreshold = 0.2;
    // if (error > errorPidSlowThreshold && !fastPidMode) {
    //   setRotationPIDMode(false);
    // } else if (error <= errorPidSlowThreshold && fastPidMode) {
    //   setRotationPIDMode(false);
    // }

    // if (error > 0.005) {
    //   rotationPidInput = moduleRotation;
    //   rotationPid.compute();
    //   int pidOutputSign = rotationPidOutput > 0 ? 1 : -1;
    //   double output = constrain(rotationPidOutput + (kS * pidOutputSign), -9, 9);
    //   motorA.pidSetpoint = output;
    //   motorB.pidSetpoint = output;
    // } else {
    //   motorA.pidSetpoint = 0;
    //   motorB.pidSetpoint = 0;
    // }

    for (int i = 0; i < 2; i++) {
      motors[i]->pidInput = motors[i]->rps;
      motors[i]->pid.compute();
      int setpointSign = motors[i]->pidSetpoint > 0 ? 1 : -1;
      motors[i]->output = constrain(motors[i]->pidOutput + ((motors[i]->kS * setpointSign) + (motors[i]->kV * motors[i]->pidSetpoint)), -100, 100);
    }
  }

  setPercentOut(motorA, motorA.output);
  setPercentOut(motorB, motorB.output);

  // Serial.print(String(motorA.input, 1) + "AI , ");
  // Serial.print(String(motorA.output, 1) + " AO , ");
  Serial.print(String(motorA.rps) + "r/s A , ");
  // Serial.print(String(motorB.input, 1) + "BI , ");
  // Serial.print(String(motorB.output, 1) + " BO , ");
  Serial.print(String(motorB.rps) + "r/s B , ");

  // Serial.print(String(fastPidMode) + "r M , ");
  // Serial.print(String(motorA.output / 100.0) + " O, ");
  // Serial.print(String(rotationPidSetpoint) + "r M , ");
  // Serial.print(String(moduleRotation) + "r M , ");

  Serial.println();
}

void setRotationPIDMode(boolean fast) {
  fastPidMode = fast;
  if (fast) {
    rotationPid.setCoefficients(kPFast, kIFast, kDFast);
    kS = kSFast;
  } else {
    rotationPid.setCoefficients(kPSlow, kISlow, kDSlow);
    kS = kSSlow;
  }
  rotationPid.reset();
  rotationPid.start();
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
    if (inChar == '!') {  // PID MODE
      pidMode = !pidMode;
      break;
    }
    if (inChar == '=') {  // ZERO MODULE
      moduleRotation = 0;
      break;
    }
    if (isDigit(inChar) || inChar == '.' || inChar == '-') {
      inString += (char)inChar;
    }
    if ((inChar == ',' || inChar == '\n') && dataIndex <= 1) {
      motors[dataIndex]->input = constrain(inString.toDouble(), -100, 100);
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
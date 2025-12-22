#include <algorithm>
#include <iostream>
#include <math.h>
#include <pigpio.h>
#include <thread>

#include "MiniPID.h"
#include "logger.h"
#include "n20_motor.h"
#include "util.h"

N20Motor::N20Motor(int en, int in, int c1, int c2)
    : en(en), in(in), c1(c1), c2(c2), ticks(0), rotations(0),
      lastVelUpdateRotations(0), pid(MiniPID(0, 0, 0)) {
  lastVelUpdateMicros = Util::micros();

  gpioSetMode(en, PI_OUTPUT);
  gpioSetMode(in, PI_OUTPUT);
  gpioSetMode(c1, PI_INPUT);
  gpioSetMode(c2, PI_INPUT);

  gpioSetAlertFuncEx(c1, updateRotationCallback, this);
  gpioSetAlertFuncEx(c2, updateRotationCallback, this);

  pid.setOutputLimits(-1, 1);

  running = true;
  updateVelocityThread = std::thread(&N20Motor::updateVelocityThreadCall, this);
}

void N20Motor::setPIDF(float kP, float kI, float kD, float kF,
                       float kKineticFriction, float kStaticFriction) {
  pid.setP(kP);
  pid.setI(kI);
  pid.setD(kD);
  pid.setF(kF);
  this->kKineticFriction = kKineticFriction;
  this->kStaticFriction = kStaticFriction;
}

void N20Motor::setPercentOut(float percentOut) {
  double clamped = std::clamp(percentOut, -1.0f, 1.0f);
  gpioWrite(in, clamped >= 0);
  gpioPWM(en, (int)Util::mapIntoRange(fabs(clamped), 0, 1, 0, 255));
  Logger::logger() << clamped << std::endl;
}

void N20Motor::setTargetVelocity(float targetRps) {
  double output = pid.getOutput(rps, targetRps);
  double sign = copysign(1.0, output);
  if (abs(this->getRps()) < 0.00001) {
    output += sign * this->kStaticFriction;
  } else {
    output += sign * this->kKineticFriction;
  }
  setPercentOut(output);
}

void N20Motor::updateVelocityThreadCall() {
  while (running) {
    double currMicros = Util::micros();
    double currRotations = getRotations();
    double diffRotation = currRotations - this->lastVelUpdateRotations;
    double diffSec = (currMicros - this->lastVelUpdateMicros) / 1000000.0;

    this->rps = diffRotation / diffSec;

    this->lastVelUpdateMicros = currMicros;
    this->lastVelUpdateRotations = currRotations;

    // 20 milliseconds
    gpioDelay(20000);
  }
}

static void updateRotationCallback(int gpio, int level, uint32_t _tick,
                                   void *userdata) {
  N20Motor *motor = (N20Motor *)userdata;
  if (gpio == motor->c1) {
    motor->levC1 = level;
  } else {
    motor->levC2 = level;
  }

  // debounce
  if (gpio == motor->lastLevGPIO) {
    return;
  }
  motor->lastLevGPIO = gpio;

  // quadrature encoder
  if (gpio == motor->c1) {
    if (level == 1) {
      if (motor->levC2 == 1)
        motor->ticks++;
      else
        motor->ticks--;
    } else {
      if (motor->levC2 == 0)
        motor->ticks++;
      else
        motor->ticks--;
    }
  } else if (gpio == motor->c2) {
    if (level == 1) {
      if (motor->levC1 == 1)
        motor->ticks--;
      else
        motor->ticks++;
    } else {
      if (motor->levC1 == 0)
        motor->ticks--;
      else
        motor->ticks++;
    }
  }
  motor->rotations = motor->ticks * N20Motor::ROTATIONS_TO_TICKS;
}

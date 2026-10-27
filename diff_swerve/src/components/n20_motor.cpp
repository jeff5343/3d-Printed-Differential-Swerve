#include "MiniPID.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <math.h>
#include <mutex>
#include <pigpio.h>
#include <thread>

#include "components/n20_motor.h"
#include "utils/logger.h"
#include "utils/util.h"

N20Motor::N20Motor(int en, int in, int c1, int c2)
    : en(en), in(in), c1(c1), c2(c2), ticks(0), rotations(0),
      pid(MiniPID(0, 0, 0)) {
  gpioSetMode(en, PI_OUTPUT);
  gpioSetMode(in, PI_OUTPUT);
  gpioSetMode(c1, PI_INPUT);
  gpioSetMode(c2, PI_INPUT);

  gpioSetAlertFuncEx(c1, _updateRotationCallback, this);
  gpioSetAlertFuncEx(c2, _updateRotationCallback, this);

  pid.setOutputLimits(-1, 1);

  std::lock_guard<std::mutex> runLk(runMtx);
  running = true;
  updateVelocityThread = std::thread(&N20Motor::updateVelocityTask, this);
}

N20Motor::~N20Motor() {
  gpioPWM(en, 0);
  gpioSetAlertFuncEx(c1, nullptr, nullptr);
  gpioSetAlertFuncEx(c2, nullptr, nullptr);

  {
    std::lock_guard<std::mutex> runLk(runMtx);
    running = false;
  }
  cv.notify_all();
  if (updateVelocityThread.joinable())
    updateVelocityThread.join();
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
  gpioPWM(en,
          static_cast<int>(Util::mapIntoRange(fabs(clamped), 0, 1, 0, 255)));
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

void N20Motor::updateVelocityTask() {
  std::unique_lock<std::mutex> runLk(runMtx);

  float lastRotations = getRotations();
  double lastMicros = Util::micros();
  std::chrono::time_point<std::chrono::steady_clock> timePoint =
      std::chrono::steady_clock::now();

  while (running) {
    runLk.unlock();
    double rotations = getRotations();
    double micros = Util::micros();
    double diffRotations = rotations - lastRotations;
    double diffSec = (micros - lastMicros) / 1000000.0;
    lastRotations = rotations;
    lastMicros = micros;

    // TODO: not necessary for now, but think about using Kalman Filter
    if (diffSec > 0)
      this->rps = diffRotations / diffSec;

    timePoint += std::chrono::milliseconds(20);
    runLk.lock();
    cv.wait_until(runLk, timePoint, [this]() { return !running; });
  }
}

void N20Motor::_updateRotationCallback(int gpio, int level, uint32_t _tick,
                                       void *userdata) {
  N20Motor *motor = static_cast<N20Motor *>(userdata);

  // quadrature encoder
  if (gpio == motor->c1) {
    if (level == motor->prevC2) {
      motor->ticks++;
    } else {
      motor->ticks--;
    }
  } else if (gpio == motor->c2) {
    if (level == motor->prevC1) {
      motor->ticks--;
    } else {
      motor->ticks++;
    }
  }

  if (gpio == motor->c1) {
    motor->prevC1 = level;
  } else {
    motor->prevC2 = level;
  }
  motor->rotations = motor->ticks * N20Motor::TICKS_TO_ROTATIONS;
}

#ifndef N20_MOTOR_
#define N20_MOTOR_

#include "MiniPID.h"

#include "logger.h"
#include <atomic>
#include <iostream>

class N20Motor {
public:
  N20Motor(int en, int in, int c1, int c2);
  N20Motor() : en(0), in(0), c1(0), c2(0) {};
  void setPIDF(float kP, float kI, float kD, float kF, float kKineticFriction,
               float kStaticFriction);
  void setPercentOut(float percentOut);
  void setTargetVelocity(float targetRps);
  float getRotations() { return rotations; }
  void setRotations(float rotations) {
    this->ticks = rotations * (1.0 / ROTATIONS_TO_TICKS);
    this->rotations = rotations;
  }
  float getRps() { return rps; }
  int getTicks() { return ticks; }

  // constants
  static constexpr float ROTATIONS_TO_TICKS = (1.0 / 200.0) * (1.0 / 4.0);
  // GPIOs
  int en;
  int in, c1, c2;
  // data
  float kKineticFriction, kStaticFriction;
  std::atomic<int> ticks;
  std::atomic<int> levC1, levC2;
  std::atomic<int> lastLevGPIO;
  std::atomic<float> rotations, rps;
  std::atomic<float> lastVelUpdateRotations, lastVelUpdateMicros;
  // PID
  MiniPID pid;
  // thread
  std::atomic<bool> running;
  std::thread updateVelocityThread;
  void updateVelocityThreadCall();
};

#endif

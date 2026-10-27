#pragma once

#include "MiniPID.h"
#include <atomic>
#include <condition_variable>
#include <thread>

class N20Motor {
public:
  N20Motor(int en, int in, int c1, int c2);

  void setPIDF(float kP, float kI, float kD, float kF, float kKineticFriction,
               float kStaticFriction);
  void setPercentOut(float percentOut);
  void setTargetVelocity(float targetRps);

  float getRotations() const { return rotations; }
  void setRotations(float rotations) {
    this->ticks = rotations * (1.0 / TICKS_TO_ROTATIONS);
    this->rotations = rotations;
  }
  float getRps() const { return rps; }
  int getTicks() const { return ticks; }

  ~N20Motor();

private:
  // encoder callback
  static void _updateRotationCallback(int gpio, int level, uint32_t _tick,
                                      void *userdata);

  // constant
  static constexpr float TICKS_TO_ROTATIONS = (1.0 / 200.0) * (1.0 / 4.0);

  // GPIO
  const int en;
  const int in, c1, c2;

  // PID
  MiniPID pid;
  float kKineticFriction{}, kStaticFriction{};

  // velocity estimation thread
  std::thread updateVelocityThread;
  void updateVelocityTask();
  bool running{false};
  std::mutex runMtx{};
  std::condition_variable cv{};
  std::atomic<float> rps{};

  // quadrature encoder callback
  std::atomic<int> prevC1{};
  std::atomic<int> prevC2{};
  std::atomic<int> ticks{};
  std::atomic<float> rotations{};
};

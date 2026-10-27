#pragma once

#include <atomic>
#include <condition_variable>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <mutex>
#include <string>
#include <thread>

class XboxController {
public:
  XboxController(std::string eventName);
  ~XboxController();
  bool isReady() const { return ready; };
  float getLeftY() const { return leftY; };
  float getLeftX() const { return leftX; };
  float getRightY() const { return rightY; };
  float getRightX() const { return rightX; };
  int getButtonPressed() const { return buttonPressed; };

private:
  const int UPDATE_HZ = 50;

  bool isConnected() const;
  bool initializeLibevdev();
  void cleanupLibevdev();
  void processEvent(const input_event &ev);

  std::string eventName;
  std::atomic<bool> ready{false};

  // controller thread
  std::thread controllerThread;
  void controllerTask();
  mutable std::mutex runningMtx{};
  std::condition_variable cv{};
  bool running{false};

  // data
  std::atomic<float> leftY{0.0f}, leftX{0.0f}, rightY{0.0f}, rightX{0.0f};
  std::atomic<int> buttonPressed{-1};

  // libevdev
  mutable std::mutex devMtx{};
  libevdev *dev{nullptr};
};

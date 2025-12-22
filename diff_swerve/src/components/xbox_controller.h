#ifndef XBOX_CONTROLLER_
#define XBOX_CONTROLLER_

#include <string>
#include <thread>
#include <atomic>

class XboxController {

public:
  XboxController(std::string eventName);
  ~XboxController();
  libevdev *initializeLibevdev();
  bool isConnected();
  bool isReady() { return ready; };
  float getLeftY() { return leftY; };
  float getLeftX() { return leftX; };
  float getRightY() { return rightY; };
  float getRightX() { return rightX; };
  int getButtonPressed() { return buttonPressed; };

private:
  std::string eventName;
  bool ready;

  // thread
  std::atomic<bool> running;
  std::thread controllerThread;
  void controllerThreadCall();

  // data
  std::atomic<float> leftY, leftX, rightY, rightX;
  std::atomic<int> buttonPressed;
};

#endif

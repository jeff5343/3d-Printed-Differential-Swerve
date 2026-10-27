#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <mutex>
#include <pigpio.h>
#include <thread>
#include <unistd.h>

#include "components/xbox_controller.h"
#include "utils/util.h"

XboxController::XboxController(std::string eventName) : eventName(eventName) {
  std::lock_guard<std::mutex> runLk(runningMtx);
  running = true;
  controllerThread = std::thread(&XboxController::controllerTask, this);
}

XboxController::~XboxController() {
  {
    std::lock_guard<std::mutex> runLk(runningMtx);
    running = false;
  }
  cv.notify_all();
  if (controllerThread.joinable()) {
    controllerThread.join();
  }
  cleanupLibevdev();
}

bool XboxController::isConnected() const {
  int fd = open(("/dev/input/" + eventName).c_str(), O_RDONLY);
  close(fd);
  return fd >= 0;
}

bool XboxController::initializeLibevdev() {
  std::lock_guard<std::mutex> lk(devMtx);
  int fd = open(("/dev/input/" + eventName).c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    std::cout << "failed to open joystick device" << std::endl;
    return false;
  }
  int rc = libevdev_new_from_fd(fd, &dev);
  if (rc < 0) {
    std::cout << "failed to initalize libevdev on joystick device" << std::endl;
    close(fd);
    return false;
  }
  return true;
}

void XboxController::cleanupLibevdev() {
  std::lock_guard<std::mutex> devLk(devMtx);
  if (!dev)
    return;
  int fd = libevdev_get_fd(dev);
  libevdev_free(dev);
  if (fd >= 0)
    close(fd);
  dev = nullptr;
  ready = false;
}

void XboxController::controllerTask() {
  std::unique_lock<std::mutex> runLk(runningMtx);
  std::unique_lock<std::mutex> devLk(devMtx);
  devLk.unlock();

  std::chrono::time_point<std::chrono::steady_clock> timePoint =
      std::chrono::steady_clock::now();

  while (running) {
    runLk.unlock();

    devLk.lock();
    bool libevdevUninitialized = dev == nullptr;
    devLk.unlock();

    if (libevdevUninitialized) {
      // check for xbox connection
      if (isConnected()) {
        // wait 50ms ensure xbox is still connected
        timePoint += std::chrono::milliseconds(50);
        runLk.lock();
        cv.wait_until(runLk, timePoint, [this]() { return !running; });
        if (isConnected() && initializeLibevdev()) {
          // initalize variables
          leftY = 0.0f;
          leftX = 0.0f;
          rightY = 0.0f;
          rightX = 0.0f;
          buttonPressed = -1;
          ready = true;
        }
      } else {
        timePoint += std::chrono::milliseconds(500);
        runLk.lock();
        cv.wait_until(runLk, timePoint, [this]() { return !running; });
      }
      continue;
    }

    struct input_event ev;
    devLk.lock();
    // read normal events
    int rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
    while (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
      processEvent(ev);
      rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
    }
    // check buffer overflow, read synthetic events
    if (rc == LIBEVDEV_READ_STATUS_SYNC) {
      rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_SYNC, &ev);
      while (rc == LIBEVDEV_READ_STATUS_SYNC) {
        processEvent(ev);
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_SYNC, &ev);
      }
    }
    devLk.unlock();
    // check xbox disconnection
    if (rc == -ENODEV) {
      cleanupLibevdev();
    }
    // error
    if (rc != -EAGAIN) {
      std::cout << "failed to read next event: " << -rc << std::endl;
    }

    timePoint += std::chrono::milliseconds(20);
    runLk.lock();
    cv.wait_until(runLk, timePoint, [this]() { return !running; });
  }
}

void XboxController::processEvent(const input_event &ev) {
  if (ev.type == EV_ABS) {
    float value = Util::mapIntoRange(ev.value, 0, 65535, -1, 1);
    switch (ev.code) {
    case ABS_Y:
      this->leftY = value;
      break;
    case ABS_X:
      this->leftX = value;
      break;
    case ABS_RZ:
      this->rightY = value;
      break;
    case ABS_Z:
      this->rightX = value;
      break;
    }
  } else if (ev.type == EV_KEY) {
    if (ev.value == 1) {
      this->buttonPressed = ev.code;
    } else {
      this->buttonPressed = -1;
    }
  }
}

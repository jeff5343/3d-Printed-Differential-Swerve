#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <pigpio.h>
#include <stdexcept>
#include <thread>

#include "util.h"
#include "xbox_controller.h"

XboxController::XboxController(std::string eventName)
    : eventName(eventName), running(true) {
  this->controllerThread =
      std::thread(&XboxController::controllerThreadCall, this);
}

XboxController::~XboxController() {
  running = false;
  if (controllerThread.joinable()) {
    controllerThread.join();
  }
}

libevdev *XboxController::initializeLibevdev() {
  struct libevdev *dev = NULL;
  int fd = open(("/dev/input/" + eventName).c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    std::cout << "failed to open joystick device" << std::endl;
    return NULL;
  }
  int err = libevdev_new_from_fd(fd, &dev);
  if (err < 0) {
    std::cout << "failed to initalize libevdev on joystick device" << std::endl;
    return NULL;
  }
  return dev;
}

bool XboxController::isConnected() {
  std::ifstream eventFile(("/dev/input/" + eventName).c_str());
  bool connected = eventFile.good();
  eventFile.close();
  return connected;
}

void XboxController::controllerThreadCall() {
  struct libevdev *dev = NULL;
  struct input_event ev;

  while (running) {
    // check for xbox connection
    if (dev == NULL) {
      if (this->isConnected()) {
        // wait to make sure it is still connected
        gpioDelay(50000);
        if (this->isConnected()) {
          dev = this->initializeLibevdev();
          this->ready = true;
          this->leftY = 0;
          this->leftX = 0;
          this->rightY = 0;
          this->rightX = 0;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      continue;
    }

    // update controller data
    int err = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
    if (err == 0) {
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
    } else if (err != -EAGAIN) {
      if (err == -19 && !this->isConnected()) {
        int fd = libevdev_get_fd(dev);
        libevdev_free(dev);
        std::close(fd);
        dev = NULL;
        this->ready = false;
      } else {
        std::cout << "failed to read next event: " << -err << std::endl;
        break;
      }
    }
  }

  // free resources
  if (dev != NULL) {
    int fd = libevdev_get_fd(dev);
    libevdev_free(dev);
    std::close(fd);
    dev = NULL;
  }
  this->ready = false;
}

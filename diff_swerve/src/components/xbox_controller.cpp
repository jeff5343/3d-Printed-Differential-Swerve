#include <iostream>
#include <fcntl.h>
#include <fstream>
#include <thread>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <stdexcept>
#include <pigpio.h>

#include "xbox_controller.h"
#include "util.h"


void XboxController::controllerThread() {
	struct libevdev* dev = NULL;
	struct input_event ev;

	while (true) {
		// check for xbox connection
		if (dev == NULL) {
			if (this->isConnected()) {
				// wait to make sure it is still connected
				gpioDelay(50000);
				if (this->isConnected()) {
					dev = this->initializeLibevdev();
					this->ready = true;
				}
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
				libevdev_free(dev);
				dev = NULL;
				this->ready = false;
			} else {
				std::cout << "failed to read next event: " << -err << std::endl;
				break;
			}
		}
	}
}

XboxController::XboxController(std::string eventName) {
	this->eventName = eventName;
	std::thread controllerThread(&XboxController::controllerThread, this);
	controllerThread.detach();
}

bool XboxController::isConnected() {
	std::ifstream eventFile(("/dev/input/" + eventName).c_str());
	return eventFile.good();
}

libevdev* XboxController::initializeLibevdev() {
	struct libevdev* dev = NULL;
	int fd = open(("/dev/input/" + eventName).c_str(), O_RDONLY|O_NONBLOCK); 
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

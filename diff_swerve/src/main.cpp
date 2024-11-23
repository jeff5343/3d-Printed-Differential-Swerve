#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <pigpio.h>
#include <cmath>
#include <math.h>
#include <string.h>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <fstream>

#include "MiniPID.h"
#include "n20_motor.h"
#include "xbox_controller.h"
#include "swerve_module.h"
#include "logger.h"
#include "util.h"

using std::cout;
using std::cin;
using std::endl;
using std::to_string;
using std::string;

int main() {
	Logger::setup();
	Logger::logger() << "<Program starting...>" << endl;

	// set sample rate of 2 us
	gpioCfgClock(2, 0, -1);
	gpioCfgClock(2, 1, -1);
	if (gpioInitialise() < 0) {
		Logger::logger() << "pigpio failed to initialize";
		return -1;
	}

	SwerveModule module(SwerveModuleConstants::SwerveModuleConstants(
		12, 16, 20, 21, // right motor pins
		24, 25, 8, 7, // left motor pins
		PIDConstants::PIDConstants(0.01, 0, 0, 0.075), // motor PID constants
		PIDConstants::PIDConstants(20, 0, 0, 0, 2.5, 2.5), // rotation PID constants
	));

	// controller
	XboxController controller(std::string("event2"));
	float inputRight = 0, inputLeft = 0;
	bool wasReady = false;

	while (true) {
		// wait for xbox connection
		bool isReady = controller.isReady();
		if (!isReady) {
			Logger::logger() << "waiting for xbox controller connection..." << endl;
			gpioSleep(PI_TIME_RELATIVE, 1, 0);
			continue;
		}
		if (!wasReady && isReady) {
			Logger::logger() << "xbox controller is connected" << endl;
		}
		wasReady = isReady;

		// input
		float leftY = controller.getLeftY(), leftX = controller.getLeftX();

		// reset position
		if (controller.getButtonPressed() == BTN_TL) {
			module.resetPosition();
			Logger::logger() << "pressed BTN_TL" << endl;
		}

		// rotation control
		float rotationTarget = 0;
		float targetVel = (sqrt(pow(leftX, 2) + pow(leftY, 2))) * 9;

		if (abs(leftX) > 0.1 || abs(leftY) > 0.1) {
			rotationTarget = (atan2(leftY, leftX) / (2.0 * M_PI)) + .25;
			if (rotationTarget < 0) {
				rotationTarget = 1 + rotationTarget;
			}
			rotationTarget = 1 - rotationTarget;
		} else {
			rotationTarget = 0;
		}

		float rotError = module.getErrorToTargetRotation();

		if (rotError > 0.01 && rotationTarget != 0) {
			module.setTargetRotation(rotationTarget);
			Logger::logger() << "ROTATING!" << rotationTarget << endl;
		} else {
			module.setTargetVelocity(targetVel);
			Logger::logger() << "DRIVIGN!" << targetVel << endl;
		}

		// TELEMETRY
		//Logger::plotter() << leftMotor.getRps() << " " << targetVel << " " << rightMotor.getRps() << endl;

		//// Logger::logger() << leftY) << ", " << leftX << " | ";
		//Logger::logger() << inputRight << ", " << inputLeft << endl;
		//Logger::logger() << rightMotor.getRotations() << " | " << leftMotor.getRotations() << endl;
		//Logger::logger() << moduleRotation << " rotations" << endl;
		//Logger::logger() << rotationTarget << " !!" <<  endl;
		//Logger::logger() << finalTarget << " !! TARGET!" <<  endl;
		//Logger::logger() << targetVel << " !! TARGET VELOCITY!" <<  endl;
		//Logger::logger() << velOutput << " !!~" <<  endl;
		//Logger::logger() << atan2(leftY, leftX) / (2.0 * M_PI) << " !!~" <<  endl;
		// Logger::logger() << rightMotor.getRps() << " | " << leftMotor.getRps();
		// Logger::logger() << rightMotor.ticks << " | " << leftMotor.ticks;

		Logger::logger() << "-------------------" << endl;

		// TODO: subtract how long system took in the body of the for loop?
		gpioDelay(20000);

		// exit program
		if (controller.getButtonPressed() == BTN_START) {
			break;
		}
	}

	module.stop();

	gpioTerminate();
}

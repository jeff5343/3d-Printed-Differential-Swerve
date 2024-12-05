#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <pigpio.h>
#include <cmath>
#include <math.h>
#include <string.h>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <fstream>
#include <list>

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
using std::list;

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

	SwerveModule frontModule(SwerveModuleConstants(
		14, 23, 15, 18, // right motor pins
		2, 17, 3, 4, // left motor pins
		PIDConstants(0.01, 0, 0, 0.075, 0, 0), // motor PID constants
		PIDConstants(20, 0, 0, 0, 0, 0) // rotation PID constants
	));
	SwerveModule rightModule(SwerveModuleConstants(
		12, 16, 20, 21, // right motor pins
		24, 25, 8, 7, // left motor pins
		PIDConstants(0.01, 0, 0, 0.075, 0, 0), // motor PID constants
		PIDConstants(5, 0, 0, 0, 0, 0) // rotation PID constants
	));
	SwerveModule leftModule(SwerveModuleConstants(
		10, 5, 9, 11, // right motor pins
		6, 13, 19, 26, // left motor pins
		PIDConstants(0.01, 0, 0, 0.075, 0, 0), // motor PID constants
		PIDConstants(20, 0, 0, 0, 0, 0) // rotation PID constants
	));

	SwerveModule* modules[3] = {&frontModule, &rightModule, &leftModule};
	// SwerveModule* modules[1] = {&frontModule};

	// controller
	XboxController controller(std::string("event2"));
	float inputRight = 0, inputLeft = 0;
	bool wasReady = false;

	// telemetry
	list<float> rpsLValues{};
	list<float> rpsRValues{};

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
		Logger::logger() << leftY << " " << leftX;

		// reset position
		if (controller.getButtonPressed() == BTN_TL) {
			for (auto module : modules) {
				module->resetPosition();
			}
			Logger::logger() << "pressed BTN_TL" << endl;
		}

		// rotation control
		float rotationTarget = 0;
		if (abs(leftX) > 0.1 || abs(leftY) > 0.1) {
			rotationTarget = (atan2(leftY, leftX) / (2.0 * M_PI)) + .25;
			if (rotationTarget < 0) {
				rotationTarget = 1 + rotationTarget;
			}
			rotationTarget = 1 - rotationTarget;
		} else {
			rotationTarget = 0;
		}

		Logger::logger() << "rot target:" << rotationTarget << endl;
		// float targetVel = (sqrt(pow(leftX, 2) + pow(leftY, 2))) * 7.5;
		float targetVel = (sqrt(pow(leftX, 2) + pow(leftY, 2))) * 100;
		for (auto module : modules) {
			module->rightMotor.setPercentOut(targetVel);
			module->leftMotor.setPercentOut(targetVel);
			Logger::logger() << "DRIVIGN!" << targetVel << endl;

			//float rotError = module->getErrorToTargetRotation(rotationTarget);
			//if (abs(rotError) > 0.01 && rotationTarget != 0) {
			//	module->setTargetRotation(rotationTarget);
			//	Logger::logger() << "ROTATING!" << rotationTarget << endl;
			//} else {
			//	module->setTargetVelocity(targetVel);
			//	Logger::logger() << "DRIVIGN!" << targetVel << endl;
			//}
			
			// TELEMETRY
			rpsLValues.push_back(module->leftMotor.getRps());		
			rpsRValues.push_back(module->rightMotor.getRps());		

			if (rpsLValues.size() > 50) {
				rpsLValues.pop_front();
			}
			if (rpsRValues.size() > 50) {
				rpsRValues.pop_front();
			}

			list<float>::iterator it;
			float avgRpsL = 0, avgRpsR = 0;
			for (it = rpsLValues.begin(); it != rpsLValues.end(); ++it) {
				avgRpsL += *it;
			}
			for (it = rpsRValues.begin(); it != rpsRValues.end(); ++it) {
				avgRpsR += *it;
			}
			avgRpsL = avgRpsL / rpsLValues.size();
			avgRpsR = avgRpsR / rpsRValues.size();

			// rps control
			Logger::logger() << "avg rps L :" << avgRpsL << endl;
			Logger::logger() << "avg rps R :" << avgRpsR << endl;
			Logger::logger() << "rps target:" << targetVel << endl;

			// rotation control
			// Logger::logger() << "rot L :" << module->leftMotor.getRotations() << endl;
			// Logger::logger() << "rot R :" << module->rightMotor.getRotations() << endl;
			// Logger::logger() << "rot:" << module->getRotation() << endl;
			// Logger::logger() << "error:" << rotError << endl;
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

	for (auto module : modules) {
		module->stop();
	}

	gpioTerminate();
}

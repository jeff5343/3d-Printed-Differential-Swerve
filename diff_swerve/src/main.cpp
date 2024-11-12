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

	// motors
	N20Motor rightMotor(18, 23, 12, 16);
	N20Motor leftMotor(24, 25, 20, 21);

	// kP, kI, kD, kF
	rightMotor.setPIDF(0.01, 0, 0, 0.075);
	leftMotor.setPIDF(0.01, 0, 0, 0.075);

	// controller
	XboxController controller(std::string("event2"));
	float inputRight = 0, inputLeft = 0;
	bool wasReady = false;

	const float moduleRotationPerMotorRotation = (17.0 / 60.0) / 2.0;
	float moduleRotation = 0;

	// rotation PID
	MiniPID rotationPid = MiniPID(20, 0, 0);
	rotationPid.setOutputLimits(-7, 7);
	float kS = 2.5;
	float kStaticFriction = 2.5;

	double rotationTarget = 0;

	while (true) {
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

		// percent out control
		float leftY = controller.getLeftY(), leftX = controller.getLeftX();
		// if (fabs(leftY) > 0.4) {
		// 	inputRight = -leftY;
		// 	inputLeft = -inputRight;
		// } else if (fabs(leftX) > 0.4) {
		// 	inputRight = -leftX;
		// 	inputLeft = -leftX;
		// } else {
		// 	inputRight = 0;
		// 	inputLeft = 0;
		// }

		// rightMotor.setPercentOut(inputRight);
		// leftMotor.setPercentOut(inputLeft);

		// velocity control
		// float targetVel = leftY * 9;
		// if (fabs(rightY) > 0.1) {
		// 	leftMotor.setTargetVelocity(targetVel);
		// 	rightMotor.setTargetVelocity(-targetVel);
		// } else {
		// 	leftMotor.setTargetVelocity(0);
		// 	rightMotor.setTargetVelocity(0);
		// }

		// reset position
		if (controller.getButtonPressed() == BTN_TL) {
			rightMotor.setRotations(0);		
			leftMotor.setRotations(0);		
			Logger::logger() << "pressed BTN_TL" << endl;
		}

		// calculate module rotation
		float rightMotorModuleRotations = rightMotor.getRotations() * moduleRotationPerMotorRotation;
		float leftMotorModuleRotations = leftMotor.getRotations() * moduleRotationPerMotorRotation;
		moduleRotation = rightMotorModuleRotations + leftMotorModuleRotations;

		float targetVel = (sqrt(pow(leftX, 2) + pow(leftY, 2))) * 9;

		// rotation position control
		if (abs(leftX) > 0.1 || abs(leftY) > 0.1) {
			rotationTarget = (atan2(leftY, leftX) / (2.0 * M_PI)) + .25;
			if (rotationTarget < 0) {
				rotationTarget = 1 + rotationTarget;
			}
			rotationTarget = 1 - rotationTarget;
		} else {
			rotationTarget = 0;
		}

		float whole;
		float relativeRotation = std::modf(moduleRotation, &whole);
		float diff =  rotationTarget - relativeRotation;
		// force difference to be between [-0.5, 0.5]
		if (diff > 0.5) {
			diff -= 1;
		} else if (diff < -0.5) {
			diff += 1;
		}
		float finalTarget = moduleRotation + diff;

		double velOutput = rotationPid.getOutput(moduleRotation, finalTarget);
		velOutput += copysign(1.0, velOutput) * kS;

		float rotError = abs(moduleRotation - finalTarget);

		// help start moving motors
		if (rotError > 0.01 && (leftMotor.rps < 0.01 || rightMotor.rps < 0.01)) {
			velOutput += copysign(1.0, velOutput) * kStaticFriction;
		}

		if (rotError > 0.01 && rotationTarget != 0) {
			leftMotor.setTargetVelocity(-velOutput);
			rightMotor.setTargetVelocity(-velOutput);

			Logger::logger() << "ROTATING!" << velOutput << endl;
		} else {
			leftMotor.setTargetVelocity(targetVel);
			rightMotor.setTargetVelocity(-targetVel);

			Logger::logger() << "DRIVIGN!" << targetVel << endl;
		}

		Logger::logger() << leftMotor.rps << " " << rightMotor.rps << endl;

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

		if (controller.getButtonPressed() == BTN_START) {
			break;
		}
	}

	rightMotor.setPercentOut(0);
	leftMotor.setPercentOut(0);

	gpioTerminate();
}

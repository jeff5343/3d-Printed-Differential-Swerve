#include <iostream>
#include <math.h>
#include <pigpio.h>
#include <thread>
#include <algorithm>

#include "MiniPID.h"
#include "n20_motor.h"
#include "logger.h"
#include "util.h"


static void updateRotationCallback(int gpio, int level, uint32_t _tick, void *userdata) {
	N20Motor *motor = (N20Motor*) userdata;

	if (gpio == motor->c1) {
		motor->levC1 = level;
	} else {
		motor->levC2 = level;
	}

	// debounce
	if (gpio == motor->lastLevGPIO) {
		return;
	}
	motor->lastLevGPIO = gpio;

	if (gpio == motor->c1) {
		if (level == 1) {
			if (motor->levC2 == 1) motor->ticks++;
			else motor->ticks--;
		} else {
			if (motor->levC2 == 0) motor->ticks++;
			else motor->ticks--;
		}
	} else if (gpio == motor->c2) {
		if (level == 1) {
			if (motor->levC1 == 1) motor->ticks--;
			else motor->ticks++;
		} else {
			if (motor->levC1 == 0) motor->ticks--;
			else motor->ticks++;
		}

	}
	motor->rotations = motor->ticks * N20Motor::ROTATIONS_TO_TICKS;
}

void N20Motor::updateVelocityThread() {
	while (true) {
		double currMicros = Util::micros();
		double currRotations = getRotations();
		double diffRotation = currRotations - this->lastVelUpdateRotations;
		double diffSec = (currMicros - this->lastVelUpdateMicros) / 1000000.0;

		this->rps = diffRotation / diffSec;

		this->lastVelUpdateMicros = currMicros;
		this ->lastVelUpdateRotations = currRotations;

		gpioDelay(20000);
	}
}

N20Motor::N20Motor(int en, int in, int c1, int c2) : pid(MiniPID(0, 0, 0)) {
	this->en = en;
	this->in = in;
	this->c1 = c1;
	this->c2 = c2;
	this->ticks = 0;
	this->rotations = 0;

	lastVelUpdateMicros = Util::micros();
	lastVelUpdateRotations = getRotations();

	gpioSetMode(en, PI_OUTPUT);
	gpioSetMode(in, PI_OUTPUT);
	gpioSetMode(c1, PI_INPUT);
	gpioSetMode(c2, PI_INPUT);

	gpioSetAlertFuncEx(c1, updateRotationCallback, this);
	gpioSetAlertFuncEx(c2, updateRotationCallback, this);

	pid.setOutputLimits(-1, 1);

	std::thread updateVelocityThread(&N20Motor::updateVelocityThread, this);
	updateVelocityThread.detach();
}

void N20Motor::setPIDF(float kP, float kI, float kD, float kF, float kKineticFriction, float kStaticFriction) {
	pid.setP(kP);
	pid.setI(kI);
	pid.setD(kD);
	pid.setF(kF);
	this->kKineticFriction = kKineticFriction;
	this->kStaticFriction = kStaticFriction;
}

void N20Motor::setPercentOut(float percentOut) {
	double clamped = std::clamp(percentOut, -1.0f, 1.0f);
	gpioWrite(in, clamped >= 0);
	gpioPWM(en, (int) Util::mapIntoRange(fabs(clamped), 0, 1, 0, 255)); 
	Logger::logger() << clamped << std::endl;
}

void N20Motor::setTargetVelocity(float targetRps) {
	double output = pid.getOutput(rps, targetRps);
	double sign = copysign(1.0, output); 
	if (abs(this->getRps()) < 0.00001) {
		output += sign * this->kStaticFriction;
	} else {
		output += sign * this->kKineticFriction;
	}
	setPercentOut(output);
}

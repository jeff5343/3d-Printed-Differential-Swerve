#ifndef N20_MOTOR_
#define N20_MOTOR_

#include "MiniPID.h"

#include "logger.h"
#include <iostream>

class N20Motor {
	public:
		// constants
		static constexpr float ROTATIONS_TO_TICKS = (1.0 / 200.0) * (1.0 / 4.0);
		// GPIOs
		int en;
		int in, c1, c2;
		// data
		volatile int ticks;
		volatile int levC1, levC2;
		volatile int lastLevGPIO;
		volatile float rotations, rps;
		volatile float lastVelUpdateRotations, lastVelUpdateMicros;
		// PID
		MiniPID	pid;

		// functions
		N20Motor(int en, int in, int c1, int c2);
		void setPIDF(float kP, float kI, float kD, float kF);
		void setPercentOut(float percentOut);
		void setTargetVelocity(float targetRps);
		void updateVelocityThread();
		float getRotations() { return rotations; }
		void setRotations(float rotations) {
			this->ticks = rotations * (1.0 / ROTATIONS_TO_TICKS);
			this->rotations = rotations;
		}
		float getRps() { return rps; }
		int getTicks() { return ticks; }
};

#endif

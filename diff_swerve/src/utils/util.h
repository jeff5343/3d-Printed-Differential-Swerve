#ifndef UTIL_
#define UTIL_

class Util {
	public:
		static long int micros();
		static float mapIntoRange(float x, float inputStart, float inputEnd, float outputStart, float outputEnd);
};

class PIDConstants {
	public:
		float kP, kI, kD, kF;
		float kKineticFriction, kStaticFriction;
		PIDConstants(float kP, float kI, float kD, float kF, float kKineticFriction, float kStaticFriction) : kP(kP), kI(kI), kD(kD), kF(kF), kKineticFriction(kKineticFriction), kStaticFriction(kStaticFriction) {};
		PIDConstants(float kP, float kI, float kD, float kF) : kP(kP), kI(kI), kD(kD), kF(kF), kKineticFriction(0), kStaticFriction(0) {};
		PIDConstants() : kP(0), kI(0), kD(0), kF(0), kKineticFriction(0), kStaticFriction(0) {};
};

#endif

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
		float kS, kStaticFriction;
		PIDConstants(float kP, float kI, float kD, float kF, float kS, float kStaticFriction) : kP(kP), kI(kI), kD(kD), kF(kF), kS(kS), kStaticFriction(kStaticFriction);
		PIDConstants(float kP, float kI, float kD, float kF) : kP(kP), kI(kI), kD(kD), kF(kF), kS(0), kStaticFriction(0);
}

#endif

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
		PIDConstants(kP, kI, kD, kF, kS, kStaticFriction) : kP, kI, kD, kF, kS, kStaticFriction;
		PIDConstants(kP, kI, kD, kF) : kP, kI, kD, kF; // TODO: set kS and kStaticFriction to 0
}

#endif

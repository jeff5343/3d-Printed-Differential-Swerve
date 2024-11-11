#ifndef UTIL_
#define UTIL_

class Util {
	public:
		static long int micros();
		static float mapIntoRange(float x, float inputStart, float inputEnd, float outputStart, float outputEnd);
};

#endif

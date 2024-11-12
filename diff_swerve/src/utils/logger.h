#ifndef LOGGER_
#define LOGGER_

#include <string.h>
#include <iostream>

class Logger {
	private:
		static bool isPlotting;
	public:
		static void setup();
		static std::ostream& logger();
		static std::ostream& plotter();
};

#endif

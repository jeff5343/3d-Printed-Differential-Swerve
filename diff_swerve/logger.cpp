#include <cstdlib>
#include <iostream>
#include <string.h>

#include "logger.h"
#include "util.h"

static std::ostream bitBucket(0);

bool Logger::isPlotting = false;

void Logger::setup() {
	if (const char* _env = std::getenv("PLOT")) {
		isPlotting = true;
	}
}

std::ostream& Logger::logger() {
	if (!isPlotting) {
		return std::cout;
	} else {
		return bitBucket;
	}
}

std::ostream& Logger::plotter() {
	if (isPlotting) {
		std::cout << Util::micros() / 1000000.0 << " ";
		return std::cout;
	} else {
		return bitBucket;
	}
}
	

#include <chrono>
#include <pigpio.h>

#include "util.h"

long int Util::micros() {
  int secs, us;
  gpioTime(PI_TIME_RELATIVE, &secs, &us);
  return (secs * 1000000) + us;
}

float Util::mapIntoRange(float input, float inputStart, float inputEnd,
                         float outputStart, float outputEnd) {
  float slope = (outputEnd - outputStart) / (inputEnd - inputStart);
  return outputStart + slope * (input - inputStart);
}

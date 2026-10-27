#include <pigpio.h>
#include <cstdint>

#include "util.h"

std::int64_t Util::micros() {
  int secs, us;
  gpioTime(PI_TIME_RELATIVE, &secs, &us);
  return static_cast<int64_t>(secs) * 1000000 + us;
}

float Util::mapIntoRange(float input, float inputStart, float inputEnd,
                         float outputStart, float outputEnd) {
  float slope = (outputEnd - outputStart) / (inputEnd - inputStart);
  return outputStart + slope * (input - inputStart);
}

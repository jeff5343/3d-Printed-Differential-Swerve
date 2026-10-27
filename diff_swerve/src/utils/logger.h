#pragma once

#include <iostream>
#include <string.h>

class Logger {
private:
  static bool isPlotting;

public:
  static void setup();
  static std::ostream &logger();
  static std::ostream &plotter();
};

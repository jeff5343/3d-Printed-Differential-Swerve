#pragma once

#include <cstdint>

constexpr unsigned PI_INPUT = 0;
constexpr unsigned PI_OUTPUT = 1;
constexpr unsigned PI_TIME_RELATIVE = 0;

using gpioAlertFuncEx_t = void (*)(int, int, uint32_t, void *);

int gpioCfgClock(unsigned micros, unsigned peripheral, unsigned source);
int gpioInitialise();
void gpioTerminate();
int gpioSetMode(unsigned gpio, unsigned mode);
int gpioWrite(unsigned gpio, unsigned level);
int gpioPWM(unsigned gpio, unsigned dutycycle);
int gpioSetAlertFuncEx(unsigned gpio, gpioAlertFuncEx_t callback, void *userdata);
uint32_t gpioDelay(uint32_t micros);
int gpioSleep(unsigned timetype, int seconds, int micros);
int gpioTime(unsigned timetype, int *seconds, int *micros);

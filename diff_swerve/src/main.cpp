#include <cmath>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <math.h>
#include <memory>
#include <pigpio.h>
#include <string.h>
#include <unistd.h>

#include "MiniPID.h"
#include "logger.h"
#include "n20_motor.h"
#include "swerve_module.h"
#include "util.h"
#include "xbox_controller.h"

int main() {
  Logger::setup();
  Logger::logger() << "<Program starting...>" << std::endl;

  // set sample rate of 2 us
  gpioCfgClock(2, 0, -1);
  gpioCfgClock(2, 1, -1);
  if (gpioInitialise() < 0) {
    Logger::logger() << "pigpio failed to initialize";
    return -1;
  }

  // initalize swerve modules
  std::unique_ptr<SwerveModule> frontModule =
      std::make_unique<SwerveModule>(SwerveModuleConstants(
          14, 23, 15, 18,                              // right motor pins
          2, 17, 3, 4,                                 // left motor pins
          PIDConstants(0.01, 0, 0, 0.075, 0.16, 0.25), // motor PID constants
          PIDConstants(10, 0, 0, 0, 5, 5)              // rotation PID constants
          ));
  std::unique_ptr<SwerveModule> rightModule =
    	std::make_unique<SwerveModule>(SwerveModuleConstants(
          12, 16, 20, 21,                              // right motor pins
          24, 25, 8, 7,                                // left motor pins
          PIDConstants(0.01, 0, 0, 0.075, 0.16, 0.25), // motor PID constants
          PIDConstants(5, 0, 0, 0, 5, 5)               // rotation PID constants
          ));
  std::unique_ptr<SwerveModule> leftModule =
      std::make_unique<SwerveModule>(SwerveModuleConstants(
          10, 5, 9, 11,                                // right motor pins
          6, 13, 19, 26,                               // left motor pins
          PIDConstants(0.01, 0, 0, 0.075, 0.21, 0.28), // motor PID constants
          PIDConstants(20, 0, 0, 0, 6, 16)             // rotation PID constants
          ));

  std::vector<std::unique_ptr<SwerveModule>> modules;
  modules.push_back(std::move(frontModule));
  modules.push_back(std::move(rightModule));
  modules.push_back(std::move(leftModule));

  // controller
  XboxController controller(std::string("event2"));
  float inputRight = 0, inputLeft = 0;
  bool wasReady = false;

  // telemetry
  std::deque<float> rpsLValues{};
  std::deque<float> rpsRValues{};

  while (true) {
    // wait for xbox connection
    bool isReady = controller.isReady();
    if (!isReady) {
      Logger::logger() << "waiting for xbox controller connection..."
                       << std::endl;
      gpioSleep(PI_TIME_RELATIVE, 1, 0);
      continue;
    }
    if (!wasReady && isReady) {
      Logger::logger() << "xbox controller is connected" << std::endl;
    }
    wasReady = isReady;

    // input
    float leftY = controller.getLeftY(), leftX = controller.getLeftX();

    // reset position
    if (controller.getButtonPressed() == BTN_TL) {
      for (auto &module : modules) {
        module->resetPosition();
      }
      Logger::logger() << "pressed BTN_TL" << std::endl;
    }

    // rotation control
    float rotationTarget = 0;
    // simple deadband
    if (abs(leftX) > 0.1 || abs(leftY) > 0.1) {
      rotationTarget = (atan2(leftY, leftX) / (2.0 * M_PI)) + .25;
      if (rotationTarget < 0) {
        rotationTarget = 1 + rotationTarget;
      }
      rotationTarget = 1 - rotationTarget;
    } else {
      rotationTarget = 0;
    }

    float targetVel = (sqrt(pow(leftX, 2) + pow(leftY, 2))) * 7.5;
    bool modulesReachedTarget = true;

    float MAX_ROT_ERR = 0.01;

    for (auto &module : modules) {
      float rotError = module->getErrorToTargetRotation(rotationTarget);
      if (abs(rotError) > MAX_ROT_ERR && rotationTarget != 0) {
        modulesReachedTarget = false;
        break;
      }
    }

    for (auto &module : modules) {
      float rotError = module->getErrorToTargetRotation(rotationTarget);
      if (abs(rotError) > MAX_ROT_ERR && rotationTarget != 0) {
        module->setTargetRotation(rotationTarget);
        Logger::logger() << "ROTATING!" << rotationTarget << std::endl;
      } else if (modulesReachedTarget && abs(targetVel) > 1) {
        module->setTargetVelocity(targetVel);
        Logger::logger() << "DRIVIGN!" << targetVel << std::endl;
      } else {
        module->setPercentOut(0);
        Logger::logger() << "STOPPING!" << targetVel << std::endl;
      }

      // TELEMETRY
      rpsLValues.push_back(module->leftMotor.getRps());
      rpsRValues.push_back(module->rightMotor.getRps());

      if (rpsLValues.size() > 50) {
        rpsLValues.pop_front();
      }
      if (rpsRValues.size() > 50) {
        rpsRValues.pop_front();
      }

      std::deque<float>::iterator it;
      float avgRpsL = 0, avgRpsR = 0;
      for (it = rpsLValues.begin(); it != rpsLValues.end(); ++it) {
        avgRpsL += *it;
      }
      for (it = rpsRValues.begin(); it != rpsRValues.end(); ++it) {
        avgRpsR += *it;
      }
      avgRpsL = avgRpsL / rpsLValues.size();
      avgRpsR = avgRpsR / rpsRValues.size();

      // rps control
      Logger::logger() << "avg rps L :" << avgRpsL << std::endl;
      Logger::logger() << "avg rps R :" << avgRpsR << std::endl;
      Logger::logger() << "rps target:" << targetVel << std::endl;

      // rotation control
      Logger::logger() << "rot target:" << rotationTarget << std::endl;
      Logger::logger() << "rot err:" << rotError << std::endl;
      // Logger::logger() << "rot L :" << module->leftMotor.getRotations() <<
      // std::endl; Logger::logger() << "rot R :" <<
      // module->rightMotor.getRotations() << std::endl; Logger::logger() <<
      // "rot:"
      // << module->getRotation() << std::endl; Logger::logger() << "error:" <<
      // rotError << std::endl;
    }

    // TELEMETRY

    // Logger::plotter() << leftMotor.getRps() << " " << targetVel << " " <<
    // rightMotor.getRps() << std::endl;

    //// Logger::logger() << leftY) << ", " << leftX << " | ";
    // Logger::logger() << inputRight << ", " << inputLeft << std::endl;
    // Logger::logger() << rightMotor.getRotations() << " | " <<
    // leftMotor.getRotations() << std::endl; Logger::logger() << moduleRotation
    // << " rotations" << std::endl; Logger::logger() << rotationTarget << " !!"
    // <<  endl; Logger::logger() << finalTarget << " !! TARGET!" <<  std::endl;
    // Logger::logger() << targetVel << " !! TARGET VELOCITY!" <<  std::endl;
    // Logger::logger() << velOutput << " !!~" <<  std::endl;
    // Logger::logger() << atan2(leftY, leftX) / (2.0 * M_PI) << " !!~" <<
    // std::endl;
    //  Logger::logger() << rightMotor.getRps() << " | " << leftMotor.getRps();
    //  Logger::logger() << rightMotor.ticks << " | " << leftMotor.ticks;

    Logger::logger() << "-------------------" << std::endl;

    // TODO: subtract how long system took in the body of the for loop?
    gpioDelay(20000);

    // exit program
    if (controller.getButtonPressed() == BTN_START) {
      break;
    }
  }

  for (auto &module : modules) {
    module->stop();
  }

  gpioTerminate();
}

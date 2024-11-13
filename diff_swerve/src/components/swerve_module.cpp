
#include "n20_motor.h"
#include "swerve_module.h"

SwerveModule::SwerveModule(SwerveModuleConstants moduleConstants) {
  this->rightMotor = N20Motor::N20Motor(moduleConstants.r_en, moduleConstants.r_in, moduleConstants.r_c1, moduleConstants.r_c2);
  this->leftMotor = N20Motor::N20Motor(moduleConstants.l_en, moduleConstants.l_in, moduleConstants.l_c1, moduleConstants.l_c2);

  PIDConstants motorPIDConstants = moduleConstants.motorPIDConstants;
  rightMotor.setPIDF(motorPIDConstants.kP, motorPIDConstants.kI, motorPIDConstants.kD, motorPIDConstants.kF);
  leftMotor.setPIDF(motorPIDConstants.kP, motorPIDConstants.kI, motorPIDConstants.kD, motorPIDConstants.kF);

  PIDConstants rotationPIDConstants = moduleConstants.rotationPIDConstants;
  this->rotationPid = MiniPID::MiniPID(rotationPIDConstants.kP, rotationPIDConstants.kI, rotationPIDConstants.kD, rotationPIDConstants.kF);
}
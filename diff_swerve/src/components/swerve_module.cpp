#include "swerve_module.h"
#include "n20_motor.h"

#include <cmath>

SwerveModule::SwerveModule(SwerveModuleConstants moduleConstants)
    : moduleConstants(moduleConstants),
      rightMotor(moduleConstants.r_en, moduleConstants.r_in,
                 moduleConstants.r_c1, moduleConstants.r_c2),
      leftMotor(moduleConstants.l_en, moduleConstants.l_in,
                moduleConstants.l_c1, moduleConstants.l_c2),
      rotationPid(moduleConstants.rotationPIDConstants.kP,
                  moduleConstants.rotationPIDConstants.kI,
                  moduleConstants.rotationPIDConstants.kD,
                  moduleConstants.rotationPIDConstants.kF) {
  PIDConstants motorPIDConstants = moduleConstants.motorPIDConstants;
  rightMotor.setPIDF(motorPIDConstants.kP, motorPIDConstants.kI,
                     motorPIDConstants.kD, motorPIDConstants.kF,
                     motorPIDConstants.kKineticFriction,
                     motorPIDConstants.kStaticFriction);
  leftMotor.setPIDF(motorPIDConstants.kP, motorPIDConstants.kI,
                    motorPIDConstants.kD, motorPIDConstants.kF,
                    motorPIDConstants.kKineticFriction,
                    motorPIDConstants.kStaticFriction);
  this->rotationPid.setOutputLimits(-7, 7);
}

float SwerveModule::getRotation() const {
  float rightMotorModuleRotations =
      this->rightMotor.getRotations() *
      SwerveModule::MODULE_ROTATION_PER_MOTOR_ROTATION;
  float leftMotorModuleRotations =
      this->leftMotor.getRotations() *
      SwerveModule::MODULE_ROTATION_PER_MOTOR_ROTATION;
  return rightMotorModuleRotations + leftMotorModuleRotations;
}

void SwerveModule::setTargetRotation(float target) {
  float moduleRotation = this->getRotation();
  float closestTarget = this->calculateClosestMatchingRotation(target);
  double velOutput = rotationPid.getOutput(moduleRotation, closestTarget);

  float rotError = abs(moduleRotation - closestTarget);

  // help start moving motors
  if (rotError > 0.01 && (std::fabs(rightMotor.getRps()) < 0.01 ||
                          std::fabs(leftMotor.getRps()) < 0.01)) {
    velOutput += copysign(1.0, velOutput) *
                 this->moduleConstants.rotationPIDConstants.kStaticFriction;
  } else {
    velOutput += copysign(1.0, velOutput) *
                 this->moduleConstants.rotationPIDConstants.kKineticFriction;
  }

  rightMotor.setTargetVelocity(-velOutput);
  leftMotor.setTargetVelocity(-velOutput);
}

void SwerveModule::setTargetVelocity(float target) {
  rightMotor.setTargetVelocity(-target);
  leftMotor.setTargetVelocity(target);
}

void SwerveModule::setPercentOut(float percentOut) {
  rightMotor.setPercentOut(-percentOut);
  leftMotor.setPercentOut(percentOut);
}

float SwerveModule::calculateClosestMatchingRotation(float target) const {
  float current = getRotation();
  float diff = target - current;
  diff -= std::floor(diff + 0.5);
  return current + diff;
}

void SwerveModule::resetPosition() {
  this->rightMotor.setRotations(0);
  this->leftMotor.setRotations(0);
}

float SwerveModule::getErrorToTargetRotation(float target) const {
  return this->getRotation() - this->calculateClosestMatchingRotation(target);
}

void SwerveModule::stop() {
  this->rightMotor.setPercentOut(0);
  this->leftMotor.setPercentOut(0);
}

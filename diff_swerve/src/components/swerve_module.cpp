
#include "n20_motor.h"
#include "swerve_module.h"

SwerveModule::SwerveModule(SwerveModuleConstants moduleConstants) {
  this->moduleConstants = moduleConstants;
  this->rightMotor = N20Motor::N20Motor(moduleConstants.r_en, moduleConstants.r_in, moduleConstants.r_c1, moduleConstants.r_c2);
  this->leftMotor = N20Motor::N20Motor(moduleConstants.l_en, moduleConstants.l_in, moduleConstants.l_c1, moduleConstants.l_c2);

  PIDConstants motorPIDConstants = moduleConstants.motorPIDConstants;
  rightMotor.setPIDF(motorPIDConstants.kP, motorPIDConstants.kI, motorPIDConstants.kD, motorPIDConstants.kF);
  leftMotor.setPIDF(motorPIDConstants.kP, motorPIDConstants.kI, motorPIDConstants.kD, motorPIDConstants.kF);

  PIDConstants rotationPIDConstants = moduleConstants.rotationPIDConstants;
  this->rotationPid = MiniPID::MiniPID(rotationPIDConstants.kP, rotationPIDConstants.kI, rotationPIDConstants.kD, rotationPIDConstants.kF);
	this->rotationPid.setOutputLimits(-7, 7);
}

float SwerveModule::getRotation() {
	float rightMotorModuleRotations = this->rightMotor.getRotations() * SwerveModule::MODULE_ROTATION_PER_MOTOR_ROTATION;
	float leftMotorModuleRotations = this->leftMotor.getRotations() * SwerveModule::MODULE_ROTATION_PER_MOTOR_ROTATION;
	return rightMotorModuleRotations + leftMotorModuleRotations;
}

// TODO: going to fastest angle reverses driving direction as well
void SwerveModule::setTargetRotation(float target) {
    float moduleRotation = this->getRotation();
    float target = this->calculateClosestMatchingRotation(target);
		double velOutput = rotationPid.getOutput(moduleRotation, target);
		velOutput += copysign(1.0, velOutput) * this->moduleConstants.rotationPIDConstants.kS;

		float rotError = abs(moduleRotation - target);

		// help start moving motors
		if (rotError > 0.01 && (rightMotor.rps < 0.01 || leftMotor.rps < 0.01)) {
			velOutput += copysign(1.0, velOutput) * this->moduleConstants.rotationPIDConstants.kStaticFriction;
		}

    rightMotor.setTargetVelocity(-velOutput);
    leftMotor.setTargetVelocity(-velOutput);
}

void SwerveModule::setTargetVelocity(float target) {
  rightMotor.setTargetVelocity(-target);
  leftMotor.setTargetVelocity(target);
}

float SwerveModule::calculateClosestMatchingRotation(float target) {
    float moduleRotation = this->getRotation();
    float whole;
		float relativeRotation = std::modf(moduleRotation, &whole);
		float diff =  target - relativeRotation;
		// force difference to be between [-0.5, 0.5]
		if (diff > 0.5) {
			diff -= 1;
		} else if (diff < -0.5) {
			diff += 1;
		}
		float finalTarget = moduleRotation + diff;
}

void SwerveModule::resetPosition() {
  this->rightMotor.setRotations(0);		
  this->leftMotor.setRotations(0);		
}

float SwerveModule::getErrorToTargetRotation(float target) {
  return this->getRotation() - this->calculateClosestMatchingRotation(target);
}

void SwerveModule::stop() {
	this->rightMotor.setPercentOut(0);
	this->leftMotor.setPercentOut(0);
}
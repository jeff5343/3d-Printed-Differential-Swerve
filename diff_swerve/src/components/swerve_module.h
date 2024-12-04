#ifndef SWERVE_MODULE_
#define SWERVE_MODULE_

#include "MiniPID.h"

#include "n20_motor.h"
#include "util.h"

class SwerveModuleConstants {
  public:
    int r_en, r_in, r_c1, r_c2;
    int l_en, l_in, l_c1, l_c2;
    PIDConstants motorPIDConstants;
    PIDConstants rotationPIDConstants;
    SwerveModuleConstants(
        int r_en, int r_in, int r_c1, int r_c2,
        int l_en, int l_in, int l_c1, int l_c2,
        PIDConstants motorPIDConstants, PIDConstants rotationPIDConstants)
        : r_en(r_en), r_in(r_in), r_c1(r_c1), r_c2(r_c2),
          l_en(l_en), l_in(l_in), l_c1(l_c1), l_c2(l_c2),
          motorPIDConstants(motorPIDConstants), rotationPIDConstants(rotationPIDConstants) {};
    SwerveModuleConstants()
        : r_en(0), r_in(0), r_c1(0), r_c2(0),
          l_en(0), l_in(0), l_c1(0), l_c2(0),
          motorPIDConstants(), rotationPIDConstants() {};
};

class SwerveModule {
  private:
    static constexpr float MODULE_ROTATION_PER_MOTOR_ROTATION = (17.0 / 60.0) / 2.0;

    SwerveModuleConstants moduleConstants;
    MiniPID rotationPid;

    float calculateClosestMatchingRotation(float target);
  public:
    N20Motor rightMotor, leftMotor;

    SwerveModule(SwerveModuleConstants moduleConstants);
    float getRotation();
    void setTargetRotation(float target);
    void setTargetVelocity(float target);
    void resetPosition();
    float getErrorToTargetRotation(float target);
    void stop();
};

#endif

#ifndef SWERVE_MODULE_
#define SWERVE_MODULE_

#include "MiniPID.h"

#include "n20_motor.h"
#include "util.h"

class SwerveModule {
  private:
    N20Motor rightMotor, leftMotor;
    MiniPID rotationPid;
	public:
		SwerveModule(SwerveModuleConstants moduleConstants);
};

class SwerveModuleConstants {
  public:
    int r_en,r_in, r_c1, r_c2; 
    int l_en, l_in, l_c1, l_c2;
    PIDConstants motorPIDConstants;  
    PIDConstants rotationPIDConstants;  
};

#endif

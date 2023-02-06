#pragma once
#include <numbers>

#define FIELD_ORIENTED true


#define TurningMotorGearRatio 7.0/150.0 
#define DrivingMotorGearRatio 1/6.75
#define WheelDiameterInMeters 0.0992


#define O_PI std::numbers::pi
#define gyroOffset (std::numbers::pi/2)*(0_rad)

#define DriveEncoderPosFactor (DrivingMotorGearRatio * std::numbers::pi * WheelDiameterInMeters)
#define DriveEncoderVelocityFactor (DriveEncoderPosFactor / 60.0)
#define TurnEncoderPosFactor (TurningMotorGearRatio * 3.1415926535 * 2.0)
#define TurnEncoderVelocityFactor (TurnEncoderPosFactor / 60.0)

#define GYRO_PORT 13
//Offsets for Absolute Encoders
#define RFZERO (1.761010-1.57079632)
#define RBZERO (3.204486-1.57079632)
#define LFZERO (5.514661-1.57079632)
#define LBZERO (5.787710-1.57079632)

//SwerveModule PID for Drive
#define KDP 1.0
#define KDI 0.0
#define KDD 0.0

//SweverModule PID for Rotate
#define KRP .45
#define KRI 0.0
#define KRD -0.00025

// 25% Power KP = 1.25, KD = 0.3125

//Drivetrain:
// Define values for GetValue Switch Statement
#define L_FRONT 0
#define R_FRONT 1
#define L_BACK 2
#define R_BACK 3
#define ABS_ANGLE 0

// MAX PERCENTAGE CONSTANTS
#define drivePercentage 0.2//0.80
#define rotatePercentage 0.6//0.80
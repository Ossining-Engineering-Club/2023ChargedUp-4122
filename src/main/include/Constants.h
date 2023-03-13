#pragma once
#include <numbers>

#define FIELD_ORIENTED true

//ArmKinematics:
//Arm Segment Lengths
#define lA 0.81
#define lB 0.75
#define lG 0.0
//Arm Position Conversions
#define AlphaConversionFactor ((16.0/(64.0*64.0))*360.0)*1.3036 // change
#define BetaConversionFactor ((18.0/(74.0*64.0))*360.0)*0.8918// change
#define GammaConversionFactor ((18.0/(74.0*64.0))*360.0)*-.05424// change
//Velocity and Slope Scaling
#define KConstant 0.6 //change

//Arm Parameter Offsets
#define AlphaOffset -122.29 //change
#define BetaOffset -13.07 //change
#define GammaOffset 308.7326 //change
//StartingXY of claw
#define StartingX 0.29
#define StartingY 0.26
//PID VALS
//Gamma
#define KGP 1.0
#define KGI 0.0
#define KGD 0.0
//Beta
#define KBP 1.0
#define KBI 0.0
#define KBD 0.0
//Alpha
#define KAP 1.0
#define KAI 0.0
#define KAD 0.0

//Autobalance
#define TipAngle 6.0
#define ApproachAngle 10.0

#define TurningMotorGearRatio 7.0/150.0 
#define DrivingMotorGearRatio 1/6.75
#define WheelDiameterInMeters 0.0992
#define UpperJointConversionFactor (1/310)*360
#define LowerJointConversionFactor (1/213.3)*360
#define Joint1CloseToBatteryCANID 16
#define Joint1AwayFromBatteryCANID 15
#define Joint2CANID 17
#define Joint3CANID 18
#define GripSpinnerCANID 21
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
#define drivePercentage 0.20///0.80//0.70
#define rotatePercentage 0.6//0.80
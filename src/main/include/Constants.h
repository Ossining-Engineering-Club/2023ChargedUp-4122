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
#define GammaConversionFactor ((18.0/(74.0*64.0))*360.0)*0.05424// change
//Velocity and Slope Scaling
#define KConstant 0.1 //change

//Arm Parameter Offsets  ALL ANGLES IN DEG
#define AlphaOffset -122.29 //change
#define BetaOffset -13.07 //change
#define GammaOffset -308.7326 //change
//StartingXY of claw
#define StartingX 0.0
#define StartingY 0.0
//PID VALS
//Gamma
#define KGP 0.02
#define KGI 0.0
#define KGD 0.0
//Beta
#define KBP 0.02
#define KBI 0.0
#define KBD 0.0
//Alpha
#define KAP 0.008
#define KAI 0.0
#define KAD 0.0

#define MedAlpha 20.928478
#define MedBeta -72.240211
#define MedGamma -35.64251

#define StowAlpha 0.476190
#define StowBeta -3.238093
#define StowGamma -1.976190

#define AutoStowAlpha 0.476190
#define AutoStowBeta -1.638093
#define AutoStowGamma -1.976190

#define HIAlpha 35.213959
#define HIBeta -160.423477
#define HIGamma -45.809055

#define FloorAlpha 31.8
#define FloorBeta -8.404788
#define FloorGamma -86.479698

#define PlaceGamma -95.741432

#define ShelfAlpha 8.500027
#define ShelfBeta -98.335808
#define ShelfGamma -178.277390

#define AutoPlaceAlpha 2.0
#define AutoPlaceBeta -232.842621
#define AutoPlaceGamma -83.979454

//Autobalance
#define TipAngle 10.0
#define ApproachAngle 13.0

#define drivePowerMax 1.0
#define drivePowerMin 0.2
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
#define ShortAutoLength 2.03
#define LongAutoLength 3.68
#define ConeStoppingThreshold 10
#define CubeStoppingThreshold 2.25

#define DriveEncoderPosFactor (DrivingMotorGearRatio * std::numbers::pi * WheelDiameterInMeters)
#define DriveEncoderVelocityFactor (DriveEncoderPosFactor / 60.0)
#define TurnEncoderPosFactor (TurningMotorGearRatio * 3.1415926535 * 2.0)
#define TurnEncoderVelocityFactor (TurnEncoderPosFactor / 60.0)

#define GYRO_PORT 13
//Offsets for Absolute Encoders
#define RFZERO (1.761010-1.57079632)-3.1415926535
#define RBZERO (3.204486-1.57079632)-3.1415926535
#define LFZERO (5.514661-1.57079632)-3.1415926535
#define LBZERO (5.787710-1.57079632)-3.1415926535

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
#define drivePercentage 0.80///0.80//0.70
#define rotatePercentage 0.80//0.80
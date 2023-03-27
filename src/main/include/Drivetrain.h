// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <numbers>

// #include <frc/ADXRS450_Gyro.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include "OECPigeonIMU.h"
#include "Constants.h"
#include "SwerveModule.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/Timer.h>


/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
 Drivetrain();

  void Drive(units::velocity::meters_per_second_t xSpeed,
             units::velocity::meters_per_second_t ySpeed, 
             units::angular_velocity::radians_per_second_t rot,
             bool fieldRelative);

  frc::Pose2d SwerveOdometryGetPose();
  OECPigeonIMU gyro{GYRO_PORT};
  void UpdateOdometry();
  void ResetDrive();
  void GoToPose(frc::Pose2d desiredPose,bool fieldRelative, double drivePower, double timeout);
  void GoToPoseRelative(frc::Pose2d desiredPose, bool fieldRelative, double drivePower, double timeout);
  void VisionAdjustTeleop(bool fieldRelative);

  void DriveUntilAngle(double angle);

  bool isFinished = false;
  void Adjust(bool strafe,bool fieldRelative);
  void AutoBallance(bool fieldRelative);

    static constexpr units::meters_per_second_t maxSpeed = 4.441_mps;  // 3 meters per second
    static constexpr units::radians_per_second_t maxTurnRate{9.89};  // 1/2 rotation per second
   
  
  SwerveModule RFMod{5, 6, 11, true, LBZERO, false, false};
  SwerveModule RBMod{8, 7, 12, true,LFZERO, false, false};
  SwerveModule LBMod{1, 2, 9, false, RFZERO, false, false};
  SwerveModule LFMod{3, 4, 10, false, RBZERO, false, false}; 
  // SwerveModule RFMod{1, 2, 9, false, RFZERO, false, false};
  // SwerveModule RBMod{3, 4, 10, false, RBZERO, false, false};
  // SwerveModule LBMod{5, 6, 11, true, LBZERO, false, false};
  // SwerveModule LFMod{8, 7, 12, true,LFZERO, false, false}; 
  units::meter_t CAMERA_HEIGHT = .34_m;
  units::meter_t TARGET_HEIGHT = .6_m;
  units::angle::radian_t CAMERA_PITCH{0};
  double distance = 0.0;
  double avgdistance = 0.0;
  double FramesLost = 0.0;
  double FrameLossConstant = 3.0;
  double driveSpeed = 0.0;
        // SwerveModuleState frontLeft;
        // SwerveModuleState frontRight;
        // SwerveModuleState backLeft;
        // SwerveModuleState backRight;
      // SwerveModule RFMod{1, 2, 9, true, RFZERO, false, false};
	    // SwerveModule LFMod{8, 7, 12, true, LFZERO, false, false};
	    // SwerveModule RBMod{3, 4, 10, false, RBZERO, false, false};
	    // SwerveModule LBMod{5, 6, 11, true, LBZERO, false, false};
  frc::Rotation2d getAngle(); 
  void GoToTarget();
  double  getVisionDistance();
  frc::Rotation2d getPitchRad();
  frc::Rotation2d getRollRad();
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);

  double forwardSpeed;
  double strafeSpeed;
  double rotationSpeed;

  frc::Timer timer;
 private:
 // **********************************************************
  frc2::PIDController strafeSpeedVisionController{.5,0,0};
  frc::Translation2d frontLeftLocation{+0.29845_m,+0.27305_m};
  frc::Translation2d frontRightLocation{+0.29845_m, -0.27305_m};
  frc::Translation2d backLeftLocation{-0.29845_m, +0.27305_m};
  frc::Translation2d backRightLocation{-0.29845_m, -0.27305_m}; 
  const double FowardSide_P_GAIN = 7.5;

  const double FowardSide_I_GAIN = 0.0;
  const double FowardSide_D_GAIN 
  = 0.0;

  const double kP_Rot = 2.5;
  const double kI_Rot = 0.0;
  const double kD_Rot = 0.0;

  frc2::PIDController controllerFowardMovement{FowardSide_P_GAIN, FowardSide_I_GAIN, FowardSide_D_GAIN};
  frc2::PIDController controllerSideMovement{FowardSide_P_GAIN, FowardSide_I_GAIN, FowardSide_D_GAIN};
  frc2::PIDController controllerRotationMovement{kP_Rot, kI_Rot, kD_Rot};



  photonlib::PhotonCamera camera{"limelight"};
  double basespeed = 0.1;


  double smoothingWidth = 90;
  double reading;
 /*SwerveModule RFMod{1, 2, 9, true, RFZERO, false, false};
  SwerveModule RBMod{3, 4, 10, false, RBZERO, false, false};
  SwerveModule LBMod{5, 6, 11, true, LBZERO, false, false};
  SwerveModule LFMod{8, 7, 12, true,LFZERO, false, false}; //privates since they're not referenced in another file - our code references it in robot.cpp, make this change when you get there.
*/
//Declare Gyro
  // frc::ADXRS450_Gyro gyro;
//Change method to be within GyroClass
// Set up Kinematics Array
  frc::SwerveDriveKinematics<4> kinematics{
      frontLeftLocation, 
      frontRightLocation, 
      backLeftLocation,
      backRightLocation
      };

//Set up Odomotery Array
  frc::SwerveDriveOdometry<4> odometry{
      kinematics,
      Drivetrain::getAngle(), 
        {LFMod.GetPosition(), 
        RFMod.GetPosition(),
        LBMod.GetPosition(), 
        RBMod.GetPosition()},frc::Pose2d{0_m,0_m,O_PI*1_rad}
        };
 };

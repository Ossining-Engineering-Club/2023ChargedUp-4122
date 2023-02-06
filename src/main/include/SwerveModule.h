// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
#include <numbers>
#include <rev/CANSparkMax.h>
#include <Rev/CANEncoder.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(int RotatorMotorNo, int DriveMotorNo, int CANCoderId,
               bool ReverseDirection, double AbsEncoderOffsetConstant, bool DriveReverse,
               bool TurnReverse);

  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;

  void SetDesiredState(const frc::SwerveModuleState& state);
  frc2::PIDController turningPIDController{KRP, KRI, KRD};
  frc::SwerveModuleState optimizedState;
  void ResetEncoder();
  double GetCurrentAngle();
  double GetAbsEncoderAngle();

 private:

//Radians per second for angular velocity, notable for the rotational movement
  static constexpr auto ModuleMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto ModuleMaxAngularAcceleration = std::numbers::pi * units::angular_acceleration::radians_per_second_squared_t{1};  // radians per second^2

// Motor Controllers & Encoders
  rev::CANSparkMax DriveMotor;
  rev::CANSparkMax RotatorMotor;

  rev::SparkMaxRelativeEncoder * driveEncoder;
  rev::SparkMaxRelativeEncoder * turningEncoder;

  ctre::phoenix::sensors::CANCoder absEncoder;

// Absolute Encoder Val. + Offsets
  double absSignum;
  double absEncoderOffset;
  double turningEncoderOffset;

// PID Controllers:
//This controler does X 
  frc2::PIDController drivePIDController{1.0, 0, 0};
};

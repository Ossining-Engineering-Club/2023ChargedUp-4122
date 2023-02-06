// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>
#include <string>

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/kinematics/SwerveModuleState.h>

#include "Constants.h"
#include "Drivetrain.h"

class Robot : public frc::TimedRobot
{
public:
  // Comment out if issues
  Robot() : swerveBot()
  {
    dash->init();
  }

  // inits might be useless and causing the issue
  void RobotInit()
  {
    frc::SmartDashboard::PutData("Field", &m_field);
  }

  void AutonomousInit()
  {
    swerveBot.gyro.ResetYaw();
    swerveBot.ResetDrive();
    fieldRelative = FIELD_ORIENTED;
    frc::Pose2d desiredPose = frc::Pose2d(1_m, 2_m, frc::Rotation2d(180_deg));
    swerveBot.GoToPose(desiredPose, fieldRelative);
  }

  void AutonomousPeriodic() override {}
  void TeleopInit() override
  {
    if (isReset == false)
    {
      swerveBot.gyro.ResetYaw();
      swerveBot.ResetDrive();
      isReset = true;
    }
  }

  void TeleopPeriodic() override
  {
    swerveBot.UpdateOdometry();
    m_field.SetRobotPose(swerveBot.SwerveOdometryGetPose());
    // Odometry Values
    frc::SmartDashboard::PutNumber("YPose", swerveBot.SwerveOdometryGetPose().Y().value());
    frc::SmartDashboard::PutNumber("XPose", swerveBot.SwerveOdometryGetPose().X().value());
    frc::SmartDashboard::PutNumber("Heading", swerveBot.SwerveOdometryGetPose().Rotation().Radians().value());
    frc::SmartDashboard::PutNumber("Drivetrain::getAngle", swerveBot.gyro.GetYaw());

    if (controller.GetXButton())
    {
      // swerveBot.Adjust(true,true);
    }
    else
    {
      ControlledDrive(true);
    }
  }

private:
  frc::XboxController controller{0}; // Xbox controller in first port
  frc::SmartDashboard *dash;         // Initialize smart dashboard
  Drivetrain swerveBot;              // Construct drivetrain object
  frc::Field2d m_field;
  bool fieldRelative;
  bool isReset = false;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> xSpeedLimiter{5 / 1_s}; // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> ySpeedLimiter{5 / 1_s}; // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> rotLimiter{5 / 1_s};    // Used to be 3 / 1_s

  void ControlledDrive(bool fieldRelative)
  { // Method to drive robot with Xbox Controller
    const auto xSpeed = xSpeedLimiter.Calculate(
                            frc::ApplyDeadband(controller.GetLeftX(), 0.4)) // Dead band used to be 0.02
                        * Drivetrain::maxSpeed;

    const auto ySpeed = ySpeedLimiter.Calculate(
                            frc::ApplyDeadband(controller.GetLeftY(), 0.4)) // Dead band used to be 0.02
                        * Drivetrain::maxSpeed;

    const auto rot = rotLimiter.Calculate(
                         frc::ApplyDeadband(controller.GetRightX(), 0.4)) // Dead band used to be 0.02
                     * Drivetrain::maxTurnRate;

    swerveBot.Drive(-ySpeed, -xSpeed, -rot, fieldRelative);
    dash->PutNumber("YPose", swerveBot.SwerveOdometryGetPose().Y().value());
    dash->PutNumber("XPose", swerveBot.SwerveOdometryGetPose().X().value());
    dash->PutNumber("Heading", swerveBot.SwerveOdometryGetPose().Rotation().Radians().value());
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif

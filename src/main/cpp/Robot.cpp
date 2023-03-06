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
#include <frc/Joystick.h>
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
    //swerveBot.gyro.ResetYaw();
    swerveBot.ResetDrive();
    fieldRelative = FIELD_ORIENTED;
    // frc::Pose2d Movement1 = frc::Pose2d(1_m, 2_m, frc::Rotation2d(180_deg));
    // swerveBot.GoToPose(Movement1, fieldRelative);
    // frc::Pose2d Movement2 = frc::Pose2d(3_m,0_m,frc::Rotation2d(90_deg));
    // swerveBot.GoToPose(Movement2,fieldRelative);
    // frc::Pose2d Movement3 = frc::Pose2d(1_m,3_m,frc::Rotation2d(0_deg));
    /*
    //0,0,0 (1.92,4.67,0)
    //ideally our starting position we can place without moving but that will remain to be seen
    frc::Pose2d  HomePose frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
    frc::Pose2d  TurningFromHomeOrPickupPose frc::Pose2d(2.58_m, 0_m, frc::Rotation2d(180_deg));
    frc::Pose2d  PickupOnePose frc::Pose2d(4.46_m, 0_m, frc::Rotation2d(0_deg));
    frc::Pose2d  2ndPickupTurningPose frc::Pose2d(0_m, 0_m, frc::Rotation2d(90_deg));
    frc::Pose2d  MovingToSecondPickupPose frc::Pose2d(0_m, -1.28_m, frc::Rotation2d(0_deg));
    frc::Pose2d  TurningAfterSecondPickupPose frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg));
    frc::Pose2d  MovingAwayAfterSecondPickupPose frc::Pose2d(4.46_m, 0_m, frc::Rotation2d(0_deg));
    //arm place
    swerveBot.GoToPose(TurningFromHomeOrPickupPose);
    swerveBot.GoToPose(PickupOnePose);
    //arm pickup
    swerveBot.GoToPose(TurningFromHomeOrPickupPose);
    swerveBot.GoToPose(HomePose);
    //arm place
    swerveBot.GoToPose(TurningFromHomeOrPickupPose);
    swerveBot.GoToPose(PickupOnePose);
    swerveBot.GoToPose(2ndPickupTurningPose);
    swerveBot.GoToPose(MovingToSecondPickupPose);
    //arm pickup
    swerveBot.GoToPose();
    swerveBot.GoToPose();
    swerveBot.GoToPose();
    swerveBot.GoToPose();
    //arm place
    //auto done
    */
  while(swerveBot.gyro.GetRoll()*-1.0 < 13.0){
    swerveBot.Drive(1.0*4.441_mps, 0.0_mps, units::radians_per_second_t{0.0},FIELD_ORIENTED);
    frc::SmartDashboard::PutNumber("roll",swerveBot.gyro.GetRoll()*-1);
  }
  
  while(swerveBot.gyro.GetRoll()*-1.0 > 4.6){
    swerveBot.Drive(1.0*4.441_mps, 0.0_mps, units::radians_per_second_t{0.0},FIELD_ORIENTED);
    frc::SmartDashboard::PutNumber("roll",swerveBot.gyro.GetRoll()*-1);
  }
      swerveBot.ResetDrive();
      frc::Pose2d RampDist = frc::Pose2d(0.0_m,0_m, 0.0_rad*std::numbers::pi);
  while(true){
      swerveBot.GoToPose(RampDist, FIELD_ORIENTED);
  }
      //swerveBot.Drive(0.0*Drivetrain::maxSpeed,0.0*Drivetrain::maxSpeed,0.0*Drivetrain::maxTurnRate,FIELD_ORIENTED);
      
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
   dash->PutNumber("ABSLBPos",swerveBot.LFMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSLBPos",swerveBot.LBMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSRFPos",swerveBot.RFMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSRBPos",swerveBot.RBMod.GetAbsEncoderAngle());
    dash->PutNumber("LFPos",swerveBot.LFMod.GetCurrentAngle());
    dash->PutNumber("LBPos",swerveBot.LBMod.GetCurrentAngle());
    dash->PutNumber("RFPos",swerveBot.RFMod.GetCurrentAngle());
    dash->PutNumber("RBPos",swerveBot.RBMod.GetCurrentAngle());
    dash->PutNumber("Gyro", (swerveBot.getAngle().Degrees().value()));
    frc::SmartDashboard::PutNumber("Heading", swerveBot.SwerveOdometryGetPose().Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Yaw", swerveBot.gyro.GetYaw());
    frc::SmartDashboard::PutNumber("pitch", swerveBot.gyro.GetPitch());
    frc::SmartDashboard::PutNumber("roll",swerveBot.gyro.GetRoll()*-1);

    if (driveController.GetXButton())
    {
      // swerveBot.Adjust(true,true);
    }
    else
    {
      ControlledDrive(FIELD_ORIENTED);
      //swerveBot.Drive(.3*Drivetrain::maxSpeed,0.0*Drivetrain::maxSpeed,0.0*Drivetrain::maxTurnRate,FIELD_ORIENTED);
      ArmControl();
      
    }
  }

private:
  frc::XboxController driveController{0}; // Xbox controller in first port
  frc::Joystick armJoint1Stick{1};
  frc::Joystick armJoint2Stick{2};
  frc::Joystick armJoint3Stick{3};
  frc::SmartDashboard *dash;         // Initialize smart dashboard
  Drivetrain swerveBot;              // Construct drivetrain object
  frc::Field2d m_field;
  bool fieldRelative;
  bool isReset = false;
  rev::CANSparkMax Joint1MotorClosestToBattery{Joint1CloseToBatteryCANID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax Joint1MotorAwayFromBattery{Joint1AwayFromBatteryCANID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax Joint2Motor{Joint2CANID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax Joint3Motor{Joint3CANID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax GripSpinnerMotor{GripSpinnerCANID, rev::CANSparkMax::MotorType::kBrushless};
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> xSpeedLimiter{5 / 1_s}; // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> ySpeedLimiter{5 / 1_s}; // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> rotLimiter{5 / 1_s};    // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> joint1SpeedLimiter{5 / 1_s}; // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> joint2SpeedLimiter{5 / 1_s}; // Used to be 3 / 1_s

  void ControlledDrive(bool fieldRelative)
  { // Method to drive robot with Xbox Controller
    const auto xSpeed = xSpeedLimiter.Calculate(
                            frc::ApplyDeadband(driveController.GetLeftX(), 0.4)) // Dead band used to be 0.02
                        * Drivetrain::maxSpeed;

    const auto ySpeed = ySpeedLimiter.Calculate(
                            frc::ApplyDeadband(driveController.GetLeftY(), 0.4)) // Dead band used to be 0.02
                        * Drivetrain::maxSpeed;

    const auto rot = rotLimiter.Calculate(
                         frc::ApplyDeadband(driveController.GetRightX(), 0.4)) // Dead band used to be 0.02
                     *0.4* Drivetrain::maxTurnRate;

    swerveBot.Drive(-ySpeed, -xSpeed, -rot, fieldRelative);
    dash->PutNumber("YPose", swerveBot.SwerveOdometryGetPose().Y().value());
    dash->PutNumber("XPose", swerveBot.SwerveOdometryGetPose().X().value());
    dash->PutNumber("Heading", swerveBot.SwerveOdometryGetPose().Rotation().Radians().value());
  }
  void ArmControl(){
    //convention is as follows
    //Joint One
    //Motor closest to battery: + = joint up, - = joint down
    //Motor on other side of base away from battery: - = joint up, + = joint down
    //Joint 2
    //Primary motor: - = joint up, + = joint down
    //Joint 3
    //Primary motor: + = joint up, - = joint down
    //On our sticks we want + on the joystick to drive the arm foward, or down
    //Each joint will have a stick assigned to it

    // Close + | Away -
    const auto joint1CloseToBatteryMotorSpeed = armJoint1Stick.GetY()*.3;
    const auto joint1AwayFromBatteryMotorSpeed = -armJoint1Stick.GetY()*.3;

    // Joint 2 + | Joint 3 -
    const auto joint2MotorSpeed = armJoint2Stick.GetY()*.3;

    // Wrist contorl on Joystick #2
    if(armJoint2Stick.GetRawButton(2)){
      Joint3Motor.Set(-0.3);
    }
    else if (armJoint2Stick.GetRawButton(3)){
      Joint3Motor.Set(0.3);
    } else{
      Joint3Motor.Set(0.0);
    }
    
    if(armJoint1Stick.GetRawButton(2)){
      GripSpinnerMotor.Set(0.2);
    } else if(armJoint1Stick.GetRawButton(3)){
      GripSpinnerMotor.Set(-0.85);
    }else{
      GripSpinnerMotor.Set(0.0);
    }

    // const auto joint3MotorSpeed = -armJoint3Stick.GetY()*.3;

    // const auto spinner2MotorSpeed = armJoint3Stick.GetX()*.3;

    dash ->PutNumber("Joint1Speed",joint1CloseToBatteryMotorSpeed);
    dash ->PutNumber("Joint2Speed",joint2MotorSpeed);
    // Commented out after switching to two joystick configurtion
    // dash ->PutNumber("Joint3Speed",joint3MotorSpeed);
    
    Joint1MotorAwayFromBattery.Set(joint1AwayFromBatteryMotorSpeed);
    Joint1MotorClosestToBattery.Set(joint1CloseToBatteryMotorSpeed);
    Joint2Motor.Set(joint2MotorSpeed);
    // Joint3Motor.Set(joint3MotorSpeed);
    
    // GripSpinnerMotor.Set(spinner2MotorSpeed);

  // Third joystick code

    // if(armJoint3Stick.GetRawButton(2)){
    //   GripSpinnerMotor.Set(0.85);
    // } else if(armJoint3Stick.GetRawButton(3)){
    //   GripSpinnerMotor.Set(-0.2);
    // }else{
    //   GripSpinnerMotor.Set(0);
    // }
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif

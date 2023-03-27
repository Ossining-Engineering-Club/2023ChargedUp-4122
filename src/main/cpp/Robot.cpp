// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>
#include <string>
#include "Arm.h"
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/PowerDistribution.h>
#include "cameraserver/CameraServer.h"
#include <frc/DigitalInput.h>

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
    // Creates UsbCamera and MjpegServer [1] and connects them
    frc::CameraServer::StartAutomaticCapture();

    // Creates the CvSink and connects it to the UsbCamera
    cs::CvSink cvSink = frc::CameraServer::GetVideo();

    // Creates the CvSource and MjpegServer [2] and connects them
    cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur", 640, 480);
  }

  void AutonomousInit()
  {
    // swerveBot.gyro.ResetYaw();
    swerveBot.gyro.ResetYaw();
    swerveBot.ResetDrive();
    arm.ResetEncoders();
    isReset = true;
    
    fieldRelative = FIELD_ORIENTED;
    // frc::Pose2d Movement1 = frc::Pose2d(1_m, 2_m, frc::Rotation2d(180_deg));
    // swerveBot.GoToPose(Movement1, fieldRelative);
    // frc::Pose2d Movement2 = frc::Pose2d(3_m,0_m,frc::Rotation2d(90_deg));
    // swerveBot.GoToPose(Movement2,fieldRelative);
    // frc::Pose2d Movement3 = frc::Pose2d(1_m,3_m,frc::Rotation2d(0_deg));

    // 0,0,0 (1.92,4.67,0)
    // ideally our starting position we can place without moving but that will remain to be seen
    frc::Pose2d HomePose = frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg));
    frc::Pose2d TurningFromHomePose = frc::Pose2d(2.58_m, 0_m, frc::Rotation2d(180_deg));
    frc::Pose2d TurningFromPickupPose = frc::Pose2d(-2.58_m, 0_m, frc::Rotation2d(180_deg));

    frc::Pose2d PickupOnePose = frc::Pose2d(-2_m, 0_m, frc::Rotation2d(0_deg));
    frc::Pose2d k2ndPickupTurningPose = frc::Pose2d(3_m, 0_m, frc::Rotation2d(90_deg));
    frc::Pose2d MovingToSecondPickupPose = frc::Pose2d(0_m, -1.28_m, frc::Rotation2d(0_deg));
    frc::Pose2d TurningAfterSecondPickupPose = frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg));
    frc::Pose2d MovingAwayAfterSecondPickupPose = frc::Pose2d(4.46_m, 0_m, frc::Rotation2d(0_deg));

    // swerveBot.GoToPose(PickupOnePose,fieldRelative,.2);
    // swerveBot.GoToPoseRelative(HomePose,fieldRelative,.2);

    // arm place
    // swerveBot.GoToPose(TurningFromHomePose,true);

    // swerveBot.GoToPose(PickupOnePose,true);
    // swerveBot.GoToPose(TurningFromHomePose,true);

    /*
    //arm pickup
    swerveBot.GoToPose(TurningFromPickupPose,true);
    swerveBot.GoToPose(HomePose, true);
    */
    /*
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
    // trive till bridhe is stable
    bool DIOS0 = !DIOSwitch0.Get();
    bool DIOS1 = !DIOSwitch1.Get();
    bool DIOS2 = !DIOSwitch2.Get();
    bool DIOS3 = !DIOSwitch3.Get();
    if (DIOS0 || DIOS1 || DIOS2 || DIOS3)
    {
      if (DIOS3)
      {
        for (int i = 0; i < 99; i++)
        { // Move to high position
          arm.GoTo(HIAlpha, HIBeta, PlaceGamma, 1.0);
          frc::Wait(0.02_s);
        }
        arm.m_clawSpinner.Set(0.2); // Place cube
        frc::Wait(0.5_s);
        arm.m_clawSpinner.Set(0.0); // Stop spinner
        for (int i = 0; i < 74; i++)
        { // Move to stow position
          arm.GoTo(StowAlpha, StowBeta, StowGamma, 1.0);
          frc::Wait(0.02_s);
        }
      }

      if (DIOS0)
      {
        double yawInit = swerveBot.gyro.GetRoll();
        dash->PutString("State", "Approach");
        // while ((swerveBot.gyro.GetRoll() - yawInit) < ApproachAngle)
        // {
        //   swerveBot.Drive(-0.25 * 4.441_mps, 0.0_mps, units::radians_per_second_t{0.0}, FIELD_ORIENTED);//negated 0.25 -> -0.25
        //   frc::SmartDashboard::PutNumber("roll", swerveBot.gyro.GetRoll());
        // }
        // dash->PutString("State", "Tipping");

        // // drive till bridge starts to level
        // while ((swerveBot.gyro.GetRoll() - yawInit) > TipAngle)
        // {
        //   swerveBot.Drive(-0.25 * 4.441_mps, 0.0_mps, units::radians_per_second_t{0.0}, FIELD_ORIENTED);//negated -0.25 -> 0.25
        //   frc::SmartDashboard::PutNumber("roll", swerveBot.gyro.GetRoll());
        // }
        //frc::Pose2d RampDist = frc::Pose2d(0.12_m, 0.0_m, 0_deg); //negated -0.12 -> 0.12
        frc::Pose2d RotateAfter = frc::Pose2d(0.0_m,0.0_m,20_deg); //negated 20_deg -> -20_deg
        //swerveBot.GoToPoseRelative(RampDist, FIELD_ORIENTED, 0.2, 10.0);
        swerveBot.GoToPoseRelative(RotateAfter, FIELD_ORIENTED, 0.2, 5.0);
        //swerveBot.Drive(0.0 * 4.441_mps, 0.0_mps, units::radians_per_second_t{1.0}, FIELD_ORIENTED);
        //frc::Wait(0_s);
        //swerveBot.Drive(0.0 * 4.441_mps, 0.0_mps, units::radians_per_second_t{0.0}, FIELD_ORIENTED); 
        //dash->PutNumber("forwardSpeed",swerveBot.forwardSpeed);
        //dash->PutNumber("strafeSpeed",swerveBot.strafeSpeed);
        //dash->PutNumber("rotationSpeed",swerveBot.rotationSpeed);      
        dash->PutString("State", "Stabilize");
      }

      else if (DIOS1)
      { // short distance
        dash->PutString("State", "Starting Short Drive");
        frc::Pose2d getOutOfCommunityPoseShort = frc::Pose2d(ShortAutoLength * 1.0_m, 0_m, 0.0_rad);
        swerveBot.GoToPoseRelative(getOutOfCommunityPoseShort, fieldRelative, .02, 10.0);
        dash->PutString("State", "Completed Short Drive");
        // frc::Pose2d zeroPose = frc::Pose2d(0_m,0_m,0_deg);
        // swerveBot.GoToPoseRelative(zeroPose, fieldRelative, .02, 5.0);
      }
      else if (DIOS2)
      {
        dash->PutString("State", "Starting Long Drive");
        frc::Pose2d getOutOfCommunityPoseLong = frc::Pose2d(LongAutoLength * 1.0_m, 0_m, 0.0_rad);
        swerveBot.GoToPoseRelative(getOutOfCommunityPoseLong, fieldRelative, .02, 10.0);
        dash->PutString("State", "Completed Long Drive");
        // frc::Pose2d zeroPose = frc::Pose2d(0_m,0_m,0_deg);
        // swerveBot.GoToPoseRelative(zeroPose, fieldRelative, .02, 5.0);
      }
    }

    //   frc::Pose2d RampDist = frc::Pose2d(-2.0_m,0.0_m, 0.0_rad*std::numbers::pi);
    // // while(true){
    //     swerveBot.GoToPose(RampDist, FIELD_ORIENTED, 0.4);
    //     dash->PutString("State", "Done");

    //  }

    //   frc::Pose2d F1 = frc::Pose2d(1.0_m,0.0_m, 0.0_rad*std::numbers::pi);
    //   // frc::Pose2d R1 = frc::Pose2d(1.0_m,-1.0_m, 0.0_rad*std::numbers::pi);
    //   frc::Pose2d B1 = frc::Pose2d(-1.0_m,0.0_m, 0.0_rad*std::numbers::pi);

    // dash->PutString("State", "First");
    // swerveBot.GoToPose(F1, FIELD_ORIENTED, 0.2);
    // //   dash->PutString("State", "Second");

    //   swerveBot.ResetDrive();
    // // swerveBot.GoToPose(R1, FIELD_ORIENTED);
    //     dash->PutString("State", "Third");

    // swerveBot.GoToPose(B1, FIELD_ORIENTED, 0.2);
    //       dash->PutString("State", "Fourth");

    // swerveBot.Drive(0.0*Drivetrain::maxSpeed,0.0*Drivetrain::maxSpeed,0.0*Drivetrain::maxTurnRate,FIELD_ORIENTED);

    // do nothing code
  }

  void AutonomousPeriodic() override {}

  void TeleopInit() override
  {
    if (isReset == false)
    {
      //swerveBot.gyro.ResetYaw();
      //swerveBot.ResetDrive();
      arm.ResetEncoders();
      isReset = true;
    }
    arm.UpdateParameters(); // Needs to be moved into isReset

    fieldRelative = true;
  }

  void TeleopPeriodic() override
  {

    // ouble targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    swerveBot.UpdateOdometry();
    arm.UpdateParameters();
    m_field.SetRobotPose(swerveBot.SwerveOdometryGetPose());
    // arm.SetToPosition(-inverseStick.GetY(),inverseStick.GetX(),0.0,true);
    if (driveController.GetYButton())
    {
      fieldRelative = true;
    }
    else if (driveController.GetXButton())
    {
      fieldRelative = false;
    }
    if (armJoint1Stick.GetRawButtonPressed(6))
    {
      swerveBot.ResetDrive();
    }

    dash->PutBoolean("Field Oriented: ", fieldRelative);
    // dash->PutNumber("limelight x diff", (targetOffsetAngle_Horizontal-5.7));
    ControlledDrive(fieldRelative);

    if (armJoint1Stick.GetRawButton(7))
    {
      arm.ResetEncoders();
    }
    // Zero
    if (armJoint2Stick.GetRawButton(4))
    {
      arm.GoTo(StowAlpha, StowBeta, StowGamma, 1.0);
    }
    // Shelf
    else if (armJoint2Stick.GetRawButton(5))
    {
      arm.GoTo(ShelfAlpha, ShelfBeta, ShelfGamma, 1.0);

    } // Medium Cone and cube
    else if (armJoint2Stick.GetRawButton(7))
    {
      arm.GoTo(MedAlpha, MedBeta, MedGamma, 1.0);
    }
    else if (armJoint2Stick.GetRawButton(6))
    {
      arm.GoTo(HIAlpha, HIBeta, HIGamma, 1.0);
    } // placeGamma
    else if (armJoint2Stick.GetRawButton(8))
    {
      arm.GoTo(FloorAlpha, FloorBeta, FloorGamma, 1.0);
    } // placeGamma
    else if (armJoint1Stick.GetRawButton(4))
    {
      arm.GoTo(arm.e_alpha->GetPosition(), arm.e_beta->GetPosition(), PlaceGamma, 1.0);
    }
    else if (armJoint1Stick.GetRawButtonReleased(4))
    {
      arm.m_clawSpinner.Set(0.2);
      frc::Wait(0.5_s);
      arm.m_clawSpinner.Set(0.0);
    }
    else
    {
      ArmControl();
    }
    // arm.CalculateXY();
    // if(fabs(arm.alpha-arm.alphaNew+arm.beta-arm.betaNew+arm.gamma-arm.gammaNew) < 4 || fabs(inverseStick.GetY()-y1+inverseStick.GetX()-x1) > .2){
    //   y1 = inverseStick.GetY();
    //   x1 = inverseStick.GetX();
    //   arm.UpdateXY(y1, -x1);

    // }
    // //arm.UpdateXY(-inverseStick.GetY(),inverseStick.GetX());
    // arm.InverseKinematics(0.0);
    // m_field.SetRobotPose(swerveBot.SwerveOdometryGetPose());
    // // Odometry Values
    // if(inverseStick.GetRawButton(2)){
    //  arm.SetToPosition(x1,y1,0.0,true);
    // }

    dash->PutBoolean("Gripped", isGripped);
    dash->PutBoolean("Overcurrent", isOverCurrent);
    dash->PutNumber("overcurrent count", OverCurrentCount);
    frc::SmartDashboard::PutNumber("Heading", swerveBot.SwerveOdometryGetPose().Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Yaw", swerveBot.gyro.GetYaw());
    frc::SmartDashboard::PutNumber("pitch", swerveBot.gyro.GetPitch());
    frc::SmartDashboard::PutNumber("roll", swerveBot.gyro.GetRoll());

    dash->PutNumber("alpha", arm.alpha);
    dash->PutNumber("beta", arm.beta);
    dash->PutNumber("gamma", arm.gamma);

    dash->PutNumber("alpha inverse angle", arm.alphaNew);
    dash->PutNumber("beta inverse angle", arm.betaNew);
    dash->PutNumber("gamma inverse angle", arm.gammaNew);

    // arm.SetToPosition(armJoint1Stick.GetX(),armJoint1Stick.GetY(),0.0,true);
    // swerveBot.Drive(.3*Drivetrain::maxSpeed,0.0*Drivetrain::maxSpeed,0.0*Drivetrain::maxTurnRate,FIELD_ORIENTED);
  }

private:
  frc::XboxController driveController{0}; // Xbox controller in first port
  frc::Joystick armJoint1Stick{1};
  frc::Joystick armJoint2Stick{2};
  frc::PowerDistribution PDHObj{1, frc::PowerDistribution::ModuleType::kRev};
  frc::DigitalInput DIOSwitch0{0};
  frc::DigitalInput DIOSwitch1{1};
  frc::DigitalInput DIOSwitch2{2};
  frc::DigitalInput DIOSwitch3{3};

  // frc::Joystick inverseStick{4};
  frc::SmartDashboard *dash; // Initialize smart dashboard
  Drivetrain swerveBot;      // Construct drivetrain object
  frc::Field2d m_field;
  bool isGripped = false;
  bool isOverCurrent = false;
  int OverCurrentCount = 0.0;
  int IntakeCount = 0;
  Arm arm{Joint1CloseToBatteryCANID, Joint1AwayFromBatteryCANID, Joint2CANID, Joint3CANID, GripSpinnerCANID};
  bool fieldRelative;
  bool isReset = false;
  double x1 = StartingX;
  double y1 = StartingY;

  double Current[20] = {0.0};
  // rev::CANSparkMax Joint1MotorClosestToBattery{Joint1CloseToBatteryCANID, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax Joint1MotorAwayFromBattery{Joint1AwayFromBatteryCANID, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax Joint2Motor{Joint2CANID, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax Joint3Motor{Joint3CANID, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax GripSpinnerMotor{GripSpinnerCANID, rev::CANSparkMax::MotorType::kBrushless};
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> xSpeedLimiter{5 / 1_s};      // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> ySpeedLimiter{5 / 1_s};      // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> rotLimiter{5 / 1_s};         // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> joint1SpeedLimiter{5 / 1_s}; // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> joint2SpeedLimiter{5 / 1_s}; // Used to be 3 / 1_s
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

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
                     * Drivetrain::maxTurnRate;
    if (driveController.GetAButton())
    {
      swerveBot.Drive(-ySpeed * (drivePowerMin / drivePercentage), -xSpeed * (drivePowerMin / drivePercentage), -rot * (drivePowerMin / drivePercentage), fieldRelative);
    }
    else
    {
      swerveBot.Drive(-ySpeed * (drivePowerMax / drivePercentage), -xSpeed * (drivePowerMax / drivePercentage), -rot * (drivePowerMax / drivePercentage), fieldRelative);
    }

    dash->PutNumber("YPose", swerveBot.SwerveOdometryGetPose().Y().value());
    dash->PutNumber("XPose", swerveBot.SwerveOdometryGetPose().X().value());
    dash->PutNumber("Heading", swerveBot.SwerveOdometryGetPose().Rotation().Radians().value());
  }
  void ArmControl()
  {
    // convention is as follows
    // Joint One
    // Motor closest to battery: + = joint up, - = joint down
    // Motor on other side of base away from battery: - = joint up, + = joint down
    // Joint 2
    // Primary motor: - = joint up, + = joint down
    // Joint 3
    // Primary motor: + = joint up, - = joint down
    // On our sticks we want + on the joystick to drive the arm foward, or down
    // Each joint will have a stick assigned to it

    // Close + | Away -
    const auto joint1CloseToBatteryMotorSpeed = armJoint1Stick.GetY() * .3;
    const auto joint1AwayFromBatteryMotorSpeed = -armJoint1Stick.GetY() * .3;

    // Joint 2 + | Joint 3 -
    const auto joint2MotorSpeed = frc::ApplyDeadband(armJoint2Stick.GetY(), .1);
    if (armJoint1Stick.GetRawButton(2))
    {
      arm.m_gammaMotor.Set(-0.3);
    }
    else if (armJoint1Stick.GetRawButton(3))
    {
      arm.m_gammaMotor.Set(0.3);
    }
    else
    {
      arm.m_gammaMotor.Set(0.0);
    }
    /*
    if(armJoint1Stick.GetRawButton(2)){
      arm.m_clawSpinner.Set(0.2);
    } else if(armJoint1Stick.GetRawButton(3)){
      arm.m_clawSpinner.Set(-0.85);
    }else{
      arm.m_clawSpinner.Set(0.0);
    }
  */
    arm.m_alphaMotor1.Set(joint1AwayFromBatteryMotorSpeed);
    arm.m_alphaMotor2.Set(joint1CloseToBatteryMotorSpeed);
    arm.m_betaMotor.Set(joint2MotorSpeed);
    // if(!armJoint2Stick.GetRawButton(5) && !armJoint2Stick.GetRawButton(4)){
    //   arm.m_alphaMotor1.Set(joint1AwayFromBatteryMotorSpeed);
    //   arm.m_alphaMotor2.Set(joint1CloseToBatteryMotorSpeed);
    //   arm.m_betaMotor.Set(joint2MotorSpeed);
    // }
    // else if(armJoint2Stick.GetRawButton(4)){
    //     //arm.GoToStowed();
    //   }else if(armJoint2Stick.GetRawButton(5)){
    //     arm.GoToShelf();
    //   }

    // // Wrist contorl on Joystick #2
    // if(armJoint2Stick.GetRawButton(2)){
    //   Joint3Motor.Set(-0.3);
    // }
    // else if (armJoint2Stick.GetRawButton(3)){
    //   Joint3Motor.Set(0.3);
    // } else{
    //   Joint3Motor.Set(0.0);
    // }
    if (PDHObj.GetCurrent(11) > ConeStoppingThreshold && IntakeCount > 15 && armJoint2Stick.GetTrigger())
    {
      isOverCurrent = true;
      isGripped = true;
    }
    if (PDHObj.GetCurrent(11) > CubeStoppingThreshold && IntakeCount > 15 && armJoint2Stick.GetRawButton(3))
    {
      isOverCurrent = true;
      isGripped = true;
    }

    if (armJoint2Stick.GetRawButton(2))
    { // Intake Out
      arm.m_clawSpinner.Set(0.2);
      isGripped = false;
    }
    else if (armJoint2Stick.GetRawButton(3))
    { // Cube In
      IntakeCount++;
      if (!isOverCurrent)
      {
        arm.m_clawSpinner.Set(-0.30);
      }
      else
      {
        arm.m_clawSpinner.Set(-0.01);
      }
    }
    else if (armJoint2Stick.GetTrigger())
    { // Cone In
      IntakeCount++;
      if (!isOverCurrent)
      {
        arm.m_clawSpinner.Set(-0.70);
      }
      else
      {
        arm.m_clawSpinner.Set(-0.01);
      }
    }
    else if (armJoint2Stick.GetTriggerReleased() || armJoint2Stick.GetRawButtonReleased(3))
    {
      isOverCurrent = false;
      OverCurrentCount = 0;
      IntakeCount = 0;
    }
    else
    {
      arm.m_clawSpinner.Set(-0.01);
    }

    // const auto joint3MotorSpeed = -armJoint3Stick.GetY()*.3;

    // const auto spinner2MotorSpeed = armJoint3Stick.GetX()*.3;

    dash->PutNumber("Joint1Speed", joint1CloseToBatteryMotorSpeed);
    dash->PutNumber("Joint2Speed", joint2MotorSpeed);
    // Commented out after switching to two joystick configurtion
    // dash ->PutNumber("Joint3Speed",joint3MotorSpeed);

    // Joint1MotorAwayFromBattery.Set(joint1AwayFromBatteryMotorSpeed);
    // Joint1MotorClosestToBattery.Set(joint1CloseToBatteryMotorSpeed);
    // Joint2Motor.Set(joint2MotorSpeed);
    // // Joint3Motor.Set(joint3MotorSpeed);

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

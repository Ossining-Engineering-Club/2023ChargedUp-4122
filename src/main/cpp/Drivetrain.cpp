// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

Drivetrain::Drivetrain()
{  
  // gyro.BootCalibrate();
  gyro.ResetYaw();
}

// Default drive method: takes the foward, lateral, and rotational speed and calculates the module states in either Field Relative or Robot Relative (depending on last parameter)
void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative)
{
  auto states = kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, Drivetrain::getAngle())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kinematics.DesaturateWheelSpeeds(&states, maxSpeed);

  auto [fl, fr, bl, br] = states;

  // Sets the modules to the desired states while driving
  LFMod.SetDesiredState(fl);
  RFMod.SetDesiredState(fr);
  LBMod.SetDesiredState(bl);
  RBMod.SetDesiredState(br);

} // Drive

void Drivetrain::UpdateOdometry()
{
  odometry.Update(Drivetrain::getAngle(),
                  {LFMod.GetPosition(),
                   RFMod.GetPosition(),
                   LBMod.GetPosition(),
                   RBMod.GetPosition()});
} // Update Odometry

// Method for autonomous, takes in a given pose and drives until it reaches that pose
void Drivetrain::GoToPose(frc::Pose2d desiredPose, bool fieldRelative, double drivePower)
{
  isFinished = false;
  Drivetrain::UpdateOdometry();
  while (isFinished == false)
  {Drivetrain::UpdateOdometry();
    if (fabs(Drivetrain::SwerveOdometryGetPose().X().value() - desiredPose.X().value()) > 0.01 ||
        fabs(Drivetrain::SwerveOdometryGetPose().Y().value() - desiredPose.Y().value()) > 0.01 ||
        fabs(Drivetrain::SwerveOdometryGetPose().Rotation().Radians().value() - desiredPose.Rotation().Radians().value()) > .02)
    {

      
      fowardSpeed = (drivePower/drivePercentage) * controllerFowardMovement.Calculate(Drivetrain::SwerveOdometryGetPose().X().value(), desiredPose.X().value());
      strafeSpeed = (drivePower/drivePercentage) * controllerSideMovement.Calculate(Drivetrain::SwerveOdometryGetPose().Y().value(), desiredPose.Y().value());
      rotationSpeed = (drivePower/drivePercentage) * controllerRotationMovement.Calculate(Drivetrain::SwerveOdometryGetPose().Rotation().Radians().value(), desiredPose.Rotation().Radians().value());
      Drivetrain::Drive(fowardSpeed * Drivetrain::maxSpeed, strafeSpeed * Drivetrain::maxSpeed, rotationSpeed * Drivetrain::maxTurnRate, fieldRelative);
    }
    else
    {
      Drivetrain::Drive(0.0 * Drivetrain::maxSpeed, 0.0 * Drivetrain::maxSpeed, 0.0 * Drivetrain::maxTurnRate, fieldRelative);
      isFinished = true;
    }
  }
} 

// Relative version of absolute go to pose method - goes to the pose away from the starting position rather than absolute position on the field
void Drivetrain::GoToPoseRelative(frc::Pose2d desiredPose, bool fieldRelative, double drivePower){
  isFinished = false;
  Drivetrain::UpdateOdometry();
  frc::Pose2d initPose = Drivetrain::SwerveOdometryGetPose();
  while (isFinished == false)
  {Drivetrain::UpdateOdometry();
    if (fabs(Drivetrain::SwerveOdometryGetPose().X().value() - (initPose.X().value() + desiredPose.X().value())) > 0.01 ||
        fabs(Drivetrain::SwerveOdometryGetPose().Y().value() - (initPose.Y().value() + desiredPose.Y().value())) > 0.01 ||
        fabs(Drivetrain::SwerveOdometryGetPose().Rotation().Radians().value() - (initPose.Rotation().Radians().value() + 
                                                                                 desiredPose.Rotation().Radians().value())) > .02)
    {
      fowardSpeed = (drivePower/drivePercentage) * controllerFowardMovement.Calculate(Drivetrain::SwerveOdometryGetPose().X().value(), desiredPose.X().value());
      strafeSpeed = (drivePower/drivePercentage) * controllerSideMovement.Calculate(Drivetrain::SwerveOdometryGetPose().Y().value(), desiredPose.Y().value());
      rotationSpeed = (drivePower/drivePercentage) * controllerRotationMovement.Calculate(Drivetrain::SwerveOdometryGetPose().Rotation().Radians().value(), desiredPose.Rotation().Radians().value());
      Drivetrain::Drive(fowardSpeed * Drivetrain::maxSpeed, strafeSpeed * Drivetrain::maxSpeed, rotationSpeed * Drivetrain::maxTurnRate, fieldRelative);
    }
    else
    {
      Drivetrain::Drive(0.0 * Drivetrain::maxSpeed, 0.0 * Drivetrain::maxSpeed, 0.0 * Drivetrain::maxTurnRate, fieldRelative);
      isFinished = true;
    }
  }

}

void Drivetrain::DriveUntilAngle(double angle){
  


}



void Drivetrain::AutoBallance(bool fieldRelative){
  //Working Logic
  //Based on 7004's code, they have a combined input of yaw and pitch pids so the robot stays straight and stays level
  //We can do a similar thing but adapted to swerve
  //Yaw dictates the rotational movement, keeping it on a setpoint to 0, then pitch is just foward and back on the x axis
  //To implement this I need two PID's, a gyro pitch and yaw reading, and a drive method
  frc2::PIDController yawPID{1.0,0.0,0.0};
  frc2::PIDController pitchPID{1.0,0.0,0.0};
  /*
  double yawSpeed = yawPID.Calculate(Drivetrain::getAngle().Degrees().value(),0);
  double pitchSpeed = pitchPID.Calculate(Drivetrain::getPitchRad().Degrees().value(),0);
  Drivetrain::Drive(pitchSpeed*Drivetrain::maxSpeed,0_mps, yawSpeed*Drivetrain::maxSpeed,fieldRelative);
  */
  }// Go to Pose
// }
/*

void Drivetrain::Adjust(bool strafe,bool fieldRelative){
      photonlib::PhotonPipelineResult result = camera.GetLatestResult();
      if(result.HasTargets())
        while(fabs(result.GetBestTarget().GetYaw())> 5){
          photonlib::PhotonPipelineResult result = camera.GetLatestResult();
          //units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,units::degree_t{result.GetBestTarget().GetPitch()});
            rotationSpeed = rotation.Calculate(result.GetBestTarget().GetYaw(), 0);
            if(strafe)
              Drivetrain::Drive(rotationSpeed*Drivetrain::maxSpeed, 0*Drivetrain::maxSpeed,0*Drivetrain::maxTurnRate, fieldRelative);
            else
              Drivetrain::Drive(0*Drivetrain::maxSpeed, 0*Drivetrain::maxSpeed,rotationSpeed*Drivetrain::maxTurnRate, fieldRelative);

      }
}
void Drivetrain::GoToTarget(){
    Drivetrain::UpdateOdometry();
    frc::Pose2d currentPose = Drivetrain::SwerveOdometryGetPose();
    Drivetrain::getVisionDistance();
    frc::Pose2d desiredPose = frc::Pose2d(avgdistance*1_m*sin(currentPose.Rotation().Radians().value()),avgdistance*1_m*cos(currentPose.Rotation().Radians().value()),currentPose.Rotation());
    Drivetrain::GoToPose(desiredPose, true);
}
double Drivetrain::getVisionDistance(){
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  reading=0.0;
  if(result.HasTargets()){
    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
          units::radian_t{result.GetBestTarget().GetPitch()});
    distance = range.value();
    avgdistance*=0.9;//Find out constants name convention
    avgdistance += 0.1*distance;//Find out constants name convention
    return avgdistance;
    /*
  for(int i = 0; i < smoothingWidth; i++){
   units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
          units::radian_t{result.GetBestTarget().GetPitch()});
    reading += range.value();
  }
  double returnval = reading/smoothingWidth;
    return returnval;


  }
}
*/

// Returns a Pose2d of the robot's position and orientation via odometry
frc::Pose2d Drivetrain::SwerveOdometryGetPose()
{
  return odometry.GetPose();
}

// Methods for returning the yaw, pitch, and roll from the Pigeon in radians

// Returns the yaw of the robot via PigeonIMU gyro in degrees
frc::Rotation2d Drivetrain::getAngle()
{
  units::radian_t yaw{gyro.GetYaw()* ((std::numbers::pi) / (180.0))};
  return frc::Rotation2d(yaw);
  // -> Old return in radians (commented since we switched to degrees)
  // return frc::Rotation2d(((gyro.GetYaw() * ((std::numbers::pi) / (180.0)) * 1_rad)) - gyroOffset); // gyroOffset defined in constants.h
} // Get Yaw in Degrees

// Returns the pitch of the robot via PigeonIMU gyro in radians
frc::Rotation2d Drivetrain::getPitchRad()
{
  return frc::Rotation2d((gyro.GetPitch() * ((std::numbers::pi) / (180.0)) * 1_rad));
} // Get Pitch in Radians

// Returns the roll of the robot via PigeonIMU gyro in radians
frc::Rotation2d Drivetrain::getRollRad()
{
  return frc::Rotation2d((gyro.GetRoll() * ((std::numbers::pi) / (180.0)) * 1_rad));
} // Get Roll in Radians

// Resets all swerve module encoders and robot odometry
void Drivetrain::ResetDrive()
{
  gyro.ResetYaw();
  LFMod.ResetEncoder();
  LBMod.ResetEncoder();
  RFMod.ResetEncoder();
  RBMod.ResetEncoder();
  odometry.ResetPosition(Drivetrain::getAngle(),
                         {LFMod.GetPosition(),
                          RFMod.GetPosition(),
                          LBMod.GetPosition(),
                          RBMod.GetPosition()},
                         odometry.GetPose());

} // Reset Drive

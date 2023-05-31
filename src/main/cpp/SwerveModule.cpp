// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"
//swerve module struct, constructed for each module with the appropriate port numbers and modifying values
/*
params:
    RotatorMotorNo: The port number of the SparkMax rotating the module 
    DriveMotorNo: The port number of the SparkMax driving the module
    CANCoderId: The CAN ID of the Absolute Encoder for the module
    ReverseDirection: Changes the absSignum to 1(true) or -1(false)
    AbsEncoderOffsetConstant: The Angular offset off the Absolute Encoder
    DriveReverse: Inverts the Driving Motor
    TurnReverse: Inverts the Turning Motor
*/
SwerveModule::SwerveModule(int RotatorMotorNo, 
                           int DriveMotorNo, 
                           int CANCoderId, 
                           bool ReverseDirection, 
                           double AbsEncoderOffsetConstant, 
                           bool DriveReverse,
                           bool TurnReverse):
    //Instantiates the Driving and Turning motors
    RotatorMotor(RotatorMotorNo, rev::CANSparkMax::MotorType::kBrushless),
    DriveMotor(DriveMotorNo, rev::CANSparkMax::MotorType::kBrushless),
    //Instantiates the Absolute Encoder
    absEncoder(CANCoderId)
    {
            absEncoderOffset = AbsEncoderOffsetConstant; // Assign Constant from table in initialization
            if(ReverseDirection == true) absSignum = -1.0;
            else absSignum = 1.0;
            
            // Sets the drive and rotator motors reverse if the module is reverse (bool)
            DriveMotor.SetInverted(DriveReverse);
            RotatorMotor.SetInverted(TurnReverse);
            
            // Creating new encoders for each module for turning/driving, getting their readings, 
            //and setting the position and velocity factors (constant file)
            driveEncoder = new rev::SparkMaxRelativeEncoder(DriveMotor.GetEncoder());
            turningEncoder = new rev::SparkMaxRelativeEncoder(RotatorMotor.GetEncoder());
            driveEncoder -> SetPositionConversionFactor(DriveEncoderPosFactor); // Constant
            
            turningEncoder -> SetPositionConversionFactor(TurnEncoderPosFactor); // Constant
            driveEncoder -> SetVelocityConversionFactor(DriveEncoderVelocityFactor); // Constant
            turningEncoder -> SetVelocityConversionFactor(TurnEncoderVelocityFactor); // Constant
            turningPIDController.EnableContinuousInput(
            -1.0 * std::numbers::pi, 1.0*std::numbers::pi);
            SwerveModule::ResetEncoder();


    }


// Get State Method for Odometry
frc::SwerveModuleState SwerveModule::GetState() const {
    //velocity
  return {units::meters_per_second_t{driveEncoder -> GetVelocity()},
          
          units::radian_t{-1.0*(turningEncoder -> GetPosition() - turningEncoderOffset)}
          };
}

// Get Position Method for Odometry
frc::SwerveModulePosition SwerveModule::GetPosition() const{
   
    //position
  return {units::meter_t{driveEncoder -> GetPosition()},
          units::radian_t{-1.0*(turningEncoder -> GetPosition() - turningEncoderOffset)}};
}

// Get the wheel angle from the turning encoder and then subtract the absolute offset of said encoder
double SwerveModule::GetCurrentAngle(){
    return (-1.0*(turningEncoder -> GetPosition() - turningEncoderOffset));
}

// Returns the angles for the designated absolute encoder
double SwerveModule::GetAbsEncoderAngle(){
    double angle = absEncoder.GetAbsolutePosition();
    angle *= (std::numbers::pi / 180.0);
    angle -= absEncoderOffset;
    // Pi Check:
    if(angle < -std::numbers::pi) angle += (2.0 * std::numbers::pi);
    else if(angle > std::numbers::pi) angle -= (2.0 * std::numbers::pi);

    return angle;
}

// Sets the encoder offset to the last recorded state and position to 0.0 for driving and turning encoder
// Doing so allows the swerve to readjust its alignment no matter where the encoder is reset
void SwerveModule::ResetEncoder(){
    turningEncoderOffset = SwerveModule::GetAbsEncoderAngle();
    driveEncoder -> SetPosition(0.0);
    turningEncoder -> SetPosition(0.0);

}
// Set desired state for the swerve
void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
    if(std::abs(state.speed.value()) > 0.01){
        optimizedState = state.Optimize(state, (SwerveModule::GetCurrentAngle()*1_rad));
        
        double turningVal = -1.0*(turningPIDController.Calculate(SwerveModule::GetCurrentAngle(), 
                                                           optimizedState.angle.Radians().value()));

        // double turningVal = -1.0*(turningPIDController.Calculate(SwerveModule::GetCurrentAngle(), 
        //                                                    (state.angle.Radians().value())));
        
        if(turningVal > 1.0) turningVal = 1.0;
        else if(turningVal < -1.0) turningVal = -1.0;


        DriveMotor.Set(1.0 * optimizedState.speed*(1.0/4.441_mps)); //Change to variable later
        RotatorMotor.Set(rotatePercentage * turningVal);
    }
    else{
        DriveMotor.Set(0.0);
        RotatorMotor.Set(0.0);
    }
}
void SwerveModule::DriveTillAngle(double angle){

}
void SwerveModule::Balance(){

}

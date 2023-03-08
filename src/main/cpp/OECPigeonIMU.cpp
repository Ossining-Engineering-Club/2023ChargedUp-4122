#include "OECPigeonIMU.h"


OECPigeonIMU::OECPigeonIMU(int canPort){
    pigeonGyro = new ctre::phoenix::sensors::PigeonIMU(canPort);
}

// Resets the PigeonIMU's gyro to 0.0 degrees
void OECPigeonIMU::ResetYaw(){
    pigeonGyro->SetYaw(0.0);
}

void OECPigeonIMU::BootCalibrate(){
    pigeonGyro->EnterCalibrationMode(ctre::phoenix::sensors::PigeonIMU::CalibrationMode::BootTareGyroAccel, 6000);
}


//Useless
// void OECPigeonIMU::Reset(){
//     pigeonGyro->SetYaw(0.0);
//     pigeonGyro.Set
//     pigeonGyro->SetPitch(0.0);
// }

//Yaw, Pitch, and Roll are returned in units of degrees
// "Moving counter-clockwise is interpreted as a positive change." 

// Returns the PigeonIMU's yaw in degrees
double OECPigeonIMU::GetYaw(){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
        return ypr[0];
}

// Returns the PigeonIMU's pitch in degrees
double OECPigeonIMU::GetPitch(){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
        return ypr[1];
}

// Returns the PigeonIMU's roll in degrees
double OECPigeonIMU::GetRoll(){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
        return ypr[2];
}
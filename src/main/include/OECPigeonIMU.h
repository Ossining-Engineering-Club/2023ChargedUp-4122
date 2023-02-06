#pragma once
#include <ctre/Phoenix.h>

class OECPigeonIMU{
    public:
        OECPigeonIMU(int canPort);
        void ResetYaw();
        double GetYaw();
        double GetPitch();
        double GetRoll();
        
    private:
        ctre::phoenix::sensors::PigeonIMU *pigeonGyro;
        
};

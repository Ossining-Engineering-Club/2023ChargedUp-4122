#pragma once
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include "Constants.h"

class Arm{

    public:
    Arm(int alphaMotor1, int alphamotor2, int betaMotor, int gammaMotor, int clawSpinner);
    void GetHandCoordinates();
    double* ProcessInputs(double x, double y);
    void SetToPosition(double X, double Y, double clawAngle,bool stick);
    void UpdateXY(double stickX, double stickY);
    void UpdateParameters();
        double alpha = 0.0;
    double beta = 0.0;
    double gamma = 0.0;
    void InverseKinematics(double clawAngle);
    void SetClawSpinner(double power);
    void CalculateXY();
    double x = 0.0;//StartingX;
    double y = 0.0;//StartingY;
     rev::CANSparkMax m_alphaMotor1;
    rev::CANSparkMax m_alphaMotor2;
    rev::CANSparkMax m_betaMotor;
    rev::CANSparkMax m_gammaMotor;
    rev::CANSparkMax m_clawSpinner;
    rev::SparkMaxRelativeEncoder * e_alpha;
    rev::SparkMaxRelativeEncoder * e_beta;
    rev::SparkMaxRelativeEncoder * e_gamma;

    private:
   
    // double alpha = 0.0;
    // double beta = 0.0;
    // double gamma = 0.0;
    double mu = 0.0;
    frc2::PIDController pid_alpha{KAP, KAI, KAD};
    frc2::PIDController pid_beta{KBP, KBI, KBD};
    frc2::PIDController pid_gamma{KGP, KGI, KGD};


};
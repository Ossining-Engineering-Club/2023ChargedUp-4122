#include "Arm.h"
#include "Constants.h"
#include <cmath>
#include <math.h> 

//All Calculations Done in Degrees!!!
Arm::Arm(int alphaMotor1, int alphaMotor2, 
         int betaMotor, int gammaMotor, int clawSpinner):
    m_alphaMotor1(alphaMotor1, rev::CANSparkMax::MotorType::kBrushless),
    m_alphaMotor2(alphaMotor2, rev::CANSparkMax::MotorType::kBrushless),
    m_betaMotor(betaMotor, rev::CANSparkMax::MotorType::kBrushless),
    m_gammaMotor(gammaMotor, rev::CANSparkMax::MotorType::kBrushless),
    m_clawSpinner(clawSpinner, rev::CANSparkMax::MotorType::kBrushless){

    //Spark Max Encoder defenitions
    e_alpha = new rev::SparkMaxRelativeEncoder(m_alphaMotor1.GetEncoder());
    e_beta = new rev::SparkMaxRelativeEncoder(m_betaMotor.GetEncoder());
    e_gamma = new rev::SparkMaxRelativeEncoder(m_gammaMotor.GetEncoder());
    //Encoder Position Factors, seen in constants file(factor states amount of rotations per rotation of motor)
    e_alpha -> SetPositionConversionFactor(AlphaConversionFactor);
    e_beta -> SetPositionConversionFactor(BetaConversionFactor);
    e_gamma -> SetPositionConversionFactor(GammaConversionFactor);
    //Zeroing
    e_alpha -> SetPosition(0.0);
    e_beta -> SetPosition(0.0);
    e_gamma -> SetPosition(0.0);
}
//output {velocity,slope}
double* Arm::ProcessInputs(double stickX, double stickY){
    double velocity = 0.0;
    double slope = 0.0;
    //Distance formula for velocity
    velocity = sqrt(pow(stickX,2.0)+pow(stickY,2.0))*MaxVelocity;
    slope = (stickY/stickX)*MaxSlope;
    double returnArr[2] ={velocity,slope};
    return returnArr;
}
void Arm::UpdateParameters(){
    Arm::alpha = (e_alpha -> GetPosition())- AlphaOffset; 
    Arm::beta = (e_beta -> GetPosition()) - BetaOffset; 
    Arm::gamma = (e_gamma -> GetPosition()) - GammaOffset; 
}
void Arm::UpdateXY(double stickX, double stickY){
    double *inputs = Arm::ProcessInputs(stickX,stickY);
    double velocity = inputs[0];
    double slope = inputs[1];
    double X = 0.0;
    double Y = 0.0;
    //Add math do do such using X and Y

    //set final values
    //Arm::x = X;
    //Arm::y = Y;
}
void Arm::InverseKinematics(){
    //Use x values to set new alpha
    
    //use calculate mu beta and gamma using alpha x and y

}
//If stick is true sets x and y based on input otherwise x y coordinates are used
void Arm::SetToPosition(double X, double Y, bool stick){
    Arm::UpdateParameters();
    if(stick == true){
        UpdateXY(X,Y);
    }else{
        Arm::x = X;
        Arm::y = Y;
    }
    Arm::InverseKinematics();
    m_alphaMotor1.Set(pid_alpha.Calculate((e_alpha -> GetPosition())- AlphaOffset,Arm::alpha));
    m_alphaMotor2.Set(-1.0*pid_alpha.Calculate((e_alpha -> GetPosition())- AlphaOffset,Arm::alpha));
    m_betaMotor.Set(pid_beta.Calculate((e_beta -> GetPosition()) - BetaOffset,Arm::beta));
    m_gammaMotor.Set(pid_gamma.Calculate((e_gamma -> GetPosition()) - GammaOffset,Arm::gamma));

}
void Arm::SetClawSpinner(double power){
   m_clawSpinner.Set(power); 
}

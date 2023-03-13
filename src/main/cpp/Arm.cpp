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
    //InvertEncoders
    // e_alpha -> SetInverted(true);
    // e_beta -> SetInverted(true);
    // e_gamma -> SetInverted(true);
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
    velocity = sqrt(pow(stickX,2.0)+pow(stickY,2.0))*KConstant;
    slope = (stickY/stickX);
    double returnArr[2] ={velocity,slope};
    return returnArr;
}
void Arm::UpdateParameters(){
    Arm::alpha = -1.0*(e_alpha -> GetPosition())- AlphaOffset; 
    Arm::beta = -1.0*(e_beta -> GetPosition()) - BetaOffset; 
    Arm::gamma = -1.0*(e_gamma -> GetPosition()) - GammaOffset; 
}
void Arm::UpdateXY(double stickX, double stickY){
    double *inputs = Arm::ProcessInputs(stickX,stickY);
    double velocity = inputs[0];
    double slope = inputs[1];
    //Add math do do such using X and Y
    Arm::CalculateXY();
}
void Arm::CalculateXY(){
    x = (lA*cos(Arm::alpha*(O_PI/180)) - lB*cos(Arm::alpha*(O_PI/180)+Arm::beta*(O_PI/180)))-StartingX;
    y = (lA*sin(Arm::alpha*(O_PI/180)) - lB*sin(Arm::alpha*(O_PI/180)+Arm::beta*(O_PI/180)))+StartingY;
    // x *= KConstant;
    // y *= KConstant;




}
void Arm::InverseKinematics(double ClawAngle){
    //r value calculation
    double r = sqrt(pow(x,2)+pow(y,2));
    Arm::beta = acos(((pow(r,2)-pow(lA,2)-pow(lB,2))/(-2*lA*lB))*(O_PI/180));
    Arm::alpha = atan((y/x) + acos((pow(lB,2)-pow(r,2)-pow(lA,2))/(-2*r*lA))*(O_PI/180));
    Arm::gamma = (1.5*O_PI) + (ClawAngle*(O_PI/180)) - Arm::alpha - Arm::beta;
    //use calculate mu beta and gamma using alpha x and y

}
//If stick is true sets x and y based on input otherwise x y coordinates are used
void Arm::SetToPosition(double X, double Y, double clawAngle,bool stick){
    Arm::UpdateParameters();
    if(stick == true){
        UpdateXY(X,Y);
    }else{
        Arm::x = X;
        Arm::y = Y;
    }
    Arm::InverseKinematics(clawAngle);
    m_alphaMotor1.Set(pid_alpha.Calculate((O_PI/180)*((e_alpha -> GetPosition())- AlphaOffset),Arm::alpha));
    m_alphaMotor2.Set(-1.0*pid_alpha.Calculate((O_PI/180)*((e_alpha -> GetPosition())- AlphaOffset),Arm::alpha));
    m_betaMotor.Set(pid_beta.Calculate((O_PI/180)*((e_beta -> GetPosition())- BetaOffset),Arm::beta));
    m_gammaMotor.Set(pid_gamma.Calculate((O_PI/180)*((e_gamma -> GetPosition())- GammaOffset),Arm::gamma));

}
void Arm::SetClawSpinner(double power){
   //m_clawSpinner.Set(power); 
}

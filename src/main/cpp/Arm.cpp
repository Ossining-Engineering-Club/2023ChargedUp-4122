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
    // //Encoder Position Factors, seen in constants file(factor states amount of rotations per rotation of motor)
    //  e_alpha -> SetPositionConversionFactor(1.0);
    //  e_beta -> SetPositionConversionFactor();
    //  e_gamma -> SetPositionConversionFactor(GammaConversionFactor);
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
    Arm::alpha = (e_alpha -> GetPosition());//- AlphaOffset; 
    Arm::beta = (e_beta -> GetPosition());// - BetaOffset; 
    Arm::gamma = (e_gamma -> GetPosition());// - GammaOffset; 
}
void Arm::UpdateXY(double stickX, double stickY){

    xnew = x +stickX*KConstant;
    ynew = y+ stickY*KConstant;
    //Add math do do such using X and Y
    //Arm::CalculateXY();
}
void Arm::CalculateXY(){
    x = (lA*cos(Arm::alpha*(O_PI/180.0)) - lB*cos(Arm::alpha*(O_PI/180.0)+Arm::beta*(O_PI/180.0)))+StartingX;
    y = (lA*sin(Arm::alpha*(O_PI/180.0)) - lB*sin(Arm::alpha*(O_PI/180.0)+Arm::beta*(O_PI/180.0)))+StartingY;
    // x *= KConstant;
    // y *= KConstant;




}
void Arm::InverseKinematics(double ClawAngle){
    //r value calculation
    double r = sqrt(xnew*xnew+ynew*ynew);
    Arm::betaNew = acos((r*r-lA*lA-lB*lB)/(-2.0*lA*lB))*(180.0/O_PI);
    Arm::alphaNew = atan(ynew/xnew)*(180.0/O_PI) + acos((lB*lB-r*r-lA*lA)/(-2.0*r*lA))*(180.0/O_PI);
    Arm::gammaNew = (1.5*O_PI) + (ClawAngle*(O_PI/180.0)) - Arm::alpha - Arm::beta;
    //use calculate mu beta and gamma using alpha x and y

}
//If stick is true sets x and y based on input otherwise x y coordinates are used
void Arm::SetToPosition(double X, double Y, double clawAngle,bool stick){
    // Arm::UpdateParameters();
    // Arm::CalculateXY();
    // if(stick == true){
    //     UpdateXY(X,Y);
    // }else{
    //     Arm::x = X;
    //     Arm::y = Y;
    // }
    // Arm::InverseKinematics(clawAngle);
    m_alphaMotor1.Set(pid_alpha.Calculate(Arm::alpha,Arm::alphaNew));
    m_alphaMotor2.Set(-1.0*pid_alpha.Calculate(Arm::alpha,Arm::alphaNew));
    m_betaMotor.Set(pid_beta.Calculate(Arm::beta,Arm::betaNew));
    m_gammaMotor.Set(pid_gamma.Calculate(Arm::gamma,Arm::gammaNew));

}
void Arm::SetClawSpinner(double power){
   //m_clawSpinner.Set(power); 
}


void Arm::ResetEncoders(){
    e_alpha -> SetPosition(0.0);
    e_beta -> SetPosition(0.0);
    e_gamma -> SetPosition(0.0);

}
void Arm::GoTo(double Alpha, double Beta, double Gamma, double multiplier){ // Multiplier is a double from 0.0 to 1.0

    double alphapos = e_alpha -> GetPosition();
    double betapos = e_beta -> GetPosition();
    double gammapos = e_gamma -> GetPosition();

    double Alpha1Speed = pid_alpha.Calculate(alphapos, Alpha);
    double Alpha2Speed = -pid_alpha.Calculate(alphapos, Alpha);
    double BetaSpeed = pid_beta.Calculate(betapos, Beta);
    double GammaSpeed = pid_gamma.Calculate(gammapos,Gamma);

    if(Alpha - alphapos > 5.0){
    m_alphaMotor1.Set(.15*multiplier); // Alpha Forward
    m_alphaMotor2.Set(-.15*multiplier); //Alpha Forward
    }else if(Alpha - alphapos < -5.0){
    m_alphaMotor1.Set(-.25*multiplier); // Alpha Backward
    m_alphaMotor2.Set(.25*multiplier); // Alpha Backward
    }else{
    m_alphaMotor1.Set(Alpha1Speed);
    m_alphaMotor2.Set(Alpha2Speed);
    }
    
    if(Beta - betapos > 5.0){
    m_betaMotor.Set(.7*multiplier); // Beta backward
    }else if(Beta - betapos < -5.0){
    m_betaMotor.Set(-1.0*multiplier); //Beta forward
    }else{
    m_betaMotor.Set(BetaSpeed);
    }

    if(Gamma - gammapos > 10.0){
    m_gammaMotor.Set(.5*multiplier);
    }else if(Gamma - gammapos < -10.0){
    m_gammaMotor.Set(-.5*multiplier);
    }else{
    m_gammaMotor.Set(GammaSpeed);
    }

}
#include "pelviscontroller.h"
#include "Eigen/Dense"
#include <qdebug.h>
#include <vector>
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include"physic.h"
#include"qcgenerator.h"


using namespace  Eigen;

PelvisController::PelvisController()
{

    qDebug()<<"creating pelvis controller";
}

double PelvisController::CPController(double imuAngularVelocity, double imuAnglePitch, double kController, double stepTime){
    double controlInput;
    //double nextPitchAngle;
    //double changeControl;
    //double z_com = 0.981;
    //double omega = qSqrt(GravityAcceleration/z_com);

    //    double cp = estimateCP.CP_IMU(imuAngularVelocity,imuAnglePitch);
    //    double kp = 0;
    //    if(qFabs(qSqrt(cp*cp)-qSqrt(OffsetCP*OffsetCP))>=0.4){
    //        kp = qFabs((cp*cp)-(OffsetCP*OffsetCP))*kController;
    //    }
    //    double kd = 0;
    //    if(qFabs(qSqrt(cp*cp)-qSqrt(OffsetCP*OffsetCP))>=0.4){
    //        kd = qFabs((cp*cp)-(OffsetCP*OffsetCP))*kController/omega;
    //    }

    //double kp = qFabs((cp*cp)-(OffsetCP*OffsetCP))*kController;
    //double kd = qFabs((cp*cp)-(OffsetCP*OffsetCP))*kController/omega;
    //double ed = estimateCP.IMUAngularVelocityFilter(imuAngularVelocity);
    //double e = estimateCP.IMUAngleFilter(imuAnglePitch);
    //changeControl = PIDController(kp,kd,0,e,ed,0);
    //controlInput = PreviousControlInput + changeControl;
    //controlInput = PIDController(kp,kd,0,e,ed,0);
    //controlInput = (-kController*(cp - OffsetCP)*omega*stepTime+PreviousControlInput)/(1+omega*stepTime);

    //nextPitchAngle = estimateCP.AnglePitchPlus(imuAngularVelocity,imuAnglePitch,stepTime);
    //controlInput = OffsetPitch - nextPitchAngle;
    double FilteredAngle = estimateCP.IMUAngleFilter(imuAnglePitch);
    double FilteredAV = estimateCP.IMUAngularVelocityFilter(imuAngularVelocity);

    //if(qFabs(230*(FilteredAngle*M_PI/180-FilteredAV)/(350*0.25))<1)
    //    if((FilteredAngle-OffsetPitch)*FilteredAV<-0.001){
    //        //if(qFabs(FilteredAngle-OffsetPitch)<0.1){
    //            if(FilteredAV>0){
    //                //controlInput = qAsin(250*((FilteredAngle-OffsetPitch)*M_PI/180-FilteredAV)/(350*0.25))*180/M_PI;
    //                controlInput = qAsin(20*(-FilteredAV)/(350*0.25))*180/M_PI;
    //            }
    //        //}
    //    }
    //    if((FilteredAngle-OffsetPitch)*FilteredAV>0.001)
    //        controlInput = qAsin(230*((FilteredAngle-OffsetPitch)*M_PI/180-FilteredAV)/(350*0.25))*180/M_PI;
    //controlInput = qAsin(280*(-FilteredAV)/(350*0.25));

    //        if((FilteredAngle-OffsetPitch)*FilteredAV>=0.001){
    //            controlInput = 0;
    //        }
    //        if((FilteredAngle-OffsetPitch)*FilteredAV<-0.001){
    //            controlInput = PreviousControlInput-(qFabs(FilteredAV)/FilteredAV)*relativeSaturationLimit;
    //        }
    //controlInput = controlInput;
    //qDebug()<<"OffsetPitch:"<<OffsetPitch<<", nextPitchAngle:"<<nextPitchAngle;
//    if((FilteredAngle-OffsetPitch)*FilteredAV<0){
//        //qDebug()<<"hhhhhhhhhhhhhh"<<(FilteredAngle-OffsetPitch)*FilteredAV;
//    }
//    if(qFabs((FilteredAngle-OffsetPitch)*FilteredAV)<0.001){
//        //if(FilteredAV>0){
//            controlInput = qAsin(40*(-FilteredAV)/(350*0.25))*180/M_PI;
//            //qDebug()<<"really"<<(FilteredAngle)<<", "<<(OffsetPitch)<<", "<<imuAnglePitch;
//        //}
//        //qDebug()<<"really"<<(FilteredAngle)<<", "<<(OffsetPitch)<<", "<<imuAnglePitch;
//        //controlInput = 1*(-FilteredAV)*180/M_PI;
//    }

    //**************Controller1***************//
    controlInput = 0;
    if((FilteredAngle-OffsetPitch)*FilteredAV<-0.00005){
        if(FilteredAV>0){
            controlInput = qAsin(20*(-FilteredAV)/(350*0.25))*180/M_PI;
            qDebug()<<"Controller1 is Working";
        }
        if(FilteredAV<0){
            controlInput = qAsin(12*(-FilteredAV)/(350*0.25))*180/M_PI;
            qDebug()<<"Controller2 is Working";
        }
        //controlInput = 1*(-FilteredAV)*180/M_PI;
    }

    if(controlInput>=maxSaturationLimit){
        qDebug()<<"Warning in PelvisController::CPController: Control input is bigger than max saturation limit";
        controlInput = maxSaturationLimit;
    }
    if(controlInput<minSaturationLimit){
        qDebug()<<"Warning in PelvisController::CPController: Control input is smaller than min saturation limit";
        controlInput = minSaturationLimit;
    }
    double delta = controlInput-PreviousControlInput;
    if(delta>=relativeSaturationLimit){
        controlInput = PreviousControlInput + relativeSaturationLimit;
        qDebug()<<"Warning in PelvisController::CPController: Relative change in control input is out of saturation limit";
    }
    if(delta<=-relativeSaturationLimit){
        controlInput = PreviousControlInput - relativeSaturationLimit;
        qDebug()<<"Warning in PelvisController::CPController: Relative change in control input is out of saturation limit";
    }
    //controlInput = PreviousControlInput + (delta>0)?relativeSaturationLimit:-relativeSaturationLimit;
    PreviousControlInput = controlInput;
//    if(qFabs(controlInput)<=0.05)
//        controlInput = 0;
    return controlInput;
}

void PelvisController::SudoTorqueController(double imuAngularVelocity, double imuAnglePitch, double kController, double stepTime, int rightInc, int leftInc, int rightQcOffset, int leftQcOffset){
    double tetaError;
    double FilteredAngle = estimateCP.IMUAngleFilter(imuAnglePitch);
    double FilteredAV = estimateCP.IMUAngularVelocityFilter(imuAngularVelocity);
    double rightAngle = -double(rightInc-rightQcOffset)*2*M_PI/(2304*100);
    double leftAngle = double(leftInc-leftQcOffset)*2*M_PI/(2304*100);
    rightAngle = AnkleAngleCalc.ankle_pitch_reverse(rightAngle,AnkleAngleCalc.ankle_mechanism_offsetR);
    leftAngle = AnkleAngleCalc.ankle_pitch_reverse(leftAngle,AnkleAngleCalc.ankle_mechanism_offsetL);

    double kp = 3.633;
    double ki = 6.853;
    double kd = 0.095;
    double km = 0.064874;
    double N = 100;
    double alpha = km*N;

    //Controller -ktetadot
    tetaError = ((-kController*FilteredAV/alpha)+(kd*PreviousAngleError/stepTime)-(ki*IntegralError))/(kp+kd/stepTime+ki*stepTime);

    //Controller kteta
//    tetaError = ((kController*FilteredAV*(M_PI/180)/alpha)-(-kp/stepTime-2*kd/(stepTime*stepTime))*PreviousAngleError-(kd/(stepTime*stepTime))*PrePreviousAngleError);
//    tetaError = tetaError/((kp/stepTime)+ki+(kd/(stepTime*stepTime)));

    RightSudoTorqueControllerOutput = rightAngle-tetaError;
    LeftSudoTorqueControllerOutput = leftAngle-tetaError;

    PrePreviousAngleError = PreviousAngleError;
    PreviousAngleError = tetaError;
    IntegralError = IntegralError + tetaError*stepTime;
}

void PelvisController::SudoTorqueController2(double imuAngularVelocity, double imuAnglePitch, double kController, double stepTime, int rightInc, int leftInc, int rightQcOffset, int leftQcOffset){
    double FilteredAngle = estimateCP.IMUAngleFilter(imuAnglePitch);
    double FilteredAV = estimateCP.IMUAngularVelocityFilter(imuAngularVelocity);
    double rightAngle = -double(rightInc-rightQcOffset)*2*M_PI/(2304*100);
    double leftAngle = double(leftInc-leftQcOffset)*2*M_PI/(2304*100);
    rightAngle = -AnkleAngleCalc.ankle_pitch_reverse(rightAngle,AnkleAngleCalc.ankle_mechanism_offsetR);
    leftAngle = -AnkleAngleCalc.ankle_pitch_reverse(leftAngle,AnkleAngleCalc.ankle_mechanism_offsetL);

//    qDebug()<<"Right Angle"<<rightAngle;
//    qDebug()<<"Left Angle"<<leftAngle;

    double kp = 3.633;
    double ki = 6.853;
    double kd = 0.095;
    double km = 0.064874;
    double N = 100;
    double alpha = km*N;

    //Controller Motor Damper is different
    double diffDamper = 0;
    double stiffness = 700;
    double tetaErrorRight;
    double tetaErrorLeft;
//    tetaErrorRight = ((-kController*FilteredAV/alpha+stiffness*rightAngle/alpha)+(kd+diffDamper/alpha)*PreviousAngleErrorRight/stepTime-ki*IntegralErrorRight)/(kp+stiffness/alpha+(kd+diffDamper/alpha)/stepTime+ki*stepTime);
//    tetaErrorLeft = ((-kController*FilteredAV/alpha+stiffness*leftAngle/alpha)+(kd+diffDamper/alpha)*PreviousAngleErrorLeft/stepTime-ki*IntegralErrorLeft)/(kp+stiffness/alpha+(kd+diffDamper/alpha)/stepTime+ki*stepTime);

    if(qFabs(FilteredAV-0.1*M_PI/180)>0.1*M_PI/180){
        tetaErrorRight = ((-kController*FilteredAV/alpha-stiffness*rightAngle/alpha)
                          +(kd)*PreviousAngleErrorRight/stepTime-ki*IntegralErrorRight)
                /(kp+stiffness/alpha+(kd+diffDamper/alpha)/stepTime+ki*stepTime)/2;
        tetaErrorLeft = ((-kController*FilteredAV/alpha-stiffness*leftAngle/alpha)
                         +(kd)*PreviousAngleErrorLeft/stepTime-ki*IntegralErrorLeft)
                /(kp+stiffness/alpha+(kd+diffDamper/alpha)/stepTime+ki*stepTime)/2;
    }
    if(qFabs(FilteredAV-0.1*M_PI/180)<=0.1*M_PI/180){
        tetaErrorRight = ((-stiffness*rightAngle/alpha)
                          +(kd)*PreviousAngleErrorRight/stepTime-ki*IntegralErrorRight)
                /(kp+stiffness/alpha+(kd+diffDamper/alpha)/stepTime+ki*stepTime)/2;
        tetaErrorLeft = ((-stiffness*leftAngle/alpha)
                         +(kd)*PreviousAngleErrorLeft/stepTime-ki*IntegralErrorLeft)
                /(kp+stiffness/alpha+(kd+diffDamper/alpha)/stepTime+ki*stepTime)/2;
    }
    //qDebug()<<"Right f4-f5 is"<<(kd)*PreviousAngleErrorRight/stepTime+stiffness*rightAngle/alpha;
    //qDebug()<<"Right f2 is "<<-kController*FilteredAV/alpha<<"Right f3 is"<<-ki*IntegralErrorRight<<"Right f4 is "<<(kd)*PreviousAngleErrorRight/stepTime<<" and Right f5 is"<<stiffness*rightAngle/alpha;

    //qDebug()<<"Left f4-f5 is"<<(kd)*PreviousAngleErrorLeft/stepTime+stiffness*leftAngle/alpha;
    //qDebug()<<"Left f2 is "<<-kController*FilteredAV/alpha<<"Left f3 is "<<-ki*IntegralErrorLeft<<"Left f4 is "<<(kd)*PreviousAngleErrorLeft/stepTime<<" and Left f5 is"<<stiffness*leftAngle/alpha;

    RightSudoTorqueControllerOutput = -rightAngle-tetaErrorRight;
    LeftSudoTorqueControllerOutput = -leftAngle-tetaErrorLeft;

    PreviousAngleErrorLeft = tetaErrorLeft;
    PreviousAngleErrorRight = tetaErrorRight;
    IntegralErrorLeft = IntegralErrorLeft + tetaErrorLeft*stepTime;
    IntegralErrorRight = IntegralErrorRight + tetaErrorRight*stepTime;

}

void PelvisController::CPOffsetFinder(double imuAngularVelocity, double imuAnglePitch)
{
    double cp = estimateCP.CP_IMU(imuAngularVelocity,imuAnglePitch);
    OffsetCP = cp;
}

double PelvisController::PIDController(double kp, double kd, double ki, double e, double ed, double ei)
{
    double result;
    result = kp*e+kd*ed+ki*ei;
    return result;
}

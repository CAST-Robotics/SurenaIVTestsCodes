#ifndef ANKLE_ADAPTATION_H
#define ANKLE_ADAPTATION_H
#include "calc.h"
#include "pidcontroller.h"

class Ankle_adaptation
{
public:
    calc cal;
    int a;
    int b;
    int c;
    int d;
    int e;
    int f;
    int g;
    int h;
    double dt=.005;
    double timer=0;
    double t_end;
    double t_evac=2;
    double t_contact;
    bool leftzstop=false;
    bool rightzstop=false;
    int bump_threshold_min=10;
    int bump_threshold_max=75;
    double AnkleZL_input=.112;
    double AnkleZR_input=.112;
    double AnkleZL_output=.112;
    double AnkleZR_output=.112;
    double AnkleZ_offsetL=0;
    double AnkleZ_offsetR=0;
    double lenghtOfAnkle=.112;
    bool rightGoingDown=false;
    bool leftGoingDown=false;

    double teta_motor_L=0;
    double teta_motor_R=0;//pitch
    double phi_motor_L=0;
    double phi_motor_R=0;//roll

    PIDController teta_PID_L;
    PIDController teta_PID_R;
    PIDController phi_PID_L;
    PIDController phi_PID_R;


    double k1=0.00004;
    double k2=0.00004;
    double k3=0.00004;
    double k4=0.00004;
    double threshold=4;
    double threshold2=95;

    Ankle_adaptation();
    void Ankle_adaptation_update(double z_l, double z_r, int _a, int _b, int _c, int _d, int _e, int _f, int _g, int _h);
    void ankleOrientationAdaptationLeft();
    void ankleOrientationAdaptationRight();
};

#endif // ANKLE_ADAPTATION_H

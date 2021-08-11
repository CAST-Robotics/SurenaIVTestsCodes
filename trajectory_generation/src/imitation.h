#ifndef IMITATION_H
#define IMITATION_H
#include <QList>
#include "ankle_adaptation.h"
#include <qmath.h>
#include <cstring>
#include <math.h>
#include"Eigen/Dense"
#include "Robot.h"
#include <vector>
class imitation
{
public:

    bool mirror=!false;

    double lengthOfThigh=0.3700;
    double lengthOfShank=0.3600;
    double lenghtOfAnkle=0.112;
    double lengthOfHip=0.10900;
    double pelvisLength=0.23;
    double ReferencePelvisHeight=0.91;
    double InitialPelvisHeight=0.9510;


    double GlobalTime=0;
    double t_initial=0;
    double t_foot=0;
    double trhf=0;
    double trhs=0;
    double trhh=0;
    double tlhf=0;
    double tlhs=0;
    double tlhh=0;
    double t_idle=0;
    double t_home=0;
    double idleTimeout=30;




    calc cal;

    int rhID=0;
    int legID=0;
    int lhID=0;



    Ankle_adaptation ankladpt;
    double Ymax=.15;
    double Ypmax=.135;
    double Zmax=.07;
    double Alphamax=10;
    double Rollmodmax=4;
    bool leg_motion_is_done=true;
    int motion_ID=0;
    double X_face=320;
    double Y_face=200;
    int n_people;

    Robot SURENA;

    double RollModified_offline_right;
    double RollModified_offline_left;
    MatrixXd PoseRoot;//position of pelvis respected to global coordinate
    MatrixXd PoseRFoot;//position of right ankle joint respected to global coordinate
    MatrixXd PoseLFoot;//position of left ankle joint respected to global coordinate
    MatrixXd R_P;
    MatrixXd R_F_L;
    MatrixXd R_F_R;

    MatrixXd R_P2;
    MatrixXd R_F_L2;
    MatrixXd R_F_R2;

    double q[34];
    


    VectorXd q_rh;
    VectorXd q_lh;
    VectorXd q_ra;
    VectorXd q_la;

    double head_yaw=0;
    double head_pitch=0;
    double head_roll=0;
    double WaistYaw=0;
    double WaistPitch=0;


    imitation();
    void motion_detect(int code);
    void foot_move(bool side, bool right, bool home, double T);
    void left_right_switch_legs();
    void left_right_switch_hands();
    void foot_move2(bool front2side, bool right, double T);
    void imitationUpdate(int code, bool simulation, int a, int b, int c, int d, int e, int f, int g, int h);
    void imitationInit();
};

#endif // IMITATION_H


#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
#include"MinimumJerkInterpolation.h"
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include <Eigen/Geometry>
#include <cstdlib>
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float32MultiArray.h>
#include<math.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include "qcgenerator.h"
#include "right_hand.h"
#include "left_hand.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <cstdlib>
#include<termios.h>
#include <iostream>
#include <QTime>
#include "trajectory_generation/handmove.h"
#include "trajectory_generation/handmoveRequest.h"
#include "trajectory_generation/handmoveResponse.h"


using namespace  std;
using namespace  Eigen;

bool simulation=true;

bool move_activate=false;

VectorXd q_ra(7);
VectorXd q_la(7);

VectorXd q_rh(7);
VectorXd q_lh(7);



double talkTime=0;

ros::Publisher pub1  ;ros::Publisher pub2  ;ros::Publisher pub3  ;ros::Publisher pub4  ;
ros::Publisher pub5  ;ros::Publisher pub6  ;ros::Publisher pub7  ;ros::Publisher pub8  ;
ros::Publisher pub9  ;ros::Publisher pub10 ;ros::Publisher pub11 ;ros::Publisher pub12 ;
ros::Publisher pub13 ;ros::Publisher pub14 ;ros::Publisher pub15 ;ros::Publisher pub16 ;
ros::Publisher pub17 ;ros::Publisher pub18 ;ros::Publisher pub19 ;ros::Publisher pub20 ;
ros::Publisher pub21 ;ros::Publisher pub22 ;ros::Publisher pub23 ;ros::Publisher pub24 ;
ros::Publisher pub25 ;ros::Publisher pub26 ;ros::Publisher pub27 ;ros::Publisher pub28 ;
ros::Publisher pub29 ;ros::Publisher pub30 ;ros::Publisher pub31 ;
right_hand hand_r;
left_hand hand_l;
// waist
double Waist2ArmZ=0.2694;
double Waist2RArmY=-0.239;
double Waist2LArmY=0.239;
//double WaistYaw=0; double WaistPitch=0;
//MatrixXd R_shoulder(3,3);
//VectorXd r_left_shoulder(3);
//VectorXd r_right_shoulder(3);
//    R_shoulder<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
//            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
//            -sin(WaistPitch),              0,               cos(WaistPitch);
//    r_left_shoulder<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2LArmY*sin(WaistYaw),
//                      Waist2LArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2LArmY,
//                                                               Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;
//    r_right_shoulder<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2RArmY*sin(WaistYaw),
//                       Waist2RArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2RArmY,
//                                                                Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;
std::string scenario_r="null";
std::string scenario_l="null";
std::string scenario_hw="null";
bool headFinished=false;
bool initializing=true;
//head
double X_face=320;
double Y_face=200;
VectorXd r_bottle_cam(3);
VectorXd r_bottle_shoulder(3);
double head_yaw=0;
double head_pitch=0;
double head_roll=0;
double WaistYaw=0;
double WaistPitch=0;
int n_people;
int n;


bool handMove(trajectory_generation::handmoveRequest &req, trajectory_generation::handmoveResponse &res){

    qDebug()<<"req";
    if(!move_activate){
        scenario_l=req.scenario_l;
        scenario_r=req.scenario_r;
        scenario_hw=req.scenario_hw;
        talkTime=req.talkTime;


        initializing=true;
        headFinished=false;
        move_activate=true;
        res.result=1;
        return true;
    }
    else{
        return false;
    }
}

void numplot(double num,double min,double max){

    QString str;
    int l=100;
    int n=int((num-min)/(max-min)*l);
    if (num<min){n=0;}
    if (num>max){n=100;}
    str+=QString::number(min);
    str+="|";
    if (n<=l/2){
        for (int i = 0; i < n; ++i) {
            str+=" ";
        }
        for (int i = 0; i < l/2-n; ++i) {
            str+="|";

        }
        str+="|";
        for (int i = 0; i < l/2; ++i) {
            str+=" ";
        }
    }
    else {
        for (int i = 0; i < l/2; ++i) {
            str+=" ";
        }
        for (int i = 0; i < n-l/2; ++i) {
            str+="|";

        }
        str+="|";
        for (int i = 0; i < l-n; ++i) {
            str+=" ";
        }

    }

    str+="|";
    str+=QString::number(max);
    str+="=>";str+=QString::number(num);
    qDebug()<<str;
    qDebug()<<"";


}

void matrix_view(MatrixXd M){

    for (int i = 0; i <M.rows() ; ++i) {
        QString str;
        for (int j = 0; j <M.cols() ; ++j) {
            str+=QString::number(M(i,j));
            str+="   ";
        }
        qDebug()<<str;
    }
    qDebug()<<"";
}

void matrix_view(VectorXd M){
    QString str;
    for (int i = 0; i <M.rows() ; ++i) {str+=QString::number(M(i));str+="   ";}
    qDebug()<<str;
    qDebug()<<"";
}

double saturate(double a, double min, double max){
    if(a<min){//ROS_INFO("subceeding!");
        return min;}
    else if(a>max){//ROS_INFO("exceeding!");
        return max;}
    else{return a;}
}

double move2pose(double max,double t_local,double T_start ,double T_end){
    double T_move=T_end-T_start;
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_end){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else{theta=max;}
    return theta;
}

double move2zero(double theta,double t,double T_home){
    double c3=10/pow(T_home,3);
    double c4=-15/pow(T_home,4);
    double c5=6/pow(T_home,5);
    double theta0;
    if(t==0){return theta;}
    if(t>=T_home){return 0;}
    double dt=0.005;
    if(fabs(1-c3*pow(t-dt,3)-c4*pow(t-dt,4)-c5*pow(t-dt,5))>1e-6){
        theta0=theta/(1-c3*pow(t-dt,3)-c4*pow(t-dt,4)-c5*pow(t-dt,5));
       // qDebug()<<theta0;
        return theta0*(1-c3*pow(t,3)-c4*pow(t,4)-c5*pow(t,5));
    }


}

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}


MatrixXd rightshoulder2waist(double WaistYaw, double WaistPitch){
    MatrixXd T(4,4);
    MatrixXd R(3,3);
    VectorXd P(3);
    R<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
            -sin(WaistPitch),              0,               cos(WaistPitch);

    P<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2RArmY*sin(WaistYaw),
            Waist2RArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2RArmY,
            Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;

    T<<R.transpose(),-R.transpose()*P,0,0,0,1;

    return T;
}

MatrixXd leftshoulder2waist(double WaistYaw, double WaistPitch){
    MatrixXd T(4,4);
    MatrixXd R(3,3);
    VectorXd P(3);
    R<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
            -sin(WaistPitch),              0,               cos(WaistPitch);

    P<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2LArmY*sin(WaistYaw),
            Waist2LArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2LArmY,
            Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;                                                                1;
    T<<R.transpose(),-R.transpose()*P,0,0,0,1;
    return T;
}


void face_detect(const geometry_msgs::PoseArray & msg){
    n_people=msg.poses.size();

    if(n_people!=0) {
        X_face=msg.poses[0].position.x;
        Y_face=msg.poses[0].position.y;
    }

}

int main(int argc, char **argv)
{

    QByteArray joint_data;

    ros::init(argc, argv, "handnode");
    ros::NodeHandle nh;

    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Float32MultiArray>("trajectory_data_upper",100);
    std_msgs::Float32MultiArray trajectory_data;
    ros::ServiceServer HandMoveService = nh.advertiseService("HandMove", handMove);

    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;
    ros::Subscriber face_sub=nh.subscribe("/ai/face", 1000, face_detect);

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    double t_r_offset=0;double t_l_offset=0;

    right_hand hand_funcs;

    VectorXd r_target_r(3);   VectorXd r_target_l(3);
    VectorXd r_middle_r(3);   VectorXd r_middle_l(3);
    VectorXd r_force_r(3);  VectorXd r_deliver_r(3); VectorXd  r_release_r(3);
    MatrixXd R_target_r(3,3); MatrixXd R_target_l(3,3);
    VectorXd r_target_rt[10];   VectorXd r_target_lt[10];
    MatrixXd R_target_rt[10]; MatrixXd R_target_lt[10];
    for (int i = 0; i < 10; ++i) {
        r_target_rt[i].resize(3,1);
        r_target_lt[i].resize(3,1);
        R_target_rt[i].resize(3,3);
        R_target_lt[i].resize(3,3);
    }
    VectorXd r_right_palm(3);VectorXd r_left_palm(3);
    int fingers_mode_r;       int fingers_mode_l;
    VectorXd q0_r(7);         VectorXd q0_l(7);




    double d0_r;              double d0_l;
    double d_r ;              double d_l ;
    double d_des_r;           double d_des_l;
    double theta_r;           double theta_l;
    double theta_target_r;    double theta_target_l;
    double sai_r;             double sai_l;
    double sai_target_r;      double sai_target_l;
    double phi_r;             double phi_l;
    double phi_target_r;      double phi_target_l;

    MinimumJerkInterpolation coef_generator;
    MatrixXd X_coef_r;        MatrixXd X_coef_l;
    MatrixXd Y_coef_r;        MatrixXd Y_coef_l;
    MatrixXd Z_coef_r;        MatrixXd Z_coef_l;

    MatrixXd t_r(1,3);        MatrixXd t_l(1,3);
    MatrixXd P_x_r(1,3);      MatrixXd P_x_l(1,3);
    MatrixXd V_x_r(1,3);      MatrixXd V_x_l(1,3);
    MatrixXd A_x_r(1,3);      MatrixXd A_x_l(1,3);

    MatrixXd P_y_r(1,3);      MatrixXd P_y_l(1,3);
    MatrixXd V_y_r(1,3);      MatrixXd V_y_l(1,3);
    MatrixXd A_y_r(1,3);      MatrixXd A_y_l(1,3);

    MatrixXd P_z_r(1,3);      MatrixXd P_z_l(1,3);
    MatrixXd V_z_r(1,3);      MatrixXd V_z_l(1,3);
    MatrixXd A_z_r(1,3);      MatrixXd A_z_l(1,3);

    int fingers_r=9;  int fingers_l=9;
 int fingers_r_2counter=0;
int fingers_l_2counter=0;
    q_ra<<13*M_PI/180,
            -10*M_PI/180,
            0,
            -25*M_PI/180,
            0,
            0,
            0;

    q_la<<13*M_PI/180,
            10*M_PI/180,
            0,
            -25*M_PI/180,
            0,
            0,
            0;
    q_lh=q_la;
    q_rh=q_ra;

    int q_motor_r[8];int q_motor_l[8];
    for (int var = 0; var < 8; ++var) {
        q_motor_r[var]=0;q_motor_l[var]=0;
    }

    vector<double> q(32);

    ros::Rate loop_rate(200);
    int count = 0;
    double time=0.0;
    double time_r,time_l;

    int gest_r=0;int gest_l=0;
    int gest_count_r;
    int gest_count_l;
    VectorXd qr_end(7);VectorXd ql_end(7);

    QTime chronometer;
    qDebug()<<"start!";
    while (ros::ok())
    {
        if(move_activate){

            if(initializing){
                qDebug()<<"scenario_l="<<scenario_l.c_str()<<"scenario_r="<<scenario_r.c_str()<<"scenario_hw="<<scenario_hw.c_str();
                q0_r=q_ra;
                q0_l=q_la;

                qDebug("q0 init ok");

//                ROS_INFO("q0_r= %f, %f, %f, %f, %f, %f, %f",q0_r(0),q0_r(1),q0_r(2),q0_r(3),q0_r(4),q0_r(5),q0_r(6));
//                ROS_INFO("q0_l= %f, %f, %f, %f, %f, %f, %f",q0_l(0),q0_l(1),q0_l(2),q0_l(3),q0_l(4),q0_l(5),q0_l(6));

                //self recognition
                if(scenario_r=="handRecog"){

                    r_target_r<<.4,
                            0.04,
                            -0.15;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
                    r_middle_r<<.3,
                            -.1,
                            -.3;
                    fingers_mode_r=10;
                }
                if(scenario_l=="handRecog"){

                    r_target_l<<.4,
                            -0.04,
                            -0.15;
                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
                    r_middle_l<<.3,
                            .1,
                            -.3;
                    fingers_mode_l=10;
                }

 if(scenario_r=="moveFingers"){
     r_target_r<<.4,
             0.04,
             -0.15;
     R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
      r_middle_r<<.4,
              0.04,
              -0.15;
     fingers_mode_r=19;
 }

 if(scenario_l=="moveFingers"){
     r_target_l<<.4,
             -0.04,
             -0.15;
     R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
     r_middle_l<<.4,
             -0.04,
             -0.15;
     fingers_mode_l=19;
 }

                //pointing
                if(scenario_r=="b"){

                    r_target_r<<.05,
                            -0.56,
                            -0;
                    R_target_r=hand_funcs.rot(1,-89*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);
                    r_middle_r<<.1,
                            -.35,
                            -.3;
                    fingers_mode_r=11;
                }
                if(scenario_l=="b"){

                    r_target_l<<.05,
                            0.56,
                            -0;
                    R_target_l=hand_funcs.rot(1,89*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);
                    r_middle_l<<.1,
                            .35,
                            -.3;
                    fingers_mode_l=11;
                }
                // mini touch
                if(scenario_r=="c"){

                    r_target_r<<.4,
                            -0.3,
                            -0.2;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,20*M_PI/180,3);
                    r_middle_r<<.2,
                            -.25,
                            -.35;
                    fingers_mode_r=10;
                }
                if(scenario_l=="c"){

                    r_target_l<<.4,
                            0.3,
                            -0.2;
                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(2,20*M_PI/180,3);
                    r_middle_l<<.2,
                            .25,
                            -.35;
                    fingers_mode_l=10;
                }
                // looking at horizon
                if(scenario_r=="d"){

                    r_target_r<<.27,
                            - 0.15,
                            0.37;
                    // R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(1,-28*M_PI/180,3);
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,30*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,-45*M_PI/180,3);

                    r_middle_r<<.4,
                            -.2,
                            - 0.05;
                    fingers_mode_r=10;

                }
                if(scenario_l=="d"){

                    r_target_l<<.27,
                            0.15,
                            0.37;
                    // R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(1,28*M_PI/180,3);

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-30*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(2,-45*M_PI/180,3);
                    r_middle_l<<.35,
                            .15,
                            -0.05;
                    //                r_middle_l<<.4,
                    //                       .2,
                    //                       -0.05;
                    fingers_mode_l=10;

                }
                // respct
                if(scenario_r=="respect"){

                    r_target_r<<.28,
                            0.1,
                            -0.35;
                    R_target_r=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,50*M_PI/180,3);
                    r_middle_r<<.35,
                            -0.1,
                            -0.3;
                    fingers_mode_r=10;

                }
                if(scenario_l=="respect"){

                    r_target_l<<.28,
                            -0.1,
                            -0.35;
                    R_target_l=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-50*M_PI/180,3);
                    r_middle_l<<.35,
                            0.1,
                            -0.3;
                    fingers_mode_l=10;

                }

                // hand-shake
                if(scenario_r=="shakeHands"){

                    r_target_r<<.4,
                            -0.05,
                            -0.4;
                    R_target_r=hand_funcs.rot(2,-65*M_PI/180,3);
                    r_middle_r<<.3,
                            -0.1,
                            -0.4;
                    fingers_mode_r=18;


                }
                if(scenario_l=="shakeHands"){

                    r_target_l<<.4,
                            0.05,
                            -0.4;
                    R_target_l=hand_funcs.rot(2,-65*M_PI/180,3);
                    r_middle_l<<.3,
                            0.1,
                            -0.4;
                    fingers_mode_l=18;

                }
                // waving
                if(scenario_r=="byebye"){

                    r_target_r<<.35,
                            -0.1,
                            0.3;
                    R_target_r=hand_funcs.rot(2,-180*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3);
                    r_middle_r<<.4,
                            -0.2,
                            -0.2;
                    fingers_mode_r=10;

                }
                if(scenario_l=="byebye"){

                    r_target_l<<.35,
                            0.1,
                            0.3;
                    R_target_l=hand_funcs.rot(2,-180*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3);
                    r_middle_l<<.4,
                            0.2,
                            -0.2;
                    fingers_mode_l=10;

                }
                // gripping
                if(scenario_r=="j"){
                    r_target_r<<.45,
                            0.,
                            -0.25;
                    r_middle_r<<.3,
                            -.0,
                            -.3;

                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3);
                    fingers_mode_r=4;
                }
                if(scenario_l=="j"){
                    r_target_l<<.45,
                            -0.,
                            -0.25;
                    r_middle_l<<.3,
                            .0,
                            -.3;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-(-10*M_PI/180+atan2(r_target_r(1),r_target_r(0))),3);
                    fingers_mode_l=4;
                }

                if(scenario_r=="wh(comeDown)"){
                    r_target_r<<.45,
                            -0.05,
                            -0.35;

                    r_middle_r<<.3,
                            -.2,
                            -.35;

                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(comeDown)"){
                    r_target_l<<.45,
                            0.05,
                            -0.35;

                    r_middle_l<<.3,
                            .2,
                            -.35;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=10;
                }

                if(scenario_r=="wh(force)"){
                    r_target_r<<.45,
                            0.00,
                            -0.35;

                    r_middle_r<<.45,
                            -0.02,
                            -0.35;


                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(force)"){
                    r_target_l<<.45,
                            -0.00,
                            -0.35;

                    r_middle_l<<.45,
                            0.02,
                            -0.35;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=10;
                }

                if(scenario_r=="wh(bringUp)"){
                    r_target_r<<.45,
                            0.00,
                            -0.22;

                    r_middle_r<<.45,
                            0.00,
                            -0.29;


                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(bringUp)"){
                    r_target_l<<.45,
                            -0.00,
                            -0.22;

                    r_middle_l<<.45,
                            -0.00,
                            -0.29;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=10;
                }

                if(scenario_r=="wh(release)"){
                    r_target_r<<.54,
                            -0.03,
                            -0.05;

                    r_middle_r<<.48,
                            -0.00,
                            -0.17;


                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(release)"){
                    r_target_l<<.54,
                            0.03,
                            -0.05;

                    r_middle_l<<.48,
                            0.00,
                            -0.17;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=10;
                }

                if(scenario_r=="wh4"){
                    r_target_r<<.45,
                            0.00,
                            -0.35;

                    r_middle_r<<.45,
                            0.00,
                            -0.31;


                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh4"){
                    r_target_l<<.45,
                            -0.00,
                            -0.35;

                    r_middle_l<<.45,
                            -0.00,
                            -0.31;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=10;
                }
                if(scenario_r=="wh5"){
                    r_target_r<<.45,
                            -0.04,
                            -0.35;

                    r_middle_r<<.45,
                            -0.02,
                            -0.35;


                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh5"){
                    r_target_l<<.45,
                            0.04,
                            -0.35;

                    r_middle_l<<.45,
                            0.02,
                            -0.35;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=10;
                }
                // microphone
                if(scenario_r=="getMic"){
                    r_target_r<<.5,
                            -0.05,
                            -0.2;
                    r_middle_r<<.3,
                            -.15,
                            -.35;

                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3);
                    fingers_mode_r=4;
                }
                if(scenario_l=="getMic"){
                    r_target_l<<.5,
                            0.05,
                            -0.2;
                    r_middle_l<<.3,
                            .15,
                            -.35;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-(-10*M_PI/180+atan2(r_target_r(1),r_target_r(0))),3);
                    fingers_mode_l=4;
                }

                if(scenario_r=="micUp"){
                    r_target_r<<.39,
                            .1,
                            -0.12;
                    r_middle_r<<.35,
                            .05,
                            -.15;

                    R_target_r=hand_funcs.rot(2,-120*M_PI/180,3)*hand_funcs.rot(1,60*M_PI/180,3)*hand_funcs.rot(3,20*M_PI/180,3);
                    fingers_mode_r=4;
                }
                if(scenario_l=="micUp"){
                    r_target_l<<.39,
                            -0.1,
                            -0.12;
                    r_middle_l<<.35,
                            -.05,
                            -.15;

                    R_target_l=hand_funcs.rot(2,-120*M_PI/180,3)*hand_funcs.rot(1,-60*M_PI/180,3)*hand_funcs.rot(3,-20*M_PI/180,3);
                    fingers_mode_l=4;
                }


                if(scenario_r=="giveMicBack"){
                    r_target_r<<.3,
                            -0.3,
                            -0.2;
                    r_middle_r<<.35,
                            -.05,
                            -.15;

                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3);
                    fingers_mode_r=4;

                }
                if(scenario_l=="giveMicBack"){
                    r_target_l<<.3,
                            0.3,
                            -0.2;
                    r_middle_l<<.35,
                            .05,
                            -.15;

                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3);
                    fingers_mode_r=4;

                }
                if(scenario_r=="m5"){
                    r_target_r<<.3,
                            -0.25,
                            -0.2;
                    r_middle_r<<.3,
                            -0.25,
                            -0.2;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3);
                    fingers_mode_r=10;

                }
                if(scenario_l=="m5"){
                    r_target_l<<.3,
                            0.25,
                            -0.2;
                    r_middle_l<<.3,
                            0.25,
                            -0.2;

                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3);
                    fingers_mode_r=10;

                }

                if(scenario_r=="talking"){

                    r_target_rt[0]<<.2,
                            -0.1,
                            -0.4;
                    R_target_rt[0]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-25*M_PI/180,3)*hand_funcs.rot(3,-25*M_PI/180,3);


                    r_target_rt[1]<<.35,
                            -0.1,
                            -0.35;
                    R_target_rt[1]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(3,-70*M_PI/180,3)*hand_funcs.rot(2,-20*M_PI/180,3);

                    r_target_rt[2]<<.4,
                            -0.1,
                            -0.15;
                    R_target_rt[2]=hand_funcs.rot(2,-100*M_PI/180,3)*hand_funcs.rot(3,-40*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);

                    r_target_rt[3]<<.2,
                            -0.1,
                            -0.4;
                    R_target_rt[3]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-25*M_PI/180,3)*hand_funcs.rot(3,-25*M_PI/180,3);

                    r_target_rt[4]<<.1,
                            -0.25,
                            -0.45;
                    R_target_rt[4]=hand_funcs.rot(3,-70*M_PI/180,3)*hand_funcs.rot(2,-50*M_PI/180,3);

                    r_target_rt[5]<<.25,
                            -0.25,
                            -0.35;
                    R_target_rt[5]=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,-50*M_PI/180,3);
                    r_target_rt[6]<<.25,
                            -0.1,
                            -0.45;
                    R_target_rt[6]=hand_funcs.rot(2,-45*M_PI/180,3)*hand_funcs.rot(3,-30*M_PI/180,3);
                    r_target_rt[7]<<.3,
                            0.1,
                            -0.35;
                    R_target_rt[7]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,20*M_PI/180,3);
                    r_target_rt[8]<<.25,
                            -0.2,
                            -0.45;
                    R_target_rt[8]=hand_funcs.rot(2,-30*M_PI/180,3)*hand_funcs.rot(3,-15*M_PI/180,3);
                }

                if(scenario_l=="talking"){
                    r_target_lt[0]<<.2,
                            0.1,
                            -0.4;
                    R_target_lt[0]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,25*M_PI/180,3)*hand_funcs.rot(3,25*M_PI/180,3);


                    r_target_lt[1]<<.35,
                            0.1,
                            -0.35;
                    R_target_lt[1]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(3,70*M_PI/180,3)*hand_funcs.rot(2,-20*M_PI/180,3);

                    r_target_lt[2]<<.4,
                            0.1,
                            -0.15;
                    R_target_lt[2]=hand_funcs.rot(2,-100*M_PI/180,3)*hand_funcs.rot(3,40*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);

                    r_target_lt[3]<<.2,
                            0.1,
                            -0.4;
                    R_target_lt[3]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,25*M_PI/180,3)*hand_funcs.rot(3,25*M_PI/180,3);

                    r_target_lt[4]<<.1,
                            0.25,
                            -0.45;
                    R_target_lt[4]=hand_funcs.rot(3,70*M_PI/180,3)*hand_funcs.rot(2,-50*M_PI/180,3);

                    r_target_lt[5]<<.25,
                            0.25,
                            -0.35;
                    R_target_lt[5]=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,50*M_PI/180,3);

                    r_target_lt[6]<<.25,
                            0.1,
                            -0.45;
                    R_target_lt[6]=hand_funcs.rot(2,-45*M_PI/180,3)*hand_funcs.rot(3,30*M_PI/180,3);
                    r_target_lt[7]<<.3,
                            -0.1,
                            -0.35;
                    R_target_lt[7]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-20*M_PI/180,3);
                    r_target_lt[8]<<.25,
                            0.2,
                            -0.45;
                    R_target_lt[8]=hand_funcs.rot(2,-30*M_PI/180,3)*hand_funcs.rot(3,15*M_PI/180,3);

                }
                //synchronized movement
                if(scenario_r=="s"){
                    r_target_r<<.35,
                            - 0.05,
                            -0.3;

                    r_middle_r<<.2,
                            -.10,
                            -.45;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_r=11;}
                if(scenario_r=="o"){
                    r_target_r<<.4,
                            - 0.1,
                            -0.25;

                    r_middle_r<<.2,
                            -.10,
                            -.45;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_r=11;}

                if(scenario_r=="ws"){
                    r_target_r<<.4,
                            - 0.1,
                            -0.15;

                    r_middle_r<<.2,
                            -.10,
                            -.45;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);

                    fingers_mode_r=11;}
                if(scenario_r=="wu"){
                    r_target_r<<.4,
                            - 0.1,
                            -0.15;

                    r_middle_r<<.2,
                            -.10,
                            -.45;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);

                    fingers_mode_r=11;}
                if(scenario_l=="s"){
                    r_target_l<<.35,
                            0.05,
                            -0.3;
                    r_middle_l<<.2,
                            .1,
                            -.45;
                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=11;
                }
                if(scenario_l=="o"){
                    r_target_l<<.4,
                            0.1,
                            -0.25;
                    r_middle_l<<.2,
                            .1,
                            -.45;
                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=11;
                }

                if(scenario_l=="ws"){
                    r_target_l<<.4,
                            0.1,
                            -0.15;
                    r_middle_l<<.2,
                            .1,
                            -.45;
                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=11;
                }

                if(scenario_l=="wu"){
                    r_target_l<<.4,
                            0.1,
                            -0.15;
                    r_middle_l<<.2,
                            .1,
                            -.45;
                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                    fingers_mode_l=11;
                }



                // temp
                if(scenario_r=="z"){

                    r_target_r<<.25,
                            -0.25,
                            -0.35;
                    R_target_r=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,-50*M_PI/180,3);
                    r_middle_r<<.25,
                            -0.25,
                            -0.35;
                }





                double v0_r=0;
                double v_target_r =.4;
                right_hand hand0_r(q0_r,r_target_r,R_target_r,0,0);
                left_hand hand0_l(q0_l,r_target_l,R_target_l,0,0);
                r_right_palm=hand0_r.r_right_palm;
                r_left_palm=hand0_l.r_left_palm;
                d0_r=hand0_r.dist;                        d0_l=hand0_l.dist;
                d_r=d0_r;                                 d_l=d0_l;
                d_des_r=hand0_r.d_des;                    d_des_l=hand0_l.d_des;
                theta_r=hand0_r.theta;                    theta_l=hand0_l.theta;
                theta_target_r=hand0_r.theta_target;      theta_target_l=hand0_l.theta_target;
                sai_r=hand0_r.sai;                        sai_l=hand0_l.sai;
                sai_target_r=hand0_r.sai_target;          sai_target_l=hand0_l.sai_target;
                phi_r=hand0_r.phi;                        phi_l=hand0_l.phi;
                phi_target_r=hand0_r.phi_target;          phi_target_l=hand0_l.phi_target;
                hand0_r.HO_FK_right_palm(q0_r);           hand0_l.HO_FK_left_palm(q0_l);

                q_ra=q0_r;q_la=q0_l;

                if (simulation)
                {
                    vector<double> q_init(31);
                    for (int i = 0; i < 31; ++i) {
                        q_init[i]=0;
                    }
                    q_init[15]=q_ra(0);   q_init[15+7]=q_la(0);
                    q_init[16]=q_ra(1);   q_init[16+7]=q_la(1);
                    q_init[17]=q_ra(2);   q_init[17+7]=q_la(2);
                    q_init[18]=q_ra(3);   q_init[18+7]=q_la(3);
                    q_init[19]=q_ra(4);   q_init[19+7]=q_la(4);
                    q_init[20]=q_ra(5);   q_init[20+7]=q_la(5);
                    q_init[21]=q_ra(6);   q_init[21+7]=q_la(6);

                    //  SendGazebo(q_init);
                }



                //****path generation

                if(scenario_r=="talking"){
                    t_r<<0,1,2;
                    r_middle_r=hand0_r.r_right_palm;
                    gest_r=0;
                    gest_count_r=1;
                    r_target_r=r_target_rt[gest_r];
                    R_target_r=R_target_rt[gest_r];

                }

                else{
                    t_r<<0,2,4;
                }

                if(scenario_l=="talking"){
                    t_l<<0,1,2;
                    r_middle_l=hand0_l.r_left_palm;
                    gest_l=0;
                    gest_count_l=1;
                    r_target_l=r_target_lt[gest_l];
                    R_target_l=R_target_lt[gest_l];
                }
                else{
                    t_l<<0,2,4;
                }

                if (scenario_r=="handRecog" && scenario_l=="handRecog") {
                    t_r<<4,6,8;
                    t_l<<4,6,8;
                }

                //            MatrixXd T_right_shoulder_transpose=rightshoulder2waist(WaistYaw,WaistPitch);
                //            MatrixXd T_left_shoulder_transpose=leftshoulder2waist(WaistYaw,WaistPitch);

                //            VectorXd temp(4);
                //            temp<<r_target_r,1;
                //            r_target_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);

                //            temp<<r_target_l,1;
                //            r_target_l=(T_left_shoulder_transpose*temp).block(0,0,3,1);

                P_x_r<< hand0_r.r_right_palm(0),r_middle_r(0),r_target_r(0);
                P_y_r<< hand0_r.r_right_palm(1),r_middle_r(1),r_target_r(1);
                P_z_r<< hand0_r.r_right_palm(2),r_middle_r(2),r_target_r(2);
                P_x_l<< hand0_l.r_left_palm(0),r_middle_l(0),r_target_l(0);
                P_y_l<< hand0_l.r_left_palm(1),r_middle_l(1),r_target_l(1);
                P_z_l<< hand0_l.r_left_palm(2),r_middle_l(2),r_target_l(2);

                //            R_target_r=T_right_shoulder_transpose.block(0,0,3,3)*R_target_r;
                //            R_target_l=T_left_shoulder_transpose.block(0,0,3,3)*R_target_l;

                V_x_r<<0,INFINITY,0;     V_x_l<<0,INFINITY,0;
                V_y_r<<0,INFINITY,0;     V_y_l<<0,INFINITY,0;
                V_z_r<<0,INFINITY,0;     V_z_l<<0,INFINITY,0;
                A_x_r<<0,INFINITY,0;     A_x_l<<0,INFINITY,0;
                A_y_r<<0,INFINITY,0;     A_y_l<<0,INFINITY,0;
                A_z_r<<0,INFINITY,0;     A_z_l<<0,INFINITY,0;

                X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
                Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
                Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

                X_coef_l=coef_generator.Coefficient(t_l,P_x_l,V_x_l,A_x_l);
                Y_coef_l=coef_generator.Coefficient(t_l,P_y_l,V_y_l,A_y_l);
                Z_coef_l=coef_generator.Coefficient(t_l,P_z_l,V_z_l,A_z_l);


                //home
                if(scenario_r=="home"){
                    qr_end=q0_r;
                    fingers_mode_r=9;
                }
                if(scenario_l=="home"||scenario_l=="ah"||scenario_l=="lookAtHorizon"){
                    ql_end=q0_l;
                    fingers_mode_l=9;
                }



                count=0;
                headFinished=false;
                initializing=false;
            }


            else {


                time=double(count)*.005;
                time_r=time; time_l=time;

                //  if((scenario_r=="byebye"||scenario_r=="shakeHands")&&time_r>=t_r(2)*2-.005){scenario_r="home";t_r_offset=2*t_r(2);fingers_mode_r=9;}
                //   if((scenario_l=="byebye"||scenario_l=="shakeHands")&&time_l>=t_l(2)*2-.005){scenario_l="home";t_l_offset=2*t_l(2);fingers_mode_l=9;}


                //            if(scenario_l!="byebye"&&scenario_l!="shakeHands"&&scenario_l!="talking"){
                //                if(time_l>=t_l(2)-.005&&time_l<=t_l(2)+2){
                //                  time_l=t_l(2)-.005;
                //                }
                //                else if(time_l>t_l(2)+2){
                //                  time_l-=2;
                //                }
                //            }

                //            if(scenario_r!="byebye"&&scenario_r!="shakeHands"&&scenario_r!="talking"){
                //                if(time_r>=t_r(2)-.005&&time_r<=t_r(2)+2){
                //                    time_r=t_r(2)-.005;
                //                }
                //                else if(time_r>t_r(2)+2){
                //                   time_r-=2;
                //                }
                //            }




                if(scenario_r=="talking"){time_r=fmod(time,t_r(2));

                    if( time_r==0){
                        r_middle_r=r_right_palm;
                        ++gest_count_r;
                        gest_r++;
                        if (gest_r>=8){gest_r=0;}

                        qDebug()<<"gest_count_r:"<<gest_count_r;
                        //                    if (gest_count_r==10){scenario_r="home";
                        //                        t_r_offset=gest_count_r*t_r(2);
                        //                                          }

                        r_target_r=r_target_rt[gest_r];
                        R_target_r=R_target_rt[gest_r];



                        P_x_r<< r_middle_r(0),r_middle_r(0),r_target_r(0);
                        P_y_r<< r_middle_r(1),r_middle_r(1),r_target_r(1);
                        P_z_r<< r_middle_r(2),r_middle_r(2),r_target_r(2);


                        V_x_r<<0,INFINITY,0;
                        V_y_r<<0,INFINITY,0;
                        V_z_r<<0,INFINITY,0;
                        A_x_r<<0,INFINITY,0;
                        A_y_r<<0,INFINITY,0;
                        A_z_r<<0,INFINITY,0;

                        X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
                        Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
                        Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);
                    }
                }
                if(scenario_l=="talking"){time_l=fmod(time,t_l(2));

                    if( time_l==0){
                        r_middle_l=r_left_palm;
                        gest_count_l++;
                        gest_l++;
                        if (gest_l>=8){gest_l=0;}
                        qDebug()<<"gest_count_l:"<<gest_count_l;
                        //        if (gest_count_l==10){ scenario_l="home";
                        //            t_l_offset=gest_count_l*t_l(2);
                        //        }
                        r_target_l=r_target_lt[gest_l];
                        R_target_l=R_target_lt[gest_l];
                        P_x_l<< r_middle_l(0),r_middle_l(0),r_target_l(0);
                        P_y_l<< r_middle_l(1),r_middle_l(1),r_target_l(1);
                        P_z_l<< r_middle_l(2),r_middle_l(2),r_target_l(2);
                        V_x_l<<0,INFINITY,0;
                        V_y_l<<0,INFINITY,0;
                        V_z_l<<0,INFINITY,0;
                        A_x_l<<0,INFINITY,0;
                        A_y_l<<0,INFINITY,0;
                        A_z_l<<0,INFINITY,0;
                        X_coef_l=coef_generator.Coefficient(t_l,P_x_l,V_x_l,A_x_l);
                        Y_coef_l=coef_generator.Coefficient(t_l,P_y_l,V_y_l,A_y_l);
                        Z_coef_l=coef_generator.Coefficient(t_l,P_z_l,V_z_l,A_z_l);
                    }
                }

                //            if(scenario_r=="home"){time_r=time+(t_r(2)-t_r_offset);}
                //            if(scenario_l=="home"){time_l=time+(t_l(2)-t_l_offset);}

                if(scenario_r=="null"){time_r=t_r(2)*3;}

                if(scenario_l=="null"){time_l=t_l(2)*3;}

                VectorXd P_r(3); VectorXd V_r(3);
                VectorXd P_l(3); VectorXd V_l(3);

                //            if(((((scenario_r=="byebye"||scenario_r=="shakeHands")&&(time_r>=2*t_r(2)))||(!(scenario_r=="byebye"||scenario_r=="shakeHands")&&time_r>=t_r(2)))&&time_l>=t_l(2)&&headFinished)||(scenario_l=="talking"&&gest_count_l==10)||(scenario_r=="talking"&&gest_count_r==10)) {
                //                move_activate=false;

                //                qDebug()<<q_la(0)<<","<<q_la(1)<<","<<q_la(2)<<","<<q_la(3)<<","<<q_la(4)<<","<<q_la(5)<<","<<q_la(6);
                //                qDebug()<<"done!";
                //            }


                // qDebug()<<"time="<<time<<"\ttime_r="<<time_r<<"\ttime_l="<<time_l;
                if(scenario_r=="home"||scenario_r=="micDown"){
                    MatrixXd t_h(1,2);
                    MatrixXd p_r(7,2);
                    t_h<<0,t_r(2);


                    p_r<<   qr_end(0),13*M_PI/180,
                            qr_end(1),-10*M_PI/180,
                            qr_end(2),0,
                            qr_end(3),-25*M_PI/180,
                            qr_end(4),0,
                            qr_end(5),0,
                            qr_end(6),0;


                    for (int i = 0; i < 7; ++i) {
                        q_ra(i)=p_r(i,0)+move2pose(p_r(i,1)-p_r(i,0),time_r,t_h(0),t_h(1));

                    }
                    if(scenario_r=="home"){ fingers_r=9;}
                    if(scenario_r=="micDown"){ fingers_r=4;}
                }
                else{
                    if(time_r<t_r(1) && time_r>=t_r(0)){
                        P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,0),
                                coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,0),
                                coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,0);
                        V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,1),
                                coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,1),
                                coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,1);




                        hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
                        r_right_palm=hand_r.r_right_palm;
                        hand_r.doQP(q_ra);
                        q_ra=hand_r.q_next;
                        d_r=hand_r.dist;
                        theta_r=hand_r.theta;
                        sai_r=hand_r.sai;
                        phi_r=hand_r.phi;
                    }

                    else if (time_r<t_r(2) && time_r>t_r(1)){
                        P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,0),
                                coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,0),
                                coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,0);
                        V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,1),
                                coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,1),
                                coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,1);


                        hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
                        r_right_palm=hand_r.r_right_palm;

                        hand_r.doQP(q_ra);
                        q_ra=hand_r.q_next;
                        d_r=hand_r.dist;
                        theta_r=hand_r.theta;
                        sai_r=hand_r.sai;
                        phi_r=hand_r.phi;
                        fingers_r=fingers_mode_r;
                        qr_end=q_ra;
                    }
                    else if (time_r>t_r(2)){
                        if(scenario_r=="s"){
                            double t=time_r-t_r(2);
                            double L=.1;
                            double T=4;
                            if (t<=T) {
                                double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);

                                V_r<< 0,-V,0;
                                if (t<=T/4) {
                                    T=T/4;
                                    V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                    V_r<< 0,0,V;
                                }
                                else if (t<=T/2) {
                                    t-=T/4;
                                    T=T/4;

                                    V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                    V_r<< 0,-V,0;
                                }
                                else if (t<=3*T/4) {
                                    t-=T/2;
                                    T=T/4;
                                    V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                    V_r<< 0,0,-V;
                                }
                                else if (t<=T) {
                                    t-=3*T/4;
                                    T=T/4;
                                    V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                    V_r<< 0,V,0;
                                }

                                hand_r.update_right_hand(q_ra,V_r,r_right_palm,R_target_r);

                                hand_r.doQP(q_ra);
                                q_ra=hand_r.q_next;
                                r_right_palm=hand_r.r_right_palm;
                                d_r=hand_r.dist;
                                theta_r=hand_r.theta;
                                sai_r=hand_r.sai;
                                phi_r=hand_r.phi;
                                fingers_r=fingers_mode_r;
                                qr_end=q_ra;}
                            //                    else{t_r_offset=T+t_r(2);scenario_r="home";
                            //                    }
                        }

                        else if(scenario_r=="o"){
                            double t=time_r-t_r(2);

                            double L=.07*2*M_PI;
                            double T=4;
                            if(t<T){
                                double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
                                V_r<<0,-V*sin(P/L*2*M_PI),V*cos(P/L*2*M_PI);


                                hand_r.update_right_hand(q_ra,V_r,r_right_palm,R_target_r);
                                hand_r.doQP(q_ra);
                                q_ra=hand_r.q_next;
                                r_right_palm=hand_r.r_right_palm;
                                d_r=hand_r.dist;
                                theta_r=hand_r.theta;
                                sai_r=hand_r.sai;
                                phi_r=hand_r.phi;
                                fingers_r=fingers_mode_r;
                                qr_end=q_ra;}
                            //                    else{t_r_offset=T+t_r(2);scenario_r="home";
                            //                    }


                        }


                        else if(scenario_r=="ws"){
                            double t=time_r-t_r(2);

                            double L=.1*3/2*M_PI;
                            double T=4;

                            if(t<T){
                                double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
                                if(t<T/2){V_r<<0,-V*sin(3*P/L*M_PI),V*cos(3*P/L*M_PI);}
                                else{V_r<<0,V*cos(-3*(P-L/2)/L*M_PI),V*sin(-3*(P-L/2)/L*M_PI);}



                                hand_r.update_right_hand(q_ra,V_r,r_right_palm,R_target_r);
                                r_right_palm=hand_r.r_right_palm;
                                hand_r.doQP(q_ra);
                                q_ra=hand_r.q_next;
                                d_r=hand_r.dist;
                                theta_r=hand_r.theta;
                                sai_r=hand_r.sai;
                                phi_r=hand_r.phi;
                                fingers_r=fingers_mode_r;
                                qr_end=q_ra;


                            }

                            //                    else{t_r_offset=T+t_r(2);scenario_r="home";
                            //                    }
                        }


                        else if(scenario_r=="wu"){
                            double t=time_r-t_r(2);

                            double L=.1*M_PI/2+.3;
                            double T=4;

                            if(t<T){
                                double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);


                                if(P<L/3){V_r<<0,0,-V;}
                                else if(P<2*L/3){V_r<<0,V*sin(-3*M_PI/L*(P-L/3)-M_PI),V*cos(-3*M_PI/L*(P-L/3)-M_PI);}
                                else{V_r<<0,0,V;}



                                hand_r.update_right_hand(q_ra,V_r,r_right_palm,R_target_r);
                                r_right_palm=hand_r.r_right_palm;
                                hand_r.doQP(q_ra);
                                q_ra=hand_r.q_next;
                                d_r=hand_r.dist;
                                theta_r=hand_r.theta;
                                sai_r=hand_r.sai;
                                phi_r=hand_r.phi;
                                fingers_r=fingers_mode_r;
                                qr_end=q_ra;


                            }

                            //                    else{t_r_offset=T+t_r(2);scenario_r="home";
                            //                    }
                        }


                        else{


                            if (time_r<t_r(2)*2 && time_r>t_r(2)){



                                if(scenario_r=="byebye"){q_ra(2)=qr_end(2)+15*M_PI/180*sin((time_r-t_r(2))/2*(2*M_PI));}
                                else if(scenario_r=="shakeHands"){
                                    if(time_r>t_r(2)+1){
                                        q_ra(3)=qr_end(3)+5*M_PI/180*sin((time_r-(t_r(2)+1))/2*(2*M_PI));}
                                }

                                else{
                                    ROS_INFO_ONCE("right reached!");

                                }
                            }
                        }










                    }//

                }


                if(scenario_r=="getMic"){
                    if(time_r>=t_r(2)-.1){fingers_r=4;}
                    else if(time_r>t_r(1)){fingers_r=3;}
                }
                if(scenario_l=="m5"){ fingers_l=10;}

                if(scenario_l=="home"||scenario_l=="micDown"){

                    MatrixXd t_h(1,2);
                    MatrixXd p_l(7,2);
                    t_h<<0,t_l(2);

                    p_l<<   ql_end(0),13*M_PI/180,
                            ql_end(1),10*M_PI/180,
                            ql_end(2),0,
                            ql_end(3),-25*M_PI/180,
                            ql_end(4),0,
                            ql_end(5),0,
                            ql_end(6),0;

                    for (int i = 0; i < 7; ++i) {
                        q_la(i)=p_l(i,0)+move2pose(p_l(i,1)-p_l(i,0),time_l,t_h(0),t_h(1));
                    }
                    if(scenario_l=="home"){ fingers_l=9;}
                    if(scenario_l=="micDown"){ fingers_l=4;}
                }

                else if(scenario_l=="lookAtHorizon"){
                    MatrixXd t_h(1,2);
                    MatrixXd p_l(7,2);
                    VectorXd q_d(7,1);
                    t_h<<0,t_l(2);
                    q_d<<-1.57268 , 1.34502 , -0.768361 , -1.5708 , 0.673369 , -0.349066 , -0.349066;
                    p_l<<ql_end,q_d;
                    for (int i = 0; i < 7; ++i) {
                        q_la(i)=p_l(i,0)+move2pose(p_l(i,1)-p_l(i,0),time_l,t_h(0),t_h(1));
                    }
                }




                else{
                    if(time_l<t_l(1) && time_l>=t_l(0)){

                        P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_l,t_l(0),5)(0,0),
                                coef_generator.GetAccVelPos(Y_coef_l.row(0),time_l,t_l(0),5)(0,0),
                                coef_generator.GetAccVelPos(Z_coef_l.row(0),time_l,t_l(0),5)(0,0);
                        V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_l,t_l(0),5)(0,1),
                                coef_generator.GetAccVelPos(Y_coef_l.row(0),time_l,t_l(0),5)(0,1),
                                coef_generator.GetAccVelPos(Z_coef_l.row(0),time_l,t_l(0),5)(0,1);


                        hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);

                        r_left_palm=hand_l.r_left_palm;
                        hand_l.doQP(q_la);

                        q_la=hand_l.q_next;
                        d_l=hand_l.dist;
                        theta_l=hand_l.theta;
                        sai_l=hand_l.sai;
                        phi_l=hand_l.phi;

                    }

                    else if (time_l<t_l(2) && time_l>t_l(1)){

                        P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_l,t_l(1),5)(0,0),
                                coef_generator.GetAccVelPos(Y_coef_l.row(1),time_l,t_l(1),5)(0,0),
                                coef_generator.GetAccVelPos(Z_coef_l.row(1),time_l,t_l(1),5)(0,0);
                        V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_l,t_l(1) ,5)(0,1),
                                coef_generator.GetAccVelPos(Y_coef_l.row(1),time_l,t_l(1),5)(0,1),
                                coef_generator.GetAccVelPos(Z_coef_l.row(1),time_l,t_l(1),5)(0,1);
                        hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);
                        r_left_palm=hand_l.r_left_palm;
                        hand_l.doQP(q_la);
                        q_la=hand_l.q_next;
                        d_l=hand_l.dist;
                        theta_l=hand_l.theta;
                        sai_l=hand_l.sai;
                        phi_l=hand_l.phi;

                        fingers_l=fingers_mode_l;
                        ql_end=q_la;

                    }


                    else if (time_l>t_l(2)){
                        if(scenario_l=="s"){

                            double t=time_l-t_l(2);
                            double L=.1;
                            double T=4;
                            if(t<T){
                                double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                V_l<< 0,V,0;
                                if (t<=T/4) {
                                    T=T/4;
                                    V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                    V_l<< 0,0,V;
                                }
                                else if (t<=T/2) {
                                    t-=T/4;
                                    T=T/4;

                                    V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                    V_l<< 0,V,0;
                                }
                                else if (t<=3*T/4) {
                                    t-=T/2;
                                    T=T/4;
                                    V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                    V_l<< 0,0,-V;
                                }
                                else if (t<=T) {
                                    t-=3*T/4;
                                    T=T/4;
                                    V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                    V_l<< 0,-V,0;
                                }


                                hand_l.update_left_hand(q_la,V_l,r_left_palm,R_target_l);
                                r_left_palm=hand_l.r_left_palm;
                                hand_l.doQP(q_la);
                                q_la=hand_l.q_next;
                                d_l=hand_l.dist;
                                theta_l=hand_l.theta;
                                sai_l=hand_l.sai;
                                phi_l=hand_l.phi;
                                fingers_l=fingers_mode_l;
                                ql_end=q_la;


                            }

                            //                    else{t_l_offset=T+t_l(2);scenario_l="home";
                            //                    }
                        }
                        else if(scenario_l=="o"){
                            double t=time_l-t_l(2);

                            double L=.07*2*M_PI;
                            double T=4;
                            if(t<T){
                                double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
                                V_l<<0,V*sin(P/L*2*M_PI),V*cos(P/L*2*M_PI);

                                hand_l.update_left_hand(q_la,V_l,r_left_palm,R_target_l);
                                r_left_palm=hand_l.r_left_palm;
                                hand_l.doQP(q_la);
                                q_la=hand_l.q_next;
                                d_l=hand_l.dist;
                                theta_l=hand_l.theta;
                                sai_l=hand_l.sai;
                                phi_l=hand_l.phi;
                                fingers_l=fingers_mode_l;
                                ql_end=q_la;


                            }

                            //                    else{t_l_offset=T+t_l(2);scenario_l="home";
                            //                    }
                        }

                        else if(scenario_l=="ws"){
                            double t=time_l-t_l(2);

                            double L=.1*3/2*M_PI;
                            double T=4;

                            if(t<T){
                                double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
                                if(t<T/2){V_l<<0,-V*sin(3*P/L*M_PI),V*cos(3*P/L*M_PI);}
                                else{V_l<<0,V*cos(-3*(P-L/2)/L*M_PI),V*sin(-3*(P-L/2)/L*M_PI);}



                                hand_l.update_left_hand(q_la,V_l,r_left_palm,R_target_l);
                                r_left_palm=hand_l.r_left_palm;
                                hand_l.doQP(q_la);
                                q_la=hand_l.q_next;
                                d_l=hand_l.dist;
                                theta_l=hand_l.theta;
                                sai_l=hand_l.sai;
                                phi_l=hand_l.phi;
                                fingers_l=fingers_mode_l;
                                ql_end=q_la;


                            }

                            //                    else{t_l_offset=T+t_l(2);scenario_l="home";
                            //                    }
                        }


                        else if(scenario_l=="wu"){
                            double t=time_l-t_l(2);

                            double L=.1/2*M_PI+.3;
                            double T=4;

                            if(t<T){
                                double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                                double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
                                if(P<L/3){V_l<<0,0,-V;}
                                else if(P<2*L/3){V_l<<0,V*sin(-3*M_PI/L*(P-L/3)-M_PI),V*cos(-3*M_PI/L*(P-L/3)-M_PI);}
                                else{V_l<<0,0,V;}

                                hand_l.update_left_hand(q_la,V_l,r_left_palm,R_target_l);
                                r_left_palm=hand_l.r_left_palm;
                                hand_l.doQP(q_la);
                                q_la=hand_l.q_next;
                                d_l=hand_l.dist;
                                theta_l=hand_l.theta;
                                sai_l=hand_l.sai;
                                phi_l=hand_l.phi;
                                fingers_l=fingers_mode_l;
                                ql_end=q_la;


                            }

                            //                    else{t_l_offset=T+t_l(2);scenario_l="home";
                            //                    }
                        }


                        else{
                            if (time_l<t_l(2)*2 && time_l>t_l(2)){

                                if(scenario_l=="byebye"){q_la(2)=ql_end(2)+15*M_PI/180*sin((time_l-t_l(2))/2*(2*M_PI));}
                                else if(scenario_l=="shakeHands"){
                                    if (time_l>t_l(2)+1)
                                    {q_la(3)=ql_end(3)+5*M_PI/180*sin((time_l-(t_l(2)+1))/2*(2*M_PI));}
                                }
                                if(scenario_r=="giveMicBack"){
                                    if(time_r>t_r(2)+1){
                                        fingers_r=3;}
                                }
                                else{
                                    ROS_INFO_ONCE("left reached!");
                                }

                            }







                        }

                    }

                }

                if(scenario_l=="getMic"&&time_l==t_l(1)){ fingers_l=3;}
                if(scenario_l=="getMic"&&time_l==(t_l(2)-.005)){fingers_l=4;}
                if(scenario_l=="m5"){ fingers_l=10;}







                //          if(time_r>2*t_r(2)&&time_l>2*t_l(2)) {
                //              move_activate=false;
                //              qDebug()<<"done!";
                //          }






                //        for (int i = 0; i < 31; ++i) {
                //            q[i]=0;
                //        }

                double pitchRange=20*M_PI/180;
                double yawRange=45*M_PI/180;
                double WYawRange=20*M_PI/180;
                double WPitchRange=15*M_PI/180;

                if(scenario_hw=="face"){

                    if(n_people!=0){
                        head_yaw+=.0004*(320-X_face);
                        head_pitch-=.0004*(200-Y_face);
                    }
                    head_yaw=saturate(head_yaw,-45,45);
                    head_pitch=saturate(head_pitch,-15,20);
                    if(double(count*.005)>=2){headFinished=true;}
                }

                else if(scenario_hw=="start0"){

                    head_pitch=move2pose(pitchRange,double(count*.005),0,2);
                    if(double(count*.005)>=2){headFinished=true;}

                }
                else if(scenario_hw=="start2"){

                    head_pitch=pitchRange-move2pose(pitchRange,double(count*.005),0,2);
                    if(double(count*.005)>=2){headFinished=true;}
                }
                //look at hands


                else if(scenario_hw=="lookAtHands"){
                    head_yaw=move2pose(yawRange-30*M_PI/180,double(count*.005),0,1)+
                            move2pose(-2*(yawRange-30*M_PI/180),double(count*.005),1,3)+
                            move2pose(yawRange-30*M_PI/180,double(count*.005),3,4);
                    if(double(count*.005)>=4){headFinished=true;}

                }

                else if(scenario_hw=="lookAtHorizon"){
                    head_yaw=move2pose(yawRange-20*M_PI/180,double(count*.005),0,2);
                    head_pitch=pitchRange-move2pose(pitchRange,double(count*.005),0,2);
                    WaistYaw=move2pose(WYawRange,double(count*.005),0,2);
                    if(double(count*.005)>=2){headFinished=true;}

                }

                else if(scenario_hw=="horizon2"){

                    head_yaw=yawRange-20*M_PI/180-move2pose(yawRange-20*M_PI/180,double(count*.005),0,2);

                    WaistYaw=WYawRange-move2pose(WYawRange,double(count*.005),0,2);
                    if(double(count*.005)>=2){headFinished=true;}

                }

                else if(scenario_hw=="footRecog"){

                    head_pitch=move2pose(pitchRange,double(count*.005),t_l(0)-4,t_l(2));

//                    q[2]=move2pose(-.15,double(count*.005),0,1)+
//                            move2pose(.3,double(count*.005),3,5)+
//                            move2pose(-.15,double(count*.005),7,8);
//                    //qDebug()<<double(count*.005)<<'\t'<<q[2];
//                    q[8]=q[2];
//                    q[6]=-q[2];
//                    q[12]=-q[2];

//                    q[4]=move2pose(.5,double(count*.005),1,2)+
//                            move2pose(-.5,double(count*.005),2,3);
//                    q[3]=-q[4]/2;
//                    q[5]=-q[4]/2;

//                    q[10]=move2pose(.5,double(count*.005),5,6)+
//                            move2pose(-.5,double(count*.005),6,7);

//                    q[9]=-q[10]/2;
//                    q[11]=-q[10]/2;

                    q[2]=move2pose(-.1,double(count*.005),0,1.5)+
                            move2pose(.2,double(count*.005),1.5,4.5)+
                            move2pose(-.1,double(count*.005),4.5,6);
                    //qDebug()<<double(count*.005)<<'\t'<<q[2];
                    q[8]=q[2];
                    q[6]=-q[2];
                    q[12]=-q[2];






                    if(double(count*.005)>=8){headFinished=true;}

                }

                else if(scenario_hw=="getMic"){
                    head_yaw=move2pose(-yawRange+25*M_PI/180,double(count*.005),0,4);
                    head_pitch=move2pose(pitchRange-10*M_PI/180,double(count*.005),0,4);
                    if(double(count*.005)>=4){headFinished=true;}

                }
                else  if(scenario_hw=="m3"){
                    head_yaw=-yawRange+25*M_PI/180-move2pose(-yawRange+25*M_PI/180,double(count*.005),0,4);
                    head_pitch=pitchRange-10*M_PI/180-move2pose(pitchRange-10*M_PI/180,double(count*.005),0,4);
                    if(double(count*.005)>=4){headFinished=true;}

                }


                else  if(scenario_hw=="smoothMove"){

                    head_yaw =(yawRange-37*M_PI/180)*sin(double(count*.005)/4*(2*M_PI))*(move2pose(1,double(count*.005),0,.5)-move2pose(1,double(count*.005),talkTime-.5,talkTime));
                    q[2]=.03*sin(double(count*.005)/4*(2*M_PI))*(move2pose(1,double(count*.005),0,.5)-move2pose(1,double(count*.005),talkTime-.5,talkTime));
                    q[8]=q[2];
                    q[6]=-q[2];
                    q[12]=-q[2];

                    if(double(count*.005)>=talkTime){headFinished=true;}
                }

                else if(scenario_hw=="lookAtPresentor"){
                    head_yaw =-move2pose(yawRange*1.5,double(count*.005),0,2);
                    if(double(count*.005)>=2){headFinished=true;}
                }
                else  if(scenario_hw=="confirm"){
                    head_pitch =move2pose(pitchRange-10*M_PI/180,double(count*.005),0,1)-move2pose(pitchRange-10*M_PI/180,double(count*.005),1,2);
                    if(double(count*.005)>=2){headFinished=true;}
                }
                else  if(scenario_hw=="t4"){
                    head_yaw =-yawRange+move2pose(yawRange,double(count*.005),0,2);
                    if(double(count*.005)>=2){headFinished=true;}
                }

                else  if(scenario_hw=="respect"){
                    WaistPitch =move2pose(WPitchRange-5*M_PI/180,double(count*.005),0,2);
                    head_pitch =move2pose(pitchRange-10*M_PI/180,double(count*.005),0,2);
                    if(double(count*.005)>=2){headFinished=true;}
                }
                else  if(scenario_hw=="c2"){
                    WaistPitch =(WPitchRange-5*M_PI/180)-move2pose(WPitchRange-5*M_PI/180,double(count*.005),0,2);
                    head_pitch =pitchRange-10*M_PI/180-move2pose(pitchRange-10*M_PI/180,double(count*.005),0,2);
                    if(double(count*.005)>=2){headFinished=true;}
                }
                else if(scenario_hw=="wh(comeDown)"){

                    double dh=move2pose(.02,double(count*.005),t_r(0),t_r(2));
                    double l=.36+.37-dh;

                    q[4]=acos((l*l-.37*.37-.36*.36)/2/.36/.37);
                    q[3]=-atan(sin(q[4])*.36/(cos(q[4])*.36+.37));
                    q[5]=-q[4]-q[3];
                    q[9]=q[3];
                    q[10]=q[4];
                    q[11]=q[5];
                    head_pitch=move2pose(10*M_PI/180,double(count*.005),t_r(0),t_r(1));
                    WaistPitch=move2pose(10*M_PI/180,double(count*.005),t_r(0),t_r(2));
                    if(double(count*.005)>=t_r(2)){headFinished=true;}

                }

                else if(scenario_hw=="wh(comeUp)"){

                    double dh=.02-move2pose(.02,double(count*.005),t_r(0),t_r(2));
                    double l=.36+.37-dh;

                    q[4]=acos((l*l-.37*.37-.36*.36)/2/.36/.37);
                    q[3]=-atan(sin(q[4])*.36/(cos(q[4])*.36+.37));
                    q[5]=-q[4]-q[3];
                    q[9]=q[3];
                    q[10]=q[4];
                    q[11]=q[5];
                    //balloon=comment
                    // head_pitch=10*M_PI/180-move2pose(10*M_PI/180,double(count*.005),t_r(0),t_r(1));
                    WaistPitch=10*M_PI/180-move2pose(10*M_PI/180,double(count*.005),t_r(0),t_r(2));
                    if(double(count*.005)>=t_r(2)){headFinished=true;}

                }
                else if(scenario_hw=="wh(look)"){
                    head_roll=move2pose(10*M_PI/180,double(count*.005),0,1)-2*move2pose(10*M_PI/180,double(count*.005),2,5)+move2pose(10*M_PI/180,double(count*.005),6,7);
                    if(double(count*.005)>=7){headFinished=true;}

                }

                else if(scenario_hw=="wh(release)"){
                    head_pitch=10*M_PI/180+2*move2pose(-6*M_PI/180,double(count*.005),0,6);
                    if(double(count*.005)>=6){headFinished=true;}

                }

                else if(scenario_hw=="wh3_3"){
                    head_pitch=-6*M_PI/180-move2pose(-6*M_PI/180,double(count*.005),0,3);
                    if(double(count*.005)>=3){headFinished=true;}

                }
                else if(scenario_hw=="home"){
                    double t=double(count*.005);
                    double T_home=4;
                    head_pitch=move2zero(head_pitch,t,T_home);
                    head_roll=move2zero(head_roll,t,T_home);
                    head_yaw=move2zero(head_yaw,t,T_home);
                    WaistPitch=move2zero(WaistPitch,t,T_home);
                    WaistYaw=move2zero(WaistYaw,t,T_home);
                    if(t>=T_home){headFinished=true;}

                }
                else{headFinished=true;}


                if((((scenario_r=="giveMicBack"&&(time_r>=t_r(2)+2))||((scenario_r=="byebye"||scenario_r=="shakeHands")&&(time_r>=2*t_r(2)))||(!((scenario_r=="byebye"||scenario_r=="shakeHands")||scenario_r=="giveMicBack")&&time_r>=t_r(2)))&&time_l>=t_l(2)&&headFinished)||(scenario_l=="talking"&&gest_count_l==int((talkTime-1)/2)+1)||(scenario_r=="talking"&&gest_count_r==int((talkTime-1)/2)+1)) {
                    move_activate=false;

                   // qDebug()<<q_la(0)<<","<<q_la(1)<<","<<q_la(2)<<","<<q_la(3)<<","<<q_la(4)<<","<<q_la(5)<<","<<q_la(6);
                    qDebug()<<"done!";
                }





            }
            q[13]=WaistYaw;
            q[14]=WaistPitch;
            q[15]=q_ra(0)-q_rh(0);
            q[16]=q_ra(1)-q_rh(1);
            q[17]=q_ra(2)-q_rh(2);
            q[18]=q_ra(3)-q_rh(3);
            q[19]=q_ra(4)-q_rh(4);
            q[20]=q_ra(5)-q_rh(5);
            q[21]=q_ra(6)-q_rh(6);
            q[22]=q_la(0)-q_lh(0);
            q[23]=q_la(1)-q_lh(1);
            q[24]=q_la(2)-q_lh(2);
            q[25]=q_la(3)-q_lh(3);
            q[26]=q_la(4)-q_lh(4);
            q[27]=q_la(5)-q_lh(5);
            q[28]=q_la(6)-q_lh(6);
            q[29]=head_yaw;
            q[30]=head_roll;
            q[31]=head_pitch;


            if(q[32]!=double(fingers_r)&&fingers_r_2counter<10){
                q[32]=2;
                fingers_r_2counter++;
//                qDebug()<<fingers_r_2counter;
            }
            else {
                q[32]=double(fingers_r);
                fingers_r_2counter=0;
            }

            if(q[33]!=double(fingers_l)&&fingers_l_2counter<10){
                q[33]=2;
                fingers_l_2counter++;
//                qDebug()<<fingers_r_2counter;
            }
            else {
                q[33]=double(fingers_l);
                fingers_l_2counter=0;
            }






            count++;





        }

        trajectory_data.data.clear();
        for (int i = 0; i <= 33; ++i) {
            trajectory_data.data.push_back(q[i]);

        }
        trajectory_data_pub.publish(trajectory_data);

        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}

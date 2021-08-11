#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
#include"taskspaceonline3.h"
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
#include<geometry_msgs/Wrench.h>
#include<math.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include "qcgenerator.h"
#include<termios.h>
#include<gazebo_msgs/LinkStates.h>
#include<sensor_msgs/JointState.h>
#include"pidcontroller.h"
#include"MinimumJerkInterpolation.h"
#include "right_hand.h"
#include "left_hand.h"
#include <geometry_msgs/PoseArray.h>
#include<termios.h>
#include <QTime>
#include"trajectory_generation/walk.h"
#include "trajectory_generation/walkRequest.h"
#include "trajectory_generation/walkResponse.h"
#include "trajectory_generation/handmove.h"
#include "trajectory_generation/handmoveRequest.h"
#include "trajectory_generation/handmoveResponse.h"
#include "trajectory_generation/offlineData.h"
#include "trajectory_generation/offlineDataRequest.h"
#include "trajectory_generation/offlineDataResponse.h"
#include "trajectory_generation/offlineTrajectory.h"
#include "trajectory_generation/offlineTrajectoryRequest.h"
#include "trajectory_generation/offlineTrajectoryResponse.h"
#include "pelviscontroller.h"
#include "ankle_adaptation.h"
#define PelvisVibControlllerCoef 5


using namespace  std;
using namespace  Eigen;


//sum-up&send params
bool simulation=false;
vector<double> q(34);
vector<double> q_upper(34);
vector<double> q_lower(34);
vector<double> q_offlineData(34);
int qc_offset[40];
//int incSensors[12];
bool qc_initial_bool;
bool qc_update_bool;
QCgenerator QC;
////////////////////////////////////simulation
ros::Publisher pub1  ;ros::Publisher pub2  ;ros::Publisher pub3  ;ros::Publisher pub4  ;
ros::Publisher pub5  ;ros::Publisher pub6  ;ros::Publisher pub7  ;ros::Publisher pub8  ;
ros::Publisher pub9  ;ros::Publisher pub10 ;ros::Publisher pub11 ;ros::Publisher pub12 ;
ros::Publisher pub13 ;ros::Publisher pub14 ;ros::Publisher pub15 ;ros::Publisher pub16 ;
ros::Publisher pub17 ;ros::Publisher pub18 ;ros::Publisher pub19 ;ros::Publisher pub20 ;
ros::Publisher pub21 ;ros::Publisher pub22 ;ros::Publisher pub23 ;ros::Publisher pub24 ;
ros::Publisher pub25 ;ros::Publisher pub26 ;ros::Publisher pub27 ;ros::Publisher pub28 ;
ros::Publisher pub29 ;ros::Publisher pub30 ;ros::Publisher pub31 ;

ros::Publisher chatter_pub;

ros::Subscriber qcinit;
ros::ServiceServer WalkService;
ros::ServiceServer StopService;
ros::ServiceServer HandMoveService;
ros::ServiceServer StopTalkService;
//ros::ServiceServer QCoffsetupdateService;
//ros::ServiceServer offlineDataService;
//ros::ServiceServer offlineTrajectoryService;
//ros::Subscriber face_sub;
ros::Subscriber sub;
//ros::Subscriber imusub;



std_msgs::Int32MultiArray msg;
std_msgs::MultiArrayDimension msg_dim;

// lower-body params
double stepLength = .25;
int NStride=1;
bool left_first=true;//right support in first step
bool turning=false;
double TurningRadius=.01;//1.5/M_PI;//
bool move_lower_active=false;
bool EndPhase_bool=true;
bool backward=false;
bool sidewalk=false;
int bump_threshold=75;//75 85;
bool AnkleZAdaptation=!false;
bool LogDataSend=false;
double ankle_adaptation_switch=1;// 1 for activating adaptation 0 for siktiring adaptation
double k_pitch=0*.8;//1;0.8;
double pelvis_roll_range=1; //1
double rankle_inc,lankle_inc;
double lknee_inc,rknee_inc;
double lankle_absolute,rankle_absolute;
double lknee_absolute,rknee_absolute;
bool JustStartphase;
Robot SURENA;//model of robot & kinematics funcs(IK & FK)
Robot SURENA_turning_side;
TaskSpaceOnline3 OnlineTaskSpace(NStride,stepLength);
QList<LinkM> links;
MatrixXd PoseRoot;//position of pelvis respected to global coordinate
MatrixXd PoseRFoot;//position of right ankle joint respected to global coordinate
MatrixXd PoseLFoot;//position of left ankle joint respected to global coordinate
double GlobalTime;
double footSensorSaturation=75;//if all sensors data are bigger than this amount, this means the foot is landed on the ground
double footSensorthreshold=4;// will start orientaition correction
double DurationOfStartPhase;
double DurationOfendPhase;
double PelvisCurrentHeight;
double shoulderPitchOffset;
double right_hand_pitch=0;
double left_hand_pitch=0;
int a,b,c,d,e,f,g,h;//data of left foot sensor
double PitchModified;
double AnkleZR,AnkleZL;
double AnkleZ_offsetR=0;
double AnkleZ_offsetL=0;
int bump_pushed[8];
int bump_notpushed[8];
int bump_pushed2[8];
int bump_notpushed2[8];
bool bump_initialize;
vector<double> cntrl(13);
bool leftzstop;
bool rightzstop;
double Fzl,Fzr,Mxl,Mxr;
double pelvis_orientation_pitch,pelvis_orientation_roll,pelvis_orientation_yaw;
double pelvis_acceleration[3];
double pelvis_angularVelocity[3];
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
double threshold2=100;
double kp=.02;

//upper-body params

bool move_upper_activate=false;
VectorXd r_right_palm(3);
VectorXd r_left_palm(3);
MatrixXd R_right_palm(3,3);
MatrixXd R_left_palm(3,3);
int fingers_mode_r;
int fingers_mode_l;
int fingers_r_2counter=0;
int fingers_l_2counter=0;
int fingers_r=9;
int fingers_l=9;
VectorXd q_ra(7);
VectorXd q_la(7);
VectorXd q_rh(7);
VectorXd q_lh(7);
double talkTime=0;
right_hand hand_r;
left_hand hand_l;
double Waist2ArmZ=0.2694;
double Waist2RArmY=-0.239;
double Waist2LArmY=0.239;
std::string scenario_r="null";
std::string scenario_l="null";
std::string scenario_hw="null";
bool headFinished=false;
bool initializing=true;
bool stopTalkbool;
int count_upper;
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

int N_data;
MatrixXd positionmat[10000];
int num_data = 0;
int N_Trajectorydata;
MatrixXd Trajectorymat[10000];
int num_Trajectorydata = 0;
Robot SURENA_Offline;

bool move_active_offlineData=false;
bool move_active_offlineTrajectory=false;
double offlineTrajectoryTend_ideal;
bool offlineTrajectoryContact=false;
int offlineTrajectoryIcontact;
//PelvisController pelvisController;
Ankle_adaptation ankladpt;
QByteArray Trajectory;
//QByteArray alai1;
//QByteArray alai2;
//QByteArray alai3;
//QByteArray alai4;
//double outControl;
int q_motor_r[8];
int q_motor_l[8];
vector<int> qref(12);
int head_yaw_motor,head_pitch_motor,head_roll_motor,WaistYaw_motor,WaistPitch_motor;

////////////////////////////////////simulation
void  SendGazebo(vector<double> q){


    std_msgs::Float64 data;
    data.data=q[1];
    pub1.publish(data);



    data.data=q[2];
    pub2.publish(data);
    data.data=q[3];
    pub3.publish(data);
    data.data=q[4];
    pub4.publish(data);
    data.data=q[5];
    pub5.publish(data);
    data.data=q[6];
    pub6.publish(data);
    data.data=q[7];
    pub7.publish(data);
    data.data=q[8];
    pub8.publish(data);
    data.data=q[9];
    pub9.publish(data);
    data.data=q[10];
    pub10.publish(data);
    data.data=q[11];
    pub11.publish(data);
    data.data=q[12];
    pub12.publish(data);
    data.data=q[13];
    pub13.publish(data);
    data.data=q[14];
    pub14.publish(data);
    data.data=q[15];
    pub15.publish(data);
    data.data=q[16];
    pub16.publish(data);
    data.data=q[17];
    pub17.publish(data);
    data.data=q[18];
    pub18.publish(data);
    data.data=q[19];
    pub19.publish(data);
    data.data=q[20];
    pub20.publish(data);
    data.data=q[21];
    pub21.publish(data);
    data.data=q[22];
    pub22.publish(data);
    data.data=q[23];
    pub23.publish(data);
    data.data=q[24];
    pub24.publish(data);
    data.data=q[25];
    pub25.publish(data);
    data.data=q[26];
    pub26.publish(data);
    data.data=q[27];
    pub27.publish(data);
    data.data=q[28];
    pub28.publish(data);
    data.data=q[29];
    pub29.publish(data);
    data.data=q[30];
    pub30.publish(data);
    data.data=q[31];
    pub31.publish(data);
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

double d2r(double d){
    return d*M_PI/180;
}


double move_rest_back(double max,double t_local,double T_start ,double T_move,double T_rest,double T_back){
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double c3_r=(10*max)/pow(T_back,3);
    double c4_r=-(15*max)/pow(T_back,4);
    double c5_r=(6*max)/pow(T_back,5);
    double T_end=T_start+T_move+T_rest+T_back;
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_start+T_move){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else if (t_local<T_start+T_move+T_rest){theta=max;}
    else if (t_local<T_start+T_move+T_rest+T_back){theta=c3_r*pow(T_end-t_local,3)+c4_r*pow(T_end-t_local,4)+c5_r*pow(T_end-t_local,5);}
    return theta;
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

double move2pose_diff(double max,double t_local,double T_start ,double T_end){
    double T_move=T_end-T_start;
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_end){theta=c3*pow(t_local-T_start,2)*3+c4*pow(t_local-T_start,3)*4+c5*pow(t_local-T_start,4)*5;}
    else{theta=0;}
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

MatrixXd leftshoulder2waist(double WaistYaw, double WaistPitch)
{
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

//void face_detect(const geometry_msgs::PoseArray & msg){
//    n_people=msg.poses.size();

//    double temp_x,temp_y;
//    double d=480*480+640*640;


//    if(n_people!=0) {

//        for (int i = 0; i < n_people; ++i) {
//         temp_x=msg.poses[i].position.x;
//         temp_y=msg.poses[i].position.y;
//         if(d>(pow(X_face-temp_x,2)+pow(Y_face-temp_y,2))){
//             d=pow(X_face-temp_x,2)+pow(Y_face-temp_y,2);
//             X_face=msg.poses[i].position.x;
//             Y_face=msg.poses[i].position.y;
//         }


//        }
//    }

//    else{
//        X_face+=1e-6*(320-X_face);
//        Y_face+= 1e-6*(200-X_face);
//    }

//}

void receiveFootSensor(const std_msgs::Int32MultiArray& msg)
{
    if (msg.data.size()!=8) {
        qDebug("the size of sensor data is in wrong");
        return;
    }
    if(bump_initialize){
        for (int i = 0; i < 8; ++i) {
            bump_pushed[i]=msg.data[i];
        }
        bump_initialize=false;
    }
    //ROS_INFO("I heard: [%d  %d %d %d %d  %d %d %d]", (int)msg.data[0],(int)msg.data[1],(int)msg.data[2],(int)msg.data[3],(int)msg.data[4],(int)msg.data[5],(int)msg.data[6],(int)msg.data[7]);
    int temp[8];



    bump_pushed[0]=1087;bump_pushed[1]= 853;bump_pushed[2]=3120;bump_pushed[3]=3010;
    //bump_pushed[4]=3111;bump_pushed[5]=2941;bump_pushed[6]=1160;bump_pushed[7]=959; commented by (Hamid)
    bump_pushed[4]=3111;bump_pushed[5]=2941;bump_pushed[6]=1170;bump_pushed[7]=923; // new (Hamid)

    bump_notpushed[0]=1012;bump_notpushed[1]= 926;bump_notpushed[2]=3033;bump_notpushed[3]=3096;
    bump_notpushed[4]=3021;bump_notpushed[5]=2987;bump_notpushed[6]=1093;bump_notpushed[7]=1011;


    //qDebug()<<msg.data[0]<<"\t"<<msg.data[1]<<"\t"<<msg.data[2]<<"\t"<<msg.data[3]<<"\t"<<msg.data[4]<<"\t"<<msg.data[5]<<"\t"<<msg.data[6]<<"\t"<<msg.data[7];
    //    if (GlobalTime<.2){
    //        for (int i = 0; i < 8; ++i) {
    //            bump_pushed[i]=msg.data[i];
    //        }
    //    }
    //    if(left_first){
    //        if(fabs(GlobalTime-(DurationOfStartPhase+OnlineTaskSpace.TStart-OnlineTaskSpace.TSS/2))<.2){
    //            for (int i = 0; i < 4; ++i) {
    //                bump_notpushed[i]=msg.data[i];
    //            }
    //        }

    //        if(fabs(GlobalTime-(DurationOfStartPhase+OnlineTaskSpace.TStart+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS/2))<.2){
    //            for (int i = 4; i < 8; ++i) {
    //                bump_notpushed[i]=msg.data[i];
    //            }
    //        }
    //    }
    //    else{
    //        if(fabs(GlobalTime-(DurationOfStartPhase+OnlineTaskSpace.TStart-OnlineTaskSpace.TSS/2))<.2){
    //            for (int i = 4; i < 8; ++i) {
    //                bump_notpushed[i]=msg.data[i];
    //            }
    //        }

    //        if(fabs(GlobalTime-(DurationOfStartPhase+OnlineTaskSpace.TStart+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS/2))<.2){
    //            for (int i = 0; i < 4; ++i) {
    //                bump_notpushed[i]=msg.data[i];
    //            }
    //        }
    //    }




    //    bump_pushed2[0]=1100;bump_pushed2[1]= 835;bump_pushed2[2]=3140;bump_pushed2[3]=2994;
    //    bump_pushed2[4]=3132;bump_pushed2[5]=2916;bump_pushed2[6]=1203;bump_pushed2[7]=916;
    //    bump_notpushed2[0]=1012;bump_notpushed2[1]= 928;bump_notpushed2[2]=3033;bump_notpushed2[3]=3097;
    //    bump_notpushed2[4]=3036;bump_notpushed2[5]=3006;bump_notpushed2[6]=1107;bump_notpushed2[7]=1015;

    //        for (int i = 0; i < 8; ++i) {
    //            qDebug()<<i<<"\t"<<bump_pushed[i]<<"("<<bump_pushed2[i]<<")\t"<<bump_notpushed[i]<<"("<<bump_notpushed2[i]<<")";
    //        }


    temp[0]=msg.data[0]-bump_notpushed[0];
    temp[1]=-1*(msg.data[1]-bump_notpushed[1]);
    temp[2]=msg.data[2]-bump_notpushed[2];
    temp[3]=-1*(msg.data[3]-bump_notpushed[3]);

    //normalizing data of sensors
    a=temp[0]*(100.0/(bump_pushed[0]-bump_notpushed[0]));
    b=temp[1]*(100.0/(bump_notpushed[1]-bump_pushed[1]));
    c=temp[2]*(100.0/(bump_pushed[2]-bump_notpushed[2]));
    d=temp[3]*(100.0/(bump_notpushed[3]-bump_pushed[3]));

    //ROS_INFO("I heard a b c d: [%d  %d %d %d]", a,b,c,d);

    temp[4]=msg.data[4]-bump_notpushed[4];
    temp[5]=-1*(msg.data[5]-bump_notpushed[5]);
    temp[6]=msg.data[6]-bump_notpushed[6];
    temp[7]=-1*(msg.data[7]-bump_notpushed[7]);

    //normalizing data of sensors
    e=temp[4]*(100.0/(bump_pushed[4]-bump_notpushed[4]));
    f=temp[5]*(100.0/(bump_notpushed[5]-bump_pushed[5]));
    g=temp[6]*(100.0/(bump_pushed[6]-bump_notpushed[6]));
    h=temp[7]*(100.0/(bump_notpushed[7]-bump_pushed[7]));

    // ROS_INFO("I heard e f g h: [%d  %d %d %d]", e,f,g,h);

    //deleting data with negative sign
    if (a<0){a=0;} if (b<0){b=0;} if (c<0){c=0;} if (d<0){d=0;}
    if (e<0){e=0;} if (f<0){f=0;} if (g<0){g=0;} if (h<0){h=0;}
}

//void FT_left_feedback(const geometry_msgs::Wrench &msg){
//    Fzl=msg.force.z;
//    Mxl=msg.torque.y;
//}

//void FT_right_feedback(const geometry_msgs::Wrench &msg){
//    Fzr=msg.force.z;
//    Mxr=msg.torque.x;
//}

MatrixXd quater2rot(double w,double x,double y, double z){
    MatrixXd R(3,3);
    R<<w*w+x*x-y*y-z*z,2*x*y-2*w*z,2*x*z+2*w*y,
            2*x*y+2*w*z,w*w-x*x+y*y-z*z,2*y*z-2*w*x,
            2*x*z-2*w*y,2*y*z+2*w*x,w*w-x*x-y*y+z*z;
    return R;
}

//*****quaternion to euler params in ankle
double quaternion2euler_pitch(double q0,double q1,double q2,double q3){
    double R11,R32,R33,R31,theta;
    R31=2*(q1*q3-q0*q2);
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    theta=atan2(-R31,sqrt(R32*R32+R33*R33));
    return theta;
}

double quaternion2euler_roll(double q0,double q1,double q2,double q3){
    double phi,R33,R32;
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    phi=atan2(R32,R33);
    return phi;
}



//void imu_data_process(const sensor_msgs::Imu &msg){
//    //MatrixXd R_pelvis(3,3);
//    //R_pelvis=quater2rot(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
////    pelvis_orientation_roll=msg.orientation.x;
////    pelvis_orientation_pitch=msg.orientation.y;
////    pelvis_orientation_yaw=msg.orientation.z;
////    pelvis_acceleration[0]=msg.linear_acceleration.x;
////    pelvis_acceleration[1]=msg.linear_acceleration.y;
////    pelvis_acceleration[2]=msg.linear_acceleration.z;
////    pelvis_angularVelocity[0]=msg.angular_velocity.x;
////    pelvis_angularVelocity[1]=msg.angular_velocity.y;
////    pelvis_angularVelocity[2]=msg.angular_velocity.z;
//    // qDebug()<<pelvis_orientation_roll<<"\t"<<pelvis_orientation_pitch<<"\t"<<pelvis_orientation_yaw;

//}

void StartPhase(){
    if ( GlobalTime<=DurationOfStartPhase) {

        double zStart=0;
        double yStart=0;
        double xStart=0;
        GlobalTime=GlobalTime+OnlineTaskSpace._timeStep;
        //        zStart=OnlineTaskSpace.InitialPelvisHeight+move2pose(OnlineTaskSpace.ReferencePelvisHeight-OnlineTaskSpace.InitialPelvisHeight,GlobalTime,0,DurationOfStartPhase);
        zStart=PelvisCurrentHeight+move2pose(OnlineTaskSpace.ReferencePelvisHeight-PelvisCurrentHeight,GlobalTime,0,DurationOfStartPhase);

        PoseRoot<<xStart,yStart,zStart,0,0,0;
        PoseRFoot<<0,
                -0.11500,
                0.112000,
                0,
                0,
                0;

        PoseLFoot<<0,
                0.11500,
                0.11200,
                0,
                0,
                0;
        if(turning||sidewalk){
//            if (fmod(GlobalTime+.001,.1)<.003){qDebug()<<GlobalTime<<"start 1";
//            MatrixXd temp(1,20);
//                    temp <<PoseRoot.transpose(),NAN,PoseRFoot.transpose(),NAN,PoseLFoot.transpose();

//                   matrix_view(temp);}

            SURENA_turning_side.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA_turning_side.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        }
        else{
//            if (fmod(GlobalTime+.001,.1)<.003){qDebug()<<GlobalTime<<"start 0";
//            MatrixXd temp(1,20);
//                   temp <<PoseRoot.transpose(),NAN,PoseRFoot.transpose(),NAN,PoseLFoot.transpose();
//                   matrix_view(temp);}

            SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        }


        double D_pitch=-1*OnlineTaskSpace.HipPitchModification*(M_PI/180);
        double TstartofPitchModify=DurationOfStartPhase/6;
        double TendofPitchModify=DurationOfStartPhase;
        if (GlobalTime>=TstartofPitchModify && GlobalTime<=TendofPitchModify){
            PitchModified=move2pose(D_pitch,GlobalTime,TstartofPitchModify,TendofPitchModify);
        }
    }
}

void EndPhase(){

    if (GlobalTime>=(DurationOfStartPhase+OnlineTaskSpace.MotionTime)){// && GlobalTime<=DurationOfendPhase+DurationOfStartPhase+OnlineTaskSpace.MotionTime) {

        double zStart=0;
        double yStart=0;
        double xStart=0;
        GlobalTime=GlobalTime+OnlineTaskSpace._timeStep;
        zStart=OnlineTaskSpace.ReferencePelvisHeight+move2pose(OnlineTaskSpace.InitialPelvisHeight-OnlineTaskSpace.ReferencePelvisHeight,GlobalTime,DurationOfStartPhase+OnlineTaskSpace.MotionTime,DurationOfStartPhase+OnlineTaskSpace.MotionTime+DurationOfendPhase);
        // zStart=PelvisCurrentHeight+move2pose(OnlineTaskSpace.InitialPelvisHeight-PelvisCurrentHeight,GlobalTime,DurationOfStartPhase+OnlineTaskSpace.MotionTime,DurationOfStartPhase+OnlineTaskSpace.MotionTime+DurationOfendPhase);

        PoseRoot<<xStart,yStart,zStart,0,0,0;
        PoseRFoot<<0,
                -0.11500,
                OnlineTaskSpace.currentRightFootZ,
                0,
                0,
                0;
        PoseLFoot<<0,
                0.11500,
                OnlineTaskSpace.currentLeftFootZ,
                0,
                0,
                0;
//        if (fmod(GlobalTime+.001,.1)<.003){qDebug()<<GlobalTime<<"end";
//        MatrixXd temp(1,20);
//                temp <<PoseRoot.transpose(),NAN,PoseRFoot.transpose(),NAN,PoseLFoot.transpose();

//               matrix_view(temp);}


        SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
        SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        SURENA_turning_side=SURENA;

        links = SURENA.GetLinks();
        double D_pitch=-1*OnlineTaskSpace.HipPitchModification*(M_PI/180);
        double TstartofPitchModify=DurationOfendPhase/6+DurationOfStartPhase+OnlineTaskSpace.MotionTime;
        double TendofPitchModify=DurationOfendPhase*5/6+DurationOfStartPhase+OnlineTaskSpace.MotionTime;
        double D_time=TendofPitchModify-TstartofPitchModify;
        if (GlobalTime>=TstartofPitchModify && GlobalTime<=TendofPitchModify){
            PitchModified=D_pitch-move2pose(D_pitch,GlobalTime,TstartofPitchModify,TendofPitchModify);
        }
    }
}

void ankleOrientationAdaptationLeft(){
    //parameters of ankle adaptation
    if(a>threshold2 &&b>threshold2&&c>threshold2&&d>threshold2){

    }
    else if(a<threshold &&b<threshold &&c<threshold &&d<threshold){
        //  theta_motor_L->0,phi_motor_L->0
        //        teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
        //        phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
    }

    else if (a>threshold ||b>threshold ||c>threshold ||d>threshold){//left
        //-----------------Pitch left ankle motor control---------------//
        if (abs(b-a)>=abs(c-d)) {
            if (abs(a-b)<120) {teta_motor_L=1*teta_motor_L+k1*(a-b);}
        }
        else {
            if (abs(d-c)<120) {teta_motor_L=1*teta_motor_L+k1*(d-c);}
        }

        //----------------Roll left ankle motor control---------------//
        if (abs(c-b)>=abs(d-a)) {
            if (abs(c-b)<120) {phi_motor_L=1*phi_motor_L+k2*(c-b);}
        }
        else {
            if (abs(a-d)<120) {phi_motor_L=1*phi_motor_L+k2*(d-a);}
        }
    }
    phi_motor_L=saturate(phi_motor_L,-M_PI/90,M_PI/90);
    teta_motor_L=saturate(teta_motor_L,-M_PI/90,M_PI/90);
}

void ankleOrientationAdaptationRight(){
    if(e>threshold2 &&f>threshold2&&g>threshold2&&h>threshold2){

    }
    else if(e<threshold &&f<threshold &&g<threshold &&h<threshold){
        //  theta_motor_L->0,phi_motor_L->0
        //        teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
        //        phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
    }
    else if(e>threshold ||f>threshold ||g>threshold ||h>threshold){//right

        //-----------------Pitch left ankle motor control---------------//
        if (abs(f-e)>=abs(g-h)) {
            if (abs(e-f)<120) {teta_motor_R=1*teta_motor_R+k3*(e-f);}
        }
        else {
            if (abs(h-g)<120) {teta_motor_R=1*teta_motor_R+k3*(h-g);}
        }


        //----------------Roll left ankle motor control---------------//
        if (abs(g-f)>=abs(h-e)) {
            if (abs(g-f)<120) {phi_motor_R=1*phi_motor_R+k4*(g-f);}
        }
        else {
            if (abs(e-h)<120) {phi_motor_R=1*phi_motor_R+k4*(h-e);}
        }

    }


    phi_motor_R=saturate(phi_motor_R,-M_PI/90,M_PI/90);
    teta_motor_R=saturate(teta_motor_R,-M_PI/90,M_PI/90);
    //------------------------saturation of ankle motors----------------------------//


    //        qDebug()<<"a:"<<a<<"\tb:"<<b<<"\tc:"<<c<<"\td:"<<d<<"\ne:"<<e<<"\tf:"<<f<<"\tg:"<<g<<"\th:"<<h;

    //        qDebug()<<"teta_motor_L:"<<teta_motor_L<<"phi_motor_L:"<<phi_motor_L<<
    //"\nteta_motor_R:"<<teta_motor_R<<"phi_motor_R:"<<phi_motor_R;
}

bool FileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    if (check_file.exists() && check_file.isFile()) {
        return true;
    } else {
        return false;
    }
}

QList <QByteArray> GetContent(QByteArray content)
{
    QList <QByteArray> result;
    int index=0;
    while (index<content.length()) {

        index=content.indexOf(":",index);
        if (index<0)
        {
            break;
        }

        QByteArray temp=content.mid(index,content.length());
        QByteArray line=temp.split('\n')[0];
        QList <QByteArray> values=line.split(':');
        if (values.length()>1){
            result.append(values[1]);
        }

        index++;
    }
    return result;
}

MatrixXd ExtractionOfMatrix(QByteArray data)
{
    MatrixXd mat;
    QByteArray insideBrackets=data.split(']')[0];
    insideBrackets= insideBrackets.split('[')[1];
    QList <QByteArray> rows=insideBrackets.split(';');
    QList <QByteArray> columns=rows[0].split(',');

    mat.resize(rows.length(),columns.length());

    //////initial mat values
    for (int i = 0; i < rows.length(); i++) {
        QList <QByteArray> currentCols=rows[i].split(',');
        for (int j = 0; j < currentCols.length(); j++) {
            mat(i,j) = currentCols[j].toDouble();

        }
    }


    return mat;
}

//bool update_qcOffset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
//    qc_update_bool=true;
//    qDebug()<<"update qc offset";
//}


void qc_initial(const sensor_msgs::JointState & msg){
//    for (int i = 0; i <= 12; ++i) {
//        incSensors[i]=int(msg.position[i+1]);

//    }

    if (qc_initial_bool||qc_update_bool){

        for (int i = 0; i <= 31; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }

        if(move_active_offlineData||move_active_offlineTrajectory||move_lower_active||move_upper_activate||qc_update_bool){
            qc_initial_bool=false;
            qc_update_bool=false;

            ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                     qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
                    qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
                    qc_offset[10],qc_offset[11],qc_offset[12],qc_offset[13],qc_offset[14],qc_offset[15],qc_offset[20],qc_offset[21],qc_offset[22],qc_offset[23]);

            ROS_INFO("Initialized!");
        }
    }
}


void publish_data(){
    if(move_upper_activate||move_lower_active||move_active_offlineData||move_active_offlineTrajectory){
        for (int i = 0; i <= 33; ++i) {
            q[i]=q_lower[i]+q_upper[i]+q_offlineData[i];
        }

        q[2]=saturate(q[2],d2r(-18),d2r(12));
        q[8]=saturate(q[8],d2r(-12),d2r(18));
//        q[3]=saturate(q[3],d2r(-38),d2r(30));
//        q[9]=saturate(q[9],d2r(-38),d2r(30));
        q[5]=saturate(q[5],d2r(-46),d2r(45));
        q[11]=saturate(q[11],d2r(-46),d2r(45));


        if(simulation){
            q[15]+=q_rh(0);
            q[16]+=q_rh(1);
            q[18]+=q_rh(3);
            q[22]+=q_lh(0);
            q[23]+=q_lh(1);
            q[25]+=q_lh(3);
        }
        //        qDebug()<<q[0]<<","<<q[1]<<","<<q[2]<<","<<q[3]<<","<<q[4]<<","<<q[5]<<","<<q[6]<<","<<q[7]<<","<<q[8]<<","<<q[9]<<","<<
        //                        q[10]<<","<<q[11]<<","<<q[12]<<","<<q[13]<<","<<q[14]<<","<<q[15]<<","<<q[16]<<","<<q[17]<<","<<q[18]<<","<<q[19]<<","<<
        //                        q[20]<<","<<q[21]<<","<<q[22]<<","<<q[23]<<","<<q[24]<<","<<q[25]<<","<<q[26]<<","<<q[27]<<","<<q[28]<<","<<q[29]<<","<<
        //                        q[30]<<","<<q[31]<<","<<q[32]<<","<<q[23];
        //        qDebug()<<q_upper[0]<<","<<q_upper[1]<<","<<q_upper[2]<<","<<q_upper[3]<<","<<q_upper[4]<<","<<q_upper[5]<<","<<q_upper[6]<<","<<q_upper[7]<<","<<q_upper[8]<<","<<q_upper[9]<<","<<
        //                        q_upper[10]<<","<<q_upper[11]<<","<<q_upper[12]<<","<<q_upper[13]<<","<<q_upper[14]<<","<<q_upper[15]<<","<<q_upper[16]<<","<<q_upper[17]<<","<<q_upper[18]<<","<<q_upper[19]<<","<<
        //                        q_upper[20]<<","<<q_upper[21]<<","<<q_upper[22]<<","<<q_upper[23]<<","<<q_upper[24]<<","<<q_upper[25]<<","<<q_upper[26]<<","<<q_upper[27]<<","<<q_upper[28]<<","<<q_upper[29]<<","<<
        //                        q_upper[30]<<","<<q_upper[31]<<","<<q_upper[32]<<","<<q_upper[23];


        //        qDebug()<<q_lower[0]<<","<<q_lower[1]<<","<<q_lower[2]<<","<<q_lower[3]<<","<<q_lower[4]<<","<<q_lower[5]<<","<<q_lower[6]<<","<<q_lower[7]<<","<<q_lower[8]<<","<<q_lower[9]<<","<<
        //                        q_lower[10]<<","<<q_lower[11]<<","<<q_lower[12]<<","<<q_lower[13]<<","<<q_lower[14]<<","<<q_lower[15]<<","<<q_lower[16]<<","<<q_lower[17]<<","<<q_lower[18]<<","<<q_lower[19]<<","<<
        //                        q_lower[20]<<","<<q_lower[21]<<","<<q_lower[22]<<","<<q_lower[23]<<","<<q_lower[24]<<","<<q_lower[25]<<","<<q_lower[26]<<","<<q_lower[27]<<","<<q_lower[28]<<","<<q_lower[29]<<","<<
        //                        q_lower[30]<<","<<q_lower[31]<<","<<q_lower[32]<<","<<q_lower[23];




if(simulation){

    SendGazebo(q);

}

else{
qref=QC.ctrldata2qc(q);


head_yaw_motor=int((q[29]*180/M_PI)/30*342);//head_yaw
head_roll_motor=int((q[30]*180/M_PI)/30*342);//head_roll
head_pitch_motor=int((q[31]*180/M_PI)/30*342);//head_pitch
WaistYaw_motor=-int((q[13]*180/M_PI)/30*342*4);//WaistYaw
WaistPitch_motor=-int((q[14])*(1/(2*M_PI))*(2304)*100);//WaistPitch ratio 100

q_motor_r[0]=-int(10*(q[15])*180/M_PI*120/60);
q_motor_r[1]=int(10*(q[16])*180/M_PI*120/60);
q_motor_r[2]=-int(7*(q[17])*180/M_PI*100/60);
q_motor_r[3]=int(7*(q[18])*180/M_PI*100/60);

q_motor_r[4]=int((q[19])*(2048)/M_PI);
q_motor_r[5]=int((q[20])*(4000-2050)/(23*M_PI/180));
q_motor_r[6]=int((q[21])*(4000-2050)/(23*M_PI/180));
q_motor_r[7]=int(q[32]);


q_motor_l[0]=int(10*(q[22])*180/M_PI*120/60);
q_motor_l[1]=int(10*(q[23])*180/M_PI*120/60);
q_motor_l[2]=-int(7*(q[24])*180/M_PI*100/60);
q_motor_l[3]=-int(7*(q[25])*180/M_PI*100/60);

q_motor_l[4]=int((q[26])*(2048)/M_PI);
q_motor_l[5]=-int((q[27])*(4000-2050)/(23*M_PI/180));
q_motor_l[6]=int((q[28])*(4000-2050)/(23*M_PI/180));
q_motor_l[7]=int(q[33]);


msg.data.clear();
for(int  i = 0;i < 12;i++)
{
    msg.data.push_back(qref[i]+qc_offset[i]);
}
//right hand epose
for(int  i = 0;i < 8;i++)
{
    msg.data.push_back(q_motor_r[i]+qc_offset[i+12]);
}
for(int  i = 0;i < 8;i++)
{
    msg.data.push_back(q_motor_l[i]+qc_offset[i+20]);
}

msg.data.push_back(WaistPitch_motor+qc_offset[28]);//waist pitch 28
msg.data.push_back(WaistYaw_motor);//waist yaw 29
msg.data.push_back(head_pitch_motor);//head pitch 30
msg.data.push_back(head_roll_motor);//head roll 31
msg.data.push_back(head_yaw_motor);//head yaw 32
chatter_pub.publish(msg);
}

//            for (int var = 0; var < msg.data.size()-1; ++var) {
//                mylog.append(QString::number(msg.data[var])+",");
//            }
//            mylog.append(QString::number(msg.data[msg.data.size()-1])+"\n");


//            for (int var = 1; var < msg.data.size(); ++var) {
//                mylog2.append(QString::number(q[var])+",");
//            }
//            mylog2.append(QString::number(q[msg.data.size()])+"\n");


}

}


bool StartWalk(trajectory_generation::walkRequest &req,trajectory_generation::walkResponse &res)
{


    if(!move_lower_active){


        leftzstop=false;
        rightzstop=false;
        GlobalTime=0;
        stepLength =req.stepLength;
        NStride=req.stepCount;
        OnlineTaskSpace.RightHipRollModification=2;
        OnlineTaskSpace.LeftHipRollModification=2;
        OnlineTaskSpace.FirstHipRollModification=2;
        left_first=req.leftFirst;//right support in first step
        EndPhase_bool=req.endPhase;
        if(EndPhase_bool){
            DurationOfendPhase=3;
        }
        else{
            DurationOfendPhase=0;
        }
        QString note=" ";
        if(left_first){note+="left ";}
        else{note+="right ";}
        turning=false;
        backward=false;
        sidewalk=false;
        JustStartphase=false;
        switch (req.motionID) {

        case 1:
            turning=true;
            TurningRadius=1.5/M_PI;//
            note+="turn(radius=1.5/PI)   number of strides= ";note+=QString::number(NStride);
            stepLength=.15;
            break;
        case 2:
            turning=true;
            TurningRadius=.01;//
            OnlineTaskSpace.RightHipRollModification=3;
            OnlineTaskSpace.LeftHipRollModification=3;
            OnlineTaskSpace.FirstHipRollModification=3;
            note+="on spot turn   number of strides= ";note+=QString::number(NStride);
            break;
        case 3:
            backward=true;
            note+="start backward walk   number of strides= ";note+=QString::number(NStride);
            break;
        case 4:
            sidewalk=true;
            note+="side walk   number of strides= ";note+=QString::number(NStride);
            break;
        case 5:
            note="Just startphase!";
            JustStartphase=true;
            break;

        default:
            turning=false;
            backward=false;
            sidewalk=false;
            note+="start straight walk   step-length= ";note+=QString::number(stepLength);
            note+="   number of strides= ";note+=QString::number(NStride);
            break;
        }
        TaskSpaceOnline3 TaskSpace_temp(NStride,stepLength);
        if(turning || sidewalk) {links = SURENA_turning_side.GetLinks();}
        else{links = SURENA.GetLinks();}

        OnlineTaskSpace=TaskSpace_temp;

        OnlineTaskSpace.StepNumber=1;
        OnlineTaskSpace.localTiming=0;
        DurationOfStartPhase=fabs(OnlineTaskSpace.ReferencePelvisHeight-PelvisCurrentHeight)*3/.05;


        if(turning&&TurningRadius<.2){OnlineTaskSpace.StepLength=TurningRadius/18*M_PI;
            OnlineTaskSpace.XofAnkleMaximumHeight=OnlineTaskSpace.StepLength*1.8;
            OnlineTaskSpace.Xe=0;
            OnlineTaskSpace.Xs=0;
            OnlineTaskSpace.zmp_max=0;
            OnlineTaskSpace.zmp_min=0;
        }



        if(sidewalk||(turning&&TurningRadius<.2)){
            //        OnlineTaskSpace.Xe=0;
            //        OnlineTaskSpace.Xs=0;
            OnlineTaskSpace.side_extra_step_length=true;
            OnlineTaskSpace.CoeffArrayPelvis();
            OnlineTaskSpace.CoeffArrayAnkle();
            OnlineTaskSpace.CoeffSideStartEnd();

        }



        qDebug()<<note<<" End phase:" <<EndPhase_bool;
        move_lower_active=true;
 ros::Rate loop_rate(200);

        while(move_lower_active){
                ros::spinOnce();
            //MotionTime=TStart+NStride*2*Tc+TDs+TEnd;
            MatrixXd RollModified(2,1);RollModified<<0,0;//parameters for hip roll angles charge, for keep pelvis straight
             MatrixXd P;
            //ankle orientation adaptation
            if (GlobalTime>DurationOfStartPhase+OnlineTaskSpace.TStart&&GlobalTime<DurationOfStartPhase+OnlineTaskSpace.MotionTime-OnlineTaskSpace.TEnd-OnlineTaskSpace.TDs) {
                double adapt_time=fmod(GlobalTime-DurationOfStartPhase-OnlineTaskSpace.TStart,2*OnlineTaskSpace.Tc);



                if (adapt_time>OnlineTaskSpace.TDs&&adapt_time<OnlineTaskSpace.Tc+OnlineTaskSpace.TDs/2) {
                    if(left_first){
                        ankleOrientationAdaptationRight();
                    }
                    else{
                        ankleOrientationAdaptationLeft();
                    }

                    //                if (fabs(adapt_time-(OnlineTaskSpace.Tc+OnlineTaskSpace.TDs/2))<.005) {
                    //                    ROS_INFO("%f I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]",GlobalTime, a,b,c,d,e,f,g,h);
                    //        }
                }
                else{
                    if(left_first){
                        teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
                        phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
                    }
                    else{
                        teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
                        phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
                    }



                }
                if (adapt_time<OnlineTaskSpace.TDs/2||adapt_time>OnlineTaskSpace.Tc+OnlineTaskSpace.TDs) {

                    if(left_first){
                        ankleOrientationAdaptationLeft();
                    }
                    else{
                        ankleOrientationAdaptationRight();
                    }

                    //            if (fabs(adapt_time-(OnlineTaskSpace.Tc+OnlineTaskSpace.TDs))<.005){
                    //                ROS_INFO("%f I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]",GlobalTime, a,b,c,d,e,f,g,h);
                    //            }

                }
                else{
                    if(left_first){
                        teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
                        phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
                    }
                    else{
                        teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
                        phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
                    }

                }
            }
            else{
                teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
                phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
                teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
                phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
            }




            StartPhase();

            //Initializing the controller for pelvis
//            if ( GlobalTime<DurationOfStartPhase) {
////                pelvisController.OffsetPitch = pelvis_orientation_pitch;
//                //pelvisController.CPOffsetFinder(pelvis_angularVelocity[1],pelvis_orientation_pitch);
//                //qDebug()<<"Note"<<pelvisController.OffsetPitch;
//            }


            int NumberOfTimeStep=(OnlineTaskSpace.Tc/OnlineTaskSpace._timeStep)+1;

            //-----------------------------------------------------------------------------------------------------//
            //------------------------------- main loop of cyclic walking -----------------------------------------//
            //-----------------------------------------------------------------------------------------------------//

            if (GlobalTime>DurationOfStartPhase && GlobalTime<(DurationOfStartPhase+OnlineTaskSpace.MotionTime)){
                //Ankle Trajectory Replacement

                //Pelvis Vibration Controller
//                outControl = pelvisController.CPController(pelvis_angularVelocity[1],pelvis_orientation_pitch,PelvisVibControlllerCoef,0.005);
//                double checkalai = pelvis_orientation_pitch + 18*pelvis_angularVelocity[1]/M_PI;
//                double pelori = pelvis_orientation_pitch;
//                double pelav = pelvis_angularVelocity[1];
//                alai1.append(QString::number(outControl)+"\n");
//                alai2.append(QString::number(checkalai)+"\n");
//                alai3.append(QString::number(pelori)+"\n");
//                alai4.append(QString::number(pelav)+"\n");

                double m1;//x_al
                double m2;//y_al
                double m3;//z_al
                double m4;//pitch_al
                double m5;//x_ar
                double m6;//y_ar
                double m7;//z_ar
                double m8;//pitch_ar

                GlobalTime=GlobalTime+OnlineTaskSpace._timeStep;

                OnlineTaskSpace.currentLeftFootX2=links[12].PositionInWorldCoordinate(0);
                OnlineTaskSpace.currentLeftFootY2=links[12].PositionInWorldCoordinate(1);
                OnlineTaskSpace.currentLeftFootZ=links[12].PositionInWorldCoordinate(2);

                OnlineTaskSpace.currentRightFootX2=links[6].PositionInWorldCoordinate(0);
                OnlineTaskSpace.currentRightFootY2=links[6].PositionInWorldCoordinate(1);
                OnlineTaskSpace.currentRightFootZ=links[6].PositionInWorldCoordinate(2);



                if ((OnlineTaskSpace.StepNumber==1) && (OnlineTaskSpace.localTiming>=OnlineTaskSpace.TStart) ) {
                    OnlineTaskSpace.localTiming=OnlineTaskSpace._timeStep;//0.001999999999000000;
                    OnlineTaskSpace.localtimingInteger=1;
                    OnlineTaskSpace.StepNumber=OnlineTaskSpace.StepNumber+1;
                }

                else if ((OnlineTaskSpace.localtimingInteger>=NumberOfTimeStep) &&   (OnlineTaskSpace.StepNumber>1    &&   OnlineTaskSpace.StepNumber<(OnlineTaskSpace.NStep+2))) {
                    OnlineTaskSpace.StepNumber=OnlineTaskSpace.StepNumber+1;
                    OnlineTaskSpace.localTiming=OnlineTaskSpace._timeStep;//0.001999999999000000;
                    OnlineTaskSpace.localtimingInteger=1;
                }

                MatrixXd m=OnlineTaskSpace.AnkleTrajectory(OnlineTaskSpace.globalTime,OnlineTaskSpace.StepNumber,OnlineTaskSpace.localTiming);

                m1=m(0,0); m2=m(1,0); m3=m(2,0); m4=m(3,0);
                m5=m(4,0); m6=m(5,0); m7=m(6,0); m8=m(7,0);


                RollModified=OnlineTaskSpace.RollAngleModification(OnlineTaskSpace.globalTime);
                P=OnlineTaskSpace.PelvisTrajectory (OnlineTaskSpace.globalTime);

                OnlineTaskSpace.globalTime=OnlineTaskSpace.globalTime+OnlineTaskSpace._timeStep;
                OnlineTaskSpace.localTiming=OnlineTaskSpace.localTiming+OnlineTaskSpace._timeStep;
                OnlineTaskSpace.localtimingInteger= OnlineTaskSpace.localtimingInteger+1;

                if (OnlineTaskSpace.globalTime<=OnlineTaskSpace.MotionTime){


                    //if you want to have modification of height of pelvis please active the Pz(0,0) instead of P(2,0)

                    if(!AnkleZAdaptation){AnkleZL=m3;AnkleZR=m7;}
                    else{
                        double local_time_cycle=0;
                        //first step just stopping left
                        if(GlobalTime<DurationOfStartPhase+OnlineTaskSpace.TStart){
                            AnkleZR=m7;
                            if(GlobalTime-DurationOfStartPhase<OnlineTaskSpace.Tx+OnlineTaskSpace.Tc){
                                AnkleZL=m3;
                                leftzstop=false;
                                AnkleZ_offsetL=0;
                            }
                            else{

                                if(!leftzstop){

                                    if((left_first&&(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold))||((!left_first)&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold))){
                                        leftzstop=true;
                                        AnkleZL=m3;
                                        AnkleZL+=AnkleZ_offsetL;
                                        AnkleZ_offsetL=AnkleZL-OnlineTaskSpace._lenghtOfAnkle;
                                        qDebug()<<"leftzstop=true AnkleZL="<<AnkleZL<<"offset="<<AnkleZ_offsetL;
                                        ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);
                                    }
                                    else{AnkleZL=m3;
                                        AnkleZL+=AnkleZ_offsetL;
                                    }
                                }

                            }
                        }
                        //cyle z
                        else if(GlobalTime<DurationOfStartPhase+OnlineTaskSpace.TGait+OnlineTaskSpace.TDs){
                            local_time_cycle=fmod(GlobalTime-DurationOfStartPhase-OnlineTaskSpace.TStart,2*OnlineTaskSpace.Tc);

                            //right foot up
                            if(local_time_cycle<=OnlineTaskSpace.TDs+OnlineTaskSpace.TStartofAnkleAdaptation){
                                rightzstop=false;
                                AnkleZR=m7;
                                AnkleZ_offsetR=0;
                            }
                            //right foot stop
                            else if(local_time_cycle<=OnlineTaskSpace.TDs+OnlineTaskSpace.TSS){
                                if(!rightzstop){
                                    if(((!left_first)&&(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold))||((left_first)&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold))){
                                        rightzstop=true;
                                        AnkleZR=m7;
                                        AnkleZR+=AnkleZ_offsetR;
                                        AnkleZ_offsetR=AnkleZR-OnlineTaskSpace._lenghtOfAnkle;
                                        qDebug()<<"rightzstop=true AnkleZR="<<AnkleZR<<"offset="<<AnkleZ_offsetR;
                                        ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);

                                    }
                                    else{AnkleZR=m7;
                                        AnkleZR+=AnkleZ_offsetR;

                                    }
                                }
                            }
                            // takhlie pye rast
                            else if(local_time_cycle<=2*OnlineTaskSpace.Tc-OnlineTaskSpace.TSS+OnlineTaskSpace.TStartofAnkleAdaptation){
                                AnkleZR=OnlineTaskSpace._lenghtOfAnkle+AnkleZ_offsetR-move2pose(AnkleZ_offsetR,local_time_cycle,OnlineTaskSpace.Tc,OnlineTaskSpace.Tc+OnlineTaskSpace.TDs);

                            }

                            // takhlie pye chap
                            if(local_time_cycle<=OnlineTaskSpace.Tc-OnlineTaskSpace.TSS+OnlineTaskSpace.TStartofAnkleAdaptation){
                                AnkleZL=OnlineTaskSpace._lenghtOfAnkle+AnkleZ_offsetL-move2pose(AnkleZ_offsetL,local_time_cycle,0,OnlineTaskSpace.TDs);

                            }
                            //left foot swing
                            else if(local_time_cycle<=OnlineTaskSpace.Tc+OnlineTaskSpace.TDs+OnlineTaskSpace.TStartofAnkleAdaptation){
                                leftzstop=false;
                                AnkleZL=m3;
                                AnkleZ_offsetL=0;
                            }
                            else if(local_time_cycle<=OnlineTaskSpace.Tc+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS){
                                if(!leftzstop){
                                    //left foot stop
                                    if(((left_first)&&(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold))||((!left_first)&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold))){

                                        leftzstop=true;

                                        AnkleZL=m3;
                                        AnkleZL+=AnkleZ_offsetL;
                                        AnkleZ_offsetL=AnkleZL-OnlineTaskSpace._lenghtOfAnkle;
                                        qDebug()<<"leftzstop=true AnkleZL="<<AnkleZL<<"offset="<<AnkleZ_offsetR;
                                        ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);

                                    }

                                    else{AnkleZL=m3;
                                        AnkleZL+=AnkleZ_offsetL;

                                    }
                                }
                            }

                        }
                        else if(GlobalTime<DurationOfStartPhase+OnlineTaskSpace.MotionTime){

                            //right foot up
                            if(GlobalTime<=DurationOfStartPhase+OnlineTaskSpace.TGait+OnlineTaskSpace.TDs+0.5*OnlineTaskSpace.TLastSS){
                                rightzstop=false;
                                AnkleZR=m7;
                                AnkleZ_offsetR=0;
                            }

                            //right foot stop
                            else if(GlobalTime<=DurationOfStartPhase+OnlineTaskSpace.MotionTime-OnlineTaskSpace.TE){
                                if(!rightzstop){
                                    if(((!left_first)&&(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold))||((left_first)&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold))){
                                        rightzstop=true;
                                        AnkleZR=m7;
                                        AnkleZR+=AnkleZ_offsetR;
                                        AnkleZ_offsetR=AnkleZR-OnlineTaskSpace._lenghtOfAnkle;
                                        qDebug()<<"rightzstop=true AnkleZR="<<AnkleZR<<"offset="<<AnkleZ_offsetR;
                                        ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);

                                    }
                                    else{AnkleZR=m7;
                                        AnkleZR+=AnkleZ_offsetR;


                                    }
                                }
                            }
                            // takhlie pye rast
                            else if(GlobalTime<=DurationOfStartPhase+OnlineTaskSpace.MotionTime){
                                AnkleZR=OnlineTaskSpace._lenghtOfAnkle+AnkleZ_offsetR-move2pose(AnkleZ_offsetR,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.MotionTime-OnlineTaskSpace.TE,OnlineTaskSpace.MotionTime-0.66*OnlineTaskSpace.TE);

                            }




                        }
                        else{
                            AnkleZR=m7;
                            AnkleZL=m3;
                        }

                    }

                    PoseRoot<<P(0,0),
                            P(1,0),
                            P(2,0),// Pz(0,0)
                            0,
                            0,
                            0;

                    PoseRFoot<<m5,
                            m6,
                            AnkleZR,
                            0,
                            m8,
                            0;

                    PoseLFoot<<m1,
                            m2,
                            AnkleZL,
                            0,
                            m4,
                            0;

                    double pitch_ar=m8;
                    double pitch_al=m4;

                    if(pitch_ar>=0){
                        PoseRFoot(2)=PoseRFoot(2)-(OnlineTaskSpace._lenghtOfAnkle*(1-cos(pitch_ar)))+OnlineTaskSpace.lf*sin(pitch_ar);
                        PoseRFoot(0)=PoseRFoot(0)+(OnlineTaskSpace.lf*(1-cos(pitch_ar)))+OnlineTaskSpace._lenghtOfAnkle*sin(pitch_ar);}
                    else{
                        PoseRFoot(2)=PoseRFoot(2)-(OnlineTaskSpace._lenghtOfAnkle*(1-cos(pitch_ar)))-OnlineTaskSpace.lb*sin(pitch_ar);//+lb*sin(pitch_ar)
                        PoseRFoot(0)=PoseRFoot(0)-(OnlineTaskSpace.lb*(1-cos(pitch_ar)))+OnlineTaskSpace._lenghtOfAnkle*sin(pitch_ar);} // +_lenghtOfAnkle*sin(pitch_ar)

                    if(pitch_al>=0){
                        PoseLFoot(2)=PoseLFoot(2)-(OnlineTaskSpace._lenghtOfAnkle*(1-cos(pitch_al)))+OnlineTaskSpace.lf*sin(pitch_al);
                        PoseLFoot(0)= PoseLFoot(0)+(OnlineTaskSpace.lf*(1-cos(pitch_al)))+OnlineTaskSpace._lenghtOfAnkle*sin(pitch_al);}
                    else{
                        PoseLFoot(2)=PoseLFoot(2)-(OnlineTaskSpace._lenghtOfAnkle*(1-cos(pitch_al)))-OnlineTaskSpace.lb*sin(pitch_al); //+lb*sin(pitch_al)
                        PoseLFoot(0)=PoseLFoot(0)-(OnlineTaskSpace.lb*(1-cos(pitch_al)))+OnlineTaskSpace._lenghtOfAnkle*sin(pitch_al);} //+_lenghtOfAnkle*sin(pitch_al)


                    if(backward){
                        double backward_coeff=.5;
                        PoseRoot(0,0)=-backward_coeff*PoseRoot(0,0);
                        PoseLFoot(0,0)=-backward_coeff*PoseLFoot(0,0);
                        PoseRFoot(0,0)=-backward_coeff*PoseRFoot(0,0);
                    }


                    MatrixXd R_P(3,3);  MatrixXd R_F_L(3,3);    MatrixXd R_F_R(3,3);
                    R_P=MatrixXd::Identity(3,3);
                    R_F_L=MatrixXd::Identity(3,3);
                    R_F_R=MatrixXd::Identity(3,3);
                    double pelvis_roll=-(PoseRoot(1,0)/OnlineTaskSpace.YpMax)*pelvis_roll_range*M_PI/180;//3 was good
                    R_P<<1,0,0,
                            0,cos(pelvis_roll),-sin(pelvis_roll),
                            0,sin(pelvis_roll),cos(pelvis_roll);


                    R_F_L<<cos(PoseLFoot(4)),0,sin(PoseLFoot(4)),
                            0,1,0,
                            -sin(PoseLFoot(4)),0,cos(PoseLFoot(4));

                    R_F_R<<cos(PoseRFoot(4)),0,sin(PoseRFoot(4)),
                            0,1,0,
                            -sin(PoseRFoot(4)),0,cos(PoseRFoot(4));

                    double alpha=.9;
                    if(sidewalk||(turning&&TurningRadius<.2)){alpha=1;}
                    if(turning&&TurningRadius>=.2){alpha=.95+.05;}
                    double time_margin=.01;
                    double coef_y_la;
                    double coef_y_ra;
                    double coef_y_p;

                    coef_y_la=1-move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.Tx+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS/2+time_margin,OnlineTaskSpace.TStart-time_margin)
                            +move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TGait-OnlineTaskSpace.TSS+time_margin,OnlineTaskSpace.TGait-time_margin);

                    coef_y_ra=1-move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TStart+OnlineTaskSpace.TDs+time_margin,OnlineTaskSpace.TStart+OnlineTaskSpace.Tc-time_margin)
                            +move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TGait+OnlineTaskSpace.TDs+time_margin,OnlineTaskSpace.TGait+OnlineTaskSpace.Tc-time_margin);

                    coef_y_p=1-move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TStart+OnlineTaskSpace.TDs/2+time_margin,OnlineTaskSpace.TStart+OnlineTaskSpace.TDs-time_margin)
                            +move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TGait+OnlineTaskSpace.TDs/2+time_margin,OnlineTaskSpace.TGait+OnlineTaskSpace.TDs-time_margin);





                    PoseLFoot(1)=PoseLFoot(1)*coef_y_la;
                    PoseRFoot(1)=PoseRFoot(1)*coef_y_ra;
                    PoseRoot(1)=PoseRoot(1)*coef_y_p;




                    if(!turning && !sidewalk)
                    {
//                        if (fmod(GlobalTime+.001,.1)<.003){qDebug()<<GlobalTime<<"cycle 0";
//                        MatrixXd temp(1,20);
//                                temp <<PoseRoot.transpose(),NAN,PoseRFoot.transpose(),NAN,PoseLFoot.transpose();

//                               matrix_view(temp);}



                        SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,R_F_L,"Body", PoseRoot,R_P);
                        SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,R_F_R,"Body", PoseRoot,R_P);

                    }


                    right_hand_pitch=0;
                    left_hand_pitch=0;
                    if(!OnlineTaskSpace.side_extra_step_length){
                        right_hand_pitch=-asin(saturate(PoseLFoot(0,0)-PoseRoot(0,0),-.1,1)/.8);
                        left_hand_pitch=-asin(saturate(PoseRFoot(0,0)-PoseRoot(0,0),-.1,1)/.8);

                    right_hand_pitch+=(move2pose(d2r(-13),GlobalTime-DurationOfStartPhase,0,OnlineTaskSpace.TStart)+move2pose(d2r(13),GlobalTime-DurationOfStartPhase-OnlineTaskSpace.MotionTime,-OnlineTaskSpace.TEnd/2,0));
                    left_hand_pitch+=(move2pose(d2r(-13),GlobalTime-DurationOfStartPhase,0,OnlineTaskSpace.TStart)+move2pose(d2r(13),GlobalTime-DurationOfStartPhase-OnlineTaskSpace.MotionTime,-OnlineTaskSpace.TEnd/2,0));
}

                    if(abs(q_ra(0)-q_rh(0))>.01||abs(q_la(0)-q_lh(0))>.01){
                        right_hand_pitch=0;
                        left_hand_pitch=0;
                    }



                    if(turning){

                        //*****************************
//                        if(TurningRadius<.2){
//                            double alpha_turn=1.05;
//                            double beta_turn=1;
//                            double turn_move_coef_yp=1+(beta_turn-1)+move2pose(1-beta_turn,PoseRoot(1,0),-OnlineTaskSpace.YpMax,0)+
//                                    move2pose(alpha_turn-1,PoseRoot(1,0),0,OnlineTaskSpace.YpMax);
//                            PoseRoot(1,0)=turn_move_coef_yp*PoseRoot(1,0);
//                        }


                        double yaw_al,yaw_ar,yaw_p;

                        yaw_p=PoseRoot(0,0)/TurningRadius;
                        double sp=sin(yaw_p);
                        double cp=cos(yaw_p);
                        PoseRoot(0,0)=(TurningRadius-PoseRoot(1,0))*sp;
                        PoseRoot(1,0)=TurningRadius-(TurningRadius-PoseRoot(1,0))*cp;

                        yaw_al=PoseLFoot(0,0)/TurningRadius;
                        double s_al=sin(yaw_al);
                        double c_al=cos(yaw_al);
                        PoseLFoot(0,0)=(TurningRadius-PoseLFoot(1,0))*s_al;
                        PoseLFoot(1,0)=TurningRadius-(TurningRadius-PoseLFoot(1,0))*c_al;

                        yaw_ar=PoseRFoot(0,0)/TurningRadius;
                        double s_ar=sin(yaw_ar);
                        double c_ar=cos(yaw_ar);
                        PoseRFoot(0,0)=(TurningRadius-PoseRFoot(1,0))*s_ar;
                        PoseRFoot(1,0)=TurningRadius-(TurningRadius-PoseRFoot(1,0))*c_ar;





                        R_P<<cos(yaw_p),-sin(yaw_p),0,
                                sin(yaw_p),cos(yaw_p),0,
                                0,0,1;
                        R_F_R<<cos(yaw_ar),-sin(yaw_ar),0,
                                sin(yaw_ar),cos(yaw_ar),0,
                                0,0,1;
                        R_F_L<<cos(yaw_al),-sin(yaw_al),0,
                                sin(yaw_al),cos(yaw_al),0,
                                0,0,1;
                        //*--**********************************************
//                        if (fmod(GlobalTime+.001,.1)<.003){qDebug()<<GlobalTime<<"cycle 2";
//                        MatrixXd temp(1,20);
//                                temp <<PoseRoot.transpose(),NAN,PoseRFoot.transpose(),NAN,PoseLFoot.transpose();

//                               matrix_view(temp);}

                        SURENA_turning_side.doIK("LLeg_AnkleR_J6",PoseLFoot,R_F_L,"Body", PoseRoot,R_P);
                        SURENA_turning_side.doIK("RLeg_AnkleR_J6",PoseRFoot,R_F_R,"Body", PoseRoot,R_P);

                    }

                    if(sidewalk){
                        double side_move_coef=.08/2/OnlineTaskSpace.StepLength;//0.5;


                        //*****************************

                        double alpha_side=1.05;
                        double beta_side=1.05;
                        double side_move_coef_yp=1+(beta_side-1)+move2pose(1-beta_side,PoseRoot(1,0),-OnlineTaskSpace.YpMax,0)+
                                move2pose(alpha_side-1,PoseRoot(1,0),0,OnlineTaskSpace.YpMax);

                        PoseRoot(1,0)=side_move_coef_yp*PoseRoot(1,0)+side_move_coef*PoseRoot(0,0);
                        PoseRoot(0,0)=0;

                        PoseLFoot(1,0)=PoseLFoot(1,0)+side_move_coef*PoseLFoot(0,0);
                        PoseLFoot(0,0)=0;

                        PoseRFoot(1,0)=PoseRFoot(1,0)+side_move_coef*PoseRFoot(0,0);
                        PoseRFoot(0,0)=0;

                        // if(fmod(GlobalTime,.5)<.006){ if (fmod(GlobalTime+.001,.1)<.003){qDebug()<<GlobalTime<<"\t"<<PoseRoot(1)<<"\t"<<PoseRFoot(1)<<"\t"<<PoseLFoot(1)<<endl;}

//                        if (fmod(GlobalTime+.001,.1)<.003){qDebug()<<GlobalTime<<"cycle 3";
//                        MatrixXd temp(1,20);
//                                temp <<PoseRoot.transpose(),NAN,PoseRFoot.transpose(),NAN,PoseLFoot.transpose();

//                               matrix_view(temp);}


                        SURENA_turning_side.doIK("LLeg_AnkleR_J6",PoseLFoot,R_F_L,"Body", PoseRoot,R_P);
                        SURENA_turning_side.doIK("RLeg_AnkleR_J6",PoseRFoot,R_F_R,"Body", PoseRoot,R_P);
                    }
                    // if(sidewalk&&turning){ROS_INFO("unable to turn and walk to side!"); break;}

                }

            }

//            Trajectory.append(QString::number(GlobalTime)+","+
//                              QString::number(PoseRoot(0))+","+
//                              QString::number(PoseRoot(1))+","+
//                              QString::number(PoseRoot(2))+","+
//                              QString::number(PoseRFoot(0))+","+
//                              QString::number(PoseRFoot(1))+","+
//                              QString::number(PoseRFoot(2))+","+
//                              QString::number(PoseLFoot(0))+","+
//                              QString::number(PoseLFoot(1))+","+
//                              QString::number(PoseLFoot(2))+"\n");


            if(GlobalTime>=DurationOfendPhase+DurationOfStartPhase+OnlineTaskSpace.MotionTime||(JustStartphase&&GlobalTime>=DurationOfStartPhase)){
                PelvisCurrentHeight=PoseRoot(2,0);
                move_lower_active=false;
                qDebug()<<"done!";

//                QFile TrajFile("/home/cast/Desktop/Trajectory.txt");
//                TrajFile.remove();
//                TrajFile.open(QFile::ReadWrite);
//                TrajFile.write(Trajectory);
//                TrajFile.close();
//                Trajectory.clear();

//                for (int i = 0; i < 12; ++i) {
//                  qDebug()<<i<<"inc= "<<incSensors[i]<<"   inc-offset= "<<incSensors[i]-qc_offset[i];
//                }

//                qDebug()<<q_lower[0]<<","<<
//                          q_lower[1]<<","<<
//                          q_lower[2]<<","<<
//                          q_lower[3]<<","<<
//                          q_lower[4]<<","<<
//                          q_lower[5]<<","<<
//                          q_lower[6]<<","<<
//                          q_lower[7]<<","<<
//                          q_lower[8]<<","<<
//                          q_lower[9]<<","<<
//                          q_lower[10]<<","<<
//                          q_lower[11];
                //   SURENA_turning_side=SURENA;

            }



            if(turning || sidewalk) {links = SURENA_turning_side.GetLinks();}
            else{links = SURENA.GetLinks();}

            if(EndPhase_bool)
            {EndPhase();
//                outControl = pelvisController.CPController(pelvis_angularVelocity[1],pelvis_orientation_pitch,PelvisVibControlllerCoef,0.005);
//                double checkalai = pelvis_orientation_pitch + 18*pelvis_angularVelocity[1]/M_PI;
//                double pelori = pelvis_orientation_pitch;
//                double pelav = pelvis_angularVelocity[1];
//                alai1.append(QString::number(outControl)+"\n");
//                alai2.append(QString::number(checkalai)+"\n");
//                alai3.append(QString::number(pelori)+"\n");
//                alai4.append(QString::number(pelav)+"\n");
            }



            double k_roll_r=1;
            double k_roll_l=1;
            if(!left_first){
                k_roll_r=OnlineTaskSpace.LeftHipRollModification/OnlineTaskSpace.RightHipRollModification;
                k_roll_l=OnlineTaskSpace.RightHipRollModification/OnlineTaskSpace.LeftHipRollModification;
            }

            cntrl[0]=0.0;
            cntrl[1]=links[1].JointAngle;
            cntrl[2]=links[2].JointAngle+k_roll_r*RollModified(0,0);
            cntrl[3]=links[3].JointAngle+k_pitch*PitchModified;
            cntrl[4]=links[4].JointAngle;
            cntrl[5]=saturate(links[5].JointAngle,-M_PI/5,M_PI/4)+ankle_adaptation_switch*(left_first*teta_motor_R+(!left_first)*teta_motor_L);
            cntrl[6]=links[6].JointAngle+ankle_adaptation_switch*(left_first*phi_motor_R+(!left_first)*phi_motor_L);//roll
            cntrl[7]=links[7].JointAngle;
            cntrl[8]=links[8].JointAngle+k_roll_l*RollModified(1,0);
            cntrl[9]=links[9].JointAngle+k_pitch*PitchModified;
            cntrl[10]=links[10].JointAngle;
            cntrl[11]=saturate(links[11].JointAngle,-M_PI/5,M_PI/4)+ankle_adaptation_switch*(left_first*teta_motor_L+(!left_first)*teta_motor_R);
            cntrl[12]=links[12].JointAngle+ankle_adaptation_switch*(left_first*phi_motor_L+(!left_first)*phi_motor_R);

if (fabs(teta_motor_R)>0.01){
 qDebug()<<"teta_motor_R:"<<teta_motor_R;
}
if (fabs(teta_motor_L)>0.01){
    qDebug()<<"teta_motor_L:"<<teta_motor_L;
}
if (fabs(phi_motor_R)>0.01){
   qDebug()<<"phi_motor_R:"<<phi_motor_R;

}
if (fabs(phi_motor_L)>0.01){
            qDebug()<<"phi_motor_L:"<<phi_motor_L;
  }


            if(!left_first){

                for (int i = 1; i <= 6; ++i) {
                    cntrl[i]= cntrl[i]+cntrl[i+6];
                    cntrl[i+6]=cntrl[i]-cntrl[i+6];
                    cntrl[i]=cntrl[i]-cntrl[i+6];
                }

                //reversing rolls
                cntrl[2]=-cntrl[2];
                cntrl[6]=-cntrl[6];
                cntrl[8]=-cntrl[8];
                cntrl[12]=-cntrl[12];
                //reversing yaws
                cntrl[1]=-cntrl[1];
                cntrl[7]=-cntrl[7];

                right_hand_pitch=right_hand_pitch+ left_hand_pitch;
                left_hand_pitch=right_hand_pitch-left_hand_pitch;
                right_hand_pitch=right_hand_pitch-left_hand_pitch;
            }



            links = SURENA.GetLinks();

            for (int i = 0; i <= 12; ++i) {
                q_lower[i]=cntrl[i];
            }

            for (int i = 13; i <= 33; ++i) {
                q_lower[i]=0;
            }


            q_lower[14] = 0;

            q_lower[15]=right_hand_pitch;
            q_lower[22]=left_hand_pitch;

//            for (int i = 0; i <= 32; ++i) {
//                q_lower[i]=0;
//            }
            //q_lower[14]=saturate(outControl,-10,10)*M_PI/180;
            //            q_lower[3]=saturate(outControl,-10,10)*M_PI/180;
            //            q_lower[9]=saturate(outControl,-10,10)*M_PI/180;
            //q_lower[5]=-saturate(outControl,-10,10)*M_PI/180;
            //q_lower[11]=-saturate(outControl,-10,10)*M_PI/180;

//           if (fmod(GlobalTime+.001,.1)<.003){
//                qDebug()<<GlobalTime;
//            MatrixXd temp(1,20);
//                    temp <<PoseRoot.transpose(),NAN,PoseRFoot.transpose(),NAN,PoseLFoot.transpose();

//                   matrix_view(temp);
//           }




            publish_data();


            loop_rate.sleep();

        }









        res.result=1;

        return true;
    }
    else{
        return false;
    }
}

bool StopWalk(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    double T0=DurationOfStartPhase+OnlineTaskSpace.TStart;
    qDebug()<<GlobalTime;
    if(GlobalTime>T0&&GlobalTime<T0+OnlineTaskSpace.Tc*2*(NStride-1)){
        //       double local_time_stop;
        //       local_time_stop=fmod(GlobalTime-T0,2*OnlineTaskSpace.Tc);
        //       GlobalTime=T0+OnlineTaskSpace.Tc*2*(NStride-1)+local_time_stop;

        int N_Now=floor((GlobalTime-T0)/(2*OnlineTaskSpace.Tc));
        GlobalTime+=(NStride-N_Now-1)*2*OnlineTaskSpace.Tc;
        OnlineTaskSpace.globalTime+=(NStride-N_Now-1)*2*OnlineTaskSpace.Tc;
        OnlineTaskSpace.StepNumber+=(NStride-N_Now-1)*2;



    }
    qDebug()<<"stop walk";
    qDebug()<<GlobalTime;
    return true;
}


bool handMove(trajectory_generation::handmoveRequest &req, trajectory_generation::handmoveResponse &res){

    qDebug()<<"req";
    if(!move_upper_activate){
        scenario_l=req.scenario_l;
        scenario_r=req.scenario_r;
        scenario_hw=req.scenario_hw;
       // talkTime=req.talkTime;
talkTime=1000;

        initializing=true;
        headFinished=false;

 stopTalkbool=false;



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




        for (int var = 0; var < 8; ++var) {
            q_motor_r[var]=0;q_motor_l[var]=0;
        }



        count_upper = 0;
        double time=0.0;
        double time_r,time_l;

        int gest_r=0;int gest_l=0;
        int gest_count_r;
        int gest_count_l;
        VectorXd qr_end(7);VectorXd ql_end(7);

        move_upper_activate=true;


        ros::Rate loop_rate(200);



        while(move_upper_activate){

            ros::spinOnce();
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

                if(scenario_r=="midBall"){

                    r_target_r<<.5,
                            -0.2,
                            -0.3;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,30*M_PI/180,3);
                    r_middle_r<<.48,
                            -.1,
                            -.2;
                    fingers_mode_r=10;
                }
                if(scenario_l=="midBall"){

                    r_target_l<<.5,
                            0.2,
                            -0.3;
                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-30*M_PI/180,3);
                    r_middle_l<<.48,
                            .1,
                            -.2;
                    fingers_mode_l=10;
                }

                // if(scenario_r=="moveFingers"){
                //     r_target_r<<.4,
                //             0.04,
                //             -0.15;
                //     R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
                //      r_middle_r<<.4,
                //              0.04,
                //              -0.15;
                //     fingers_mode_r=19;
                // }

                // if(scenario_l=="moveFingers"){
                //     r_target_l<<.4,
                //             -0.04,
                //             -0.15;
                //     R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
                //     r_middle_l<<.4,
                //             -0.04,
                //             -0.15;
                //     fingers_mode_l=19;
                // }



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

                    r_target_r<<.35,
                            -0.30,
                            -0.26;
                    R_target_r=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,-35*M_PI/180,3);
                    r_middle_r<<.3,
                            -0.1,
                            -0.4;
                    fingers_mode_r=9;


                }
                if(scenario_l=="shakeHands"){

                    r_target_l<<.35,
                            0.3,
                            -0.3;
                    R_target_l=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,35*M_PI/180,3);
                    r_middle_l<<.3,
                            0.1,
                            -0.4;
                    fingers_mode_l=9;

                }

                if(scenario_r=="homeFingers"){

                    r_target_r<<r_right_palm;
                    R_target_r=R_right_palm;
                    r_middle_r<<r_right_palm;
                    fingers_mode_r=9;
                }


                if(scenario_l=="homeFingers"){

                    r_target_l<<r_left_palm;
                    R_target_l=R_left_palm;
                    r_middle_l<<r_left_palm;
                    fingers_mode_l=9;
                }

                if(scenario_r=="homePath"){
                    right_hand handtemp;
                    handtemp.HO_FK_right_palm(q_rh);
                    r_target_r=handtemp.r_right_palm;
                    R_target_r=handtemp.R_right_palm;
                    r_middle_r<<.3,-.3,-.3;
                    fingers_mode_r=9;
                }
                if(scenario_l=="homePath"){
                    left_hand handtemp;
                    handtemp.HO_FK_left_palm(q_lh);

                    r_target_l=handtemp.r_left_palm;
                    R_target_l=handtemp.R_left_palm;
                    r_middle_l<<.3,.3,-.3;
                    fingers_mode_l=9;
                }

                if(scenario_r=="openFingers"){

                    r_target_r<<r_right_palm;
                    R_target_r=R_right_palm;
                    r_middle_r<<r_right_palm;
                    fingers_mode_r=3;
                }


                if(scenario_l=="openFingers"){

                    r_target_l<<r_left_palm;
                    R_target_l=R_left_palm;
                    r_middle_l<<r_left_palm;
                    fingers_mode_l=3;
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
                    r_target_r<<.43,
                            -0.08,
                            -0.38;

                    r_middle_r<<.3,
                            -.2,
                            -.40;

                    R_target_r=hand_funcs.rot(2,-80*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(comeDown)"){
                    r_target_l<<.43,
                            0.08,
                            -0.38;

                    r_middle_l<<.3,
                            .2,
                            -.40;

                    R_target_l=hand_funcs.rot(2,-80*M_PI/180,3);
                    fingers_mode_l=10;
                }

                if(scenario_r=="wh(force)"){
                    r_target_r<<.43,
                            0.01,
                            -0.38;

                    r_middle_r<<.43,
                            -0.05,
                            -0.38;


                    R_target_r=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(3,-10*M_PI/180,3)*hand_funcs.rot(1,8*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(force)"){
                    r_target_l<<.43,
                            -0.01,
                            -0.38;

                    r_middle_l<<.43,
                            0.05,
                            -0.38;

                    R_target_l=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(3,10*M_PI/180,3)*hand_funcs.rot(1,-8*M_PI/180,3);
                    fingers_mode_l=10;
                }

                if(scenario_r=="wh(bringUp)"){
                    r_target_r<<.41,
                            0.01,
                            -0.1;

                    r_middle_r<<.42,
                            0.01,
                            -0.2;


                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-10*M_PI/180,3)*hand_funcs.rot(1,8*M_PI/180,3);
                    fingers_mode_l=10;
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(bringUp)"){
                    r_target_l<<.41,
                            -0.01,
                            -0.1;

                    r_middle_l<<.42,
                            -0.01,
                            -0.2;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,10*M_PI/180,3)*hand_funcs.rot(1,-8*M_PI/180,3);
                    fingers_mode_l=10;
                    fingers_mode_l=10;
                }

                if(scenario_r=="wh(release)"){
                    r_target_r<<.54,
                            -0.05,
                            -0.01;

                    r_middle_r<<.48,
                            0.01,
                            -0.14;


                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-8*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(release)"){
                    r_target_l<<.54,
                            0.05,
                            -0.01;

                    r_middle_l<<.48,
                            -0.01,
                            -0.14;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,8*M_PI/180,3);
                    fingers_mode_l=10;
                }

                if(scenario_r=="wh(release_box)"){
                    r_target_r<<.43,
                            -0.1,
                            -0.1;

                    r_middle_r<<.43,
                            -0.05,
                            -0.1;


                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-8*M_PI/180,3);
                    fingers_mode_r=10;
                }
                if(scenario_l=="wh(release_box)"){
                    r_target_l<<.43,
                            0.1,
                            -0.1;

                    r_middle_l<<.43,
                            0.05,
                            -0.1;

                    R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,8*M_PI/180,3);
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
                }

                    if(scenario_r=="wh(putDown)"){
                        r_target_r<<.45,
                                -0.0,
                                -0.15;

                        r_middle_r<<.45,
                                 -0.00,
                                -0.07;

                        R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-8*M_PI/180,3);
                        fingers_mode_r=10;

                    }
                    if(scenario_l=="wh(putDown)"){
                        r_target_l<<.45,
                                0.00,
                                -0.15;

                        r_middle_l<<.45,
                                0.00,
                                -0.07;

                        R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,8*M_PI/180,3);
                        fingers_mode_r=10;

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
                            0.05,
                            -0.12;
                    r_middle_r<<.35,
                            .0,
                            -.19;

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
                            -0.03,
                            -0.35;
                    R_target_rt[7]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,10*M_PI/180,3);
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
                            0.03,
                            -0.35;
                    R_target_lt[7]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180,3);
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
                R_right_palm=hand0_r.R_right_palm;
                r_left_palm=hand0_l.r_left_palm;
                R_left_palm=hand0_l.R_left_palm;
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

                      SendGazebo(q_init);
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

                if (scenario_r=="homeFingers"||scenario_r=="openFingers") {
                    t_r<<0,2,4;

                }
                if (scenario_l=="homeFingers"||scenario_l=="openFingers") {
                    t_l<<0,2,4;
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
                if(scenario_r=="micDown"){
                    qr_end=q0_r;
                    fingers_mode_r=4;
                }

                if(scenario_l=="home"||scenario_l=="ah"||scenario_l=="lookAtHorizon"){
                    ql_end=q0_l;
                    fingers_mode_l=9;
                }



if(scenario_r=="openFingers"){fingers_mode_r=3;fingers_r=3;}
if(scenario_l=="openFingers"){fingers_mode_l=3;fingers_l=3;}

                if(scenario_l=="lookAtHorizon"){fingers_mode_l=10;fingers_l=10;}
  if(scenario_r=="lookAtHorizon"){fingers_mode_r=10;fingers_r=10;}

                count_upper=0;
                headFinished=false;
                initializing=false;
            }


            else {


                time=double(count_upper)*.005;
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


                    p_r<<   qr_end(0),q_rh(0),
                            qr_end(1),q_rh(1),
                            qr_end(2),q_rh(2),
                            qr_end(3),q_rh(3),
                            qr_end(4),q_rh(4),
                            qr_end(5),q_rh(5),
                            qr_end(6),q_rh(6);


                    for (int i = 0; i < 7; ++i) {
                        q_ra(i)=p_r(i,0)+move2pose(p_r(i,1)-p_r(i,0),time_r,t_h(0),t_h(1));

                    }

                    q_upper[2]=move2zero(q_upper[2],time_r,t_h(1));
                    q_upper[8]=q_upper[2];
                    q_upper[6]=-q_upper[2];
                    q_upper[12]=-q_upper[2];


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
                        R_right_palm=hand_r.R_right_palm;
                        hand_r.doQP(q_ra);
                        q_ra=hand_r.q_next;
                        d_r=hand_r.dist;
                        theta_r=hand_r.theta;
                        sai_r=hand_r.sai;
                        phi_r=hand_r.phi;

                        if(scenario_r=="homeFingers"||scenario_r=="openFingers"){fingers_r=fingers_mode_r;}
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
                        R_right_palm=hand_r.R_right_palm;


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
                                R_right_palm=hand_r.R_right_palm;
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
                                R_right_palm=hand_r.R_right_palm;
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
                                R_right_palm=hand_r.R_right_palm;
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
                                R_right_palm=hand_r.R_right_palm;
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
//                                        q_ra(3)=qr_end(3)-5*M_PI/180*sin((time_r-(t_r(2)+1))/2*(2*M_PI));
                                    q_ra(3)=qr_end(3)-move2pose(5*M_PI/180,time_r,t_r(2)+1,t_r(2)+2)+move2pose(5*M_PI/180,time_r,t_r(2)+2,t_r(2)+3);
                                    }
                                }

                                if(scenario_r=="giveMicBack"){
                                    if(time_r>t_r(2)+1){
                                        fingers_r=3;

                                    }
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

                    p_l<<   ql_end(0),q_lh(0),
                            ql_end(1),q_lh(1),
                            ql_end(2),q_lh(2),
                            ql_end(3),q_lh(3),
                            ql_end(4),q_lh(4),
                            ql_end(5),q_lh(5),
                            ql_end(6),q_lh(6);

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
                        R_left_palm=hand_l.R_left_palm;
                        hand_l.doQP(q_la);

                        q_la=hand_l.q_next;
                        d_l=hand_l.dist;
                        theta_l=hand_l.theta;
                        sai_l=hand_l.sai;
                        phi_l=hand_l.phi;
                        if(scenario_l=="homeFingers"||scenario_l=="openFingers"){fingers_l=fingers_mode_l;}

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
                                R_left_palm=hand_l.R_left_palm;
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
                                R_left_palm=hand_l.R_left_palm;
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
                                R_left_palm=hand_l.R_left_palm;
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
                                R_left_palm=hand_l.R_left_palm;
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
                double WYawRange=8*M_PI/180;
                double WPitchRange=15*M_PI/180;

                if(scenario_hw=="face"){

                    if(n_people!=0){
                        head_yaw+=.00001*(320-X_face);
                        head_pitch-=.00001*(200-Y_face);
                    }
                    n_people=0;
                    X_face=320;
                    Y_face=200;
                    head_yaw=saturate(head_yaw,-45,45);
                    head_pitch=saturate(head_pitch,-15,20);
//                    (scenario_r=="byebye"||scenario_r=="shakeHands")*t_r(2)
                    if(double(count_upper*.005)>=max(8.0,max(t_r(2)+(scenario_r=="byebye"||scenario_r=="shakeHands")*t_r(2),t_l(2)+(scenario_l=="byebye"||scenario_l=="shakeHands")*t_l(2)))){headFinished=true;}
                }

                else if(scenario_hw=="start0"){

                    head_pitch=move2pose(pitchRange-5*M_PI/180,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}

                }
                else if(scenario_hw=="start2"){

                    head_pitch=pitchRange-move2pose(pitchRange,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}
                }

                else if(scenario_hw=="shakeHands"){
                    WaistYaw=move2pose(-WYawRange,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}

                }
                //look at hands


                else if(scenario_hw=="lookAtHands"){
                    head_yaw=move2pose(yawRange-30*M_PI/180,double(count_upper*.005),0,2)+
                            move2pose(-2*(yawRange-30*M_PI/180),double(count_upper*.005),2,6)+
                            move2pose(yawRange-30*M_PI/180,double(count_upper*.005),6,8);
                    q_ra(4)=q0_r(4)+move2pose(-30*M_PI/180,double(count_upper*.005),0,4)+
                            move2pose(30*M_PI/180,double(count_upper*.005),4,8);
                    q_la(4)=q0_l(4)+move2pose(30*M_PI/180,double(count_upper*.005),0,4)+
                            move2pose(-30*M_PI/180,double(count_upper*.005),4,8);
                    fingers_l=fingers_r=19;

                    if(double(count_upper*.005)>=8){headFinished=true;}

                }

                else if(scenario_hw=="lookAtAudience"){
                    head_yaw=move2pose(yawRange-30*M_PI/180,double(count_upper*.005),0,1.5)+
                            move2pose(-2*(yawRange-30*M_PI/180),double(count_upper*.005),1.5,4.5)+
                            move2pose(yawRange-30*M_PI/180,double(count_upper*.005),4.5,6);

                    fingers_l=fingers_r=9;

                    if(double(count_upper*.005)>=6){headFinished=true;}

                }

                else if(scenario_hw=="lookAtHorizon"){
                    head_yaw=move2pose(yawRange-35*M_PI/180,double(count_upper*.005),1,3);
                    //head_pitch=pitchRange-move2pose(pitchRange,double(count_upper*.005),0,2);
                    head_pitch=move2zero(head_pitch,double(count_upper*.005),1);

                    WaistYaw=move2pose(-WYawRange,double(count_upper*.005),1,3);
                    if(double(count_upper*.005)>=3){headFinished=true;}

                }

                else if(scenario_hw=="horizon2"){

                    head_yaw=yawRange-20*M_PI/180-move2pose(yawRange-20*M_PI/180,double(count_upper*.005),0,2);

                    WaistYaw=WYawRange-move2pose(WYawRange,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}

                }

                else if(scenario_hw=="waistBack"){
                    WaistPitch=move2pose(-5*M_PI/180,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}
                }

                else if(scenario_hw=="waistLeft"){
                    WaistYaw=move2pose(15*M_PI/180,double(count_upper*.005),0,3);
                    if(double(count_upper*.005)>=3){headFinished=true;}
                }

                else if(scenario_hw=="footRecog"){

//                    head_pitch=move2pose(pitchRange,double(count_upper*.005),t_l(0)-4,t_l(2));

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

                    q_upper[2]=move2pose(-.1,double(count_upper*.005),0,1.5)+
                            move2pose(.2,double(count_upper*.005),1.5,4.5)+
                            move2pose(-.1,double(count_upper*.005),4.5,6);
                    //qDebug()<<double(count*.005)<<'\t'<<q[2];
                    q_upper[8]=q_upper[2];
                    q_upper[6]=-q_upper[2];
                    q_upper[12]=-q_upper[2];






                    if(double(count_upper*.005)>=8){headFinished=true;}

                }

                else if(scenario_hw=="getMic"){
                    head_yaw=move2pose(-yawRange+25*M_PI/180,double(count_upper*.005),0,4);
                    head_pitch=move2pose(pitchRange-10*M_PI/180,double(count_upper*.005),0,4);
                    if(double(count_upper*.005)>=4){headFinished=true;}

                }
                else  if(scenario_hw=="m3"){
                    head_yaw=-yawRange+25*M_PI/180-move2pose(-yawRange+25*M_PI/180,double(count_upper*.005),0,4);
                    head_pitch=pitchRange-10*M_PI/180-move2pose(pitchRange-10*M_PI/180,double(count_upper*.005),0,4);
                    if(double(count_upper*.005)>=4){headFinished=true;}

                }


                else  if(scenario_hw=="smoothMove"){

                    head_yaw =(yawRange-37*M_PI/180)*sin(double(count_upper*.005)/4*(2*M_PI))*(move2pose(1,double(count_upper*.005),0,.5)-move2pose(1,double(count_upper*.005),talkTime-.5,talkTime));
                    q_upper[2]=.03*sin(double(count_upper*.005)/4*(2*M_PI))*(move2pose(1,double(count_upper*.005),0,.5)-move2pose(1,double(count_upper*.005),talkTime-.5,talkTime));
                    q_upper[8]=q_upper[2];
                    q_upper[6]=-q_upper[2];
                    q_upper[12]=-q_upper[2];

                    if(double(count_upper*.005)>=talkTime){headFinished=true;}
                }

                else if(scenario_hw=="lookAtPresentor"){
                    head_yaw =-move2pose(yawRange*1.2,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}
                }
                else  if(scenario_hw=="confirm"){
                    head_pitch =move2pose(pitchRange-10*M_PI/180,double(count_upper*.005),0,1)-move2pose(pitchRange-10*M_PI/180,double(count_upper*.005),1,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}
                }
                else  if(scenario_hw=="t4"){
                    head_yaw =-yawRange+move2pose(yawRange,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}
                }

                else  if(scenario_hw=="respect"){
                    WaistPitch =move2pose(WPitchRange-5*M_PI/180,double(count_upper*.005),0,2);
                    head_pitch =move2pose(pitchRange-10*M_PI/180,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}
                }
                else  if(scenario_hw=="c2"){
                    WaistPitch =(WPitchRange-5*M_PI/180)-move2pose(WPitchRange-5*M_PI/180,double(count_upper*.005),0,2);
                    head_pitch =pitchRange-10*M_PI/180-move2pose(pitchRange-10*M_PI/180,double(count_upper*.005),0,2);
                    if(double(count_upper*.005)>=2){headFinished=true;}
                }
                else if(scenario_hw=="wh(comeDown)"){

                    double dh=move2pose(.02,double(count_upper*.005),t_r(0),t_r(2));
                    double l=.36+.37-dh;

                    q_upper[4]=acos((l*l-.37*.37-.36*.36)/2/.36/.37);
                    q_upper[3]=-atan(sin(q_upper[4])*.36/(cos(q_upper[4])*.36+.37));
                    q_upper[5]=-q_upper[4]-q_upper[3];
                    q_upper[9]=q_upper[3];
                    q_upper[10]=q_upper[4];
                    q_upper[11]=q_upper[5];
                    //head_pitch=move2pose(10*M_PI/180,double(count_upper*.005),t_r(0),t_r(1));
                    WaistPitch=move2pose(10*M_PI/180,double(count_upper*.005),t_r(0),t_r(2));
                    if(double(count_upper*.005)>=t_r(2)){headFinished=true;}

                }

                else if(scenario_hw=="wh(comeUp)"){

                    double dh=0.02-move2pose(0.02,double(count_upper*.005),t_r(0),t_r(2));
                    double l=.36+.37-dh;

                    q_upper[4]=acos((l*l-.37*.37-.36*.36)/2/.36/.37);
                    q_upper[3]=-atan(sin(q_upper[4])*.36/(cos(q_upper[4])*.36+.37));
                    q_upper[5]=-q_upper[4]-q_upper[3];
                    q_upper[9]=q_upper[3];
                    q_upper[10]=q_upper[4];
                    q_upper[11]=q_upper[5];
                    //balloon=comment
                    // head_pitch=10*M_PI/180-move2pose(10*M_PI/180,double(count*.005),t_r(0),t_r(1));
                    WaistPitch=10*M_PI/180-move2pose(10*M_PI/180,double(count_upper*.005),t_r(0),t_r(2));

//                    q_lower[18]=move2pose(-20*M_PI/180,double(count_upper*.005),t_r(0),t_r(2));
//                    q_lower[25]=q_lower[18];

                    if(double(count_upper*.005)>=t_r(2)){headFinished=true;}

                }
                else if(scenario_hw=="wh(look)"){
                    head_roll=move2pose(10*M_PI/180,double(count_upper*.005),0,1)-2*move2pose(10*M_PI/180,double(count_upper*.005),2,5)+move2pose(10*M_PI/180,double(count_upper*.005),6,7);
                    if(double(count_upper*.005)>=7){headFinished=true;}

                }

                else if(scenario_hw=="wh(release)"){
                    head_pitch=10*M_PI/180+move2pose(-18*M_PI/180,double(count_upper*.005),0,3);
                    if(double(count_upper*.005)>=3){headFinished=true;}

                }

                else if(scenario_hw=="wh3_3"){
                    head_pitch=-6*M_PI/180-move2pose(-6*M_PI/180,double(count_upper*.005),0,3);
                    if(double(count_upper*.005)>=3){headFinished=true;}

                }
                else if(scenario_hw=="home"){
                    double t=double(count_upper*.005);
                    double T_home=4;
                    head_pitch=move2zero(head_pitch,t,T_home);
                    head_roll=move2zero(head_roll,t,T_home);
                    head_yaw=move2zero(head_yaw,t,T_home);
                    WaistPitch=move2zero(WaistPitch,t,T_home);
                    WaistYaw=move2zero(WaistYaw,t,T_home);
                    q_upper[2]=move2zero(q_upper[2],t,T_home);
                    q_upper[8]=q_upper[2];
                    q_upper[6]=-q_upper[2];
                    q_upper[12]=-q_upper[2];



                    if(t>=T_home){headFinished=true;}

                }

                else if(scenario_hw=="WaistPitchTest"){
                    double t=double(count_upper*.005);
                    double th1=14*M_PI/180;
                    double th2=-9*M_PI/180;
                    WaistPitch=move2pose(th1,t,0,4)+move2pose(th2-th1,t,14,22)+move2pose(-th2,t,32,36);

                    if(t>=36){headFinished=true;}
                }




                else{headFinished=true;}


//                if((((scenario_r=="giveMicBack"&&(time_r>=t_r(2)+2))||((scenario_r=="byebye"||scenario_r=="shakeHands")&&(time_r>=2*t_r(2)))||(!((scenario_r=="byebye"||scenario_r=="shakeHands")||scenario_r=="giveMicBack")&&time_r>=t_r(2)))&&time_l>=t_l(2)&&headFinished)||(scenario_l=="talking"&&gest_count_l==int((talkTime-1)/2)+1)||(scenario_r=="talking"&&gest_count_r==int((talkTime-1)/2)+1)) {
                //  if((((scenario_r=="giveMicBack"&&(time_r>=t_r(2)+2))||((scenario_r=="byebye"||scenario_r=="shakeHands")&&(time_r>=2*t_r(2)))||(!((scenario_r=="byebye"||scenario_r=="shakeHands")||scenario_r=="giveMicBack")&&time_r>=t_r(2)))&&time_l>=t_l(2)&&headFinished)||(scenario_l=="talking"&&stopTalkbool)||(scenario_r=="talking"&&stopTalkbool)) {
                      if((((scenario_r=="giveMicBack"&&(time_r>=t_r(2)+2))||(scenario_r=="byebye"&&(time_r>=2*t_r(2))||scenario_r=="shakeHands"&&(time_r>=3+t_r(2)))||(!((scenario_r=="byebye"||scenario_r=="shakeHands")||scenario_r=="giveMicBack")&&time_r>=t_r(2)))&&time_l>=t_l(2)&&headFinished)||(scenario_l=="talking"&&stopTalkbool)||(scenario_r=="talking"&&stopTalkbool)) {

                move_upper_activate=false;
                    // qDebug()<<q_la(0)<<","<<q_la(1)<<","<<q_la(2)<<","<<q_la(3)<<","<<q_la(4)<<","<<q_la(5)<<","<<q_la(6);
                    qDebug()<<"done!";
                }


                //qDebug()<<"r_right_palm";matrix_view(r_right_palm);
                //qDebug()<<"R_right_palm";matrix_view(R_right_palm);
                //qDebug()<<"r_left_palm";matrix_view(r_left_palm);
                //qDebug()<<"R_left_palm";matrix_view(R_left_palm);
                //qDebug()<<"";


            }
            q_upper[13]=WaistYaw;
            q_upper[14]=WaistPitch;
            q_upper[15]=q_ra(0)-q_rh(0);
            q_upper[16]=q_ra(1)-q_rh(1);
            q_upper[17]=q_ra(2)-q_rh(2);
            q_upper[18]=q_ra(3)-q_rh(3);
            q_upper[19]=q_ra(4)-q_rh(4);
            q_upper[20]=q_ra(5)-q_rh(5);
            q_upper[21]=q_ra(6)-q_rh(6);
            q_upper[22]=q_la(0)-q_lh(0);
            q_upper[23]=q_la(1)-q_lh(1);
            q_upper[24]=q_la(2)-q_lh(2);
            q_upper[25]=q_la(3)-q_lh(3);
            q_upper[26]=q_la(4)-q_lh(4);
            q_upper[27]=q_la(5)-q_lh(5);
            q_upper[28]=q_la(6)-q_lh(6);
            q_upper[29]=head_yaw;
            q_upper[30]=head_roll;
            q_upper[31]=head_pitch;


            if(q_upper[32]!=double(fingers_r)&&fingers_r_2counter<10){
                q_upper[32]=2;
                fingers_r_2counter++;
                               // qDebug()<<fingers_r_2counter;
            }
            else {
                q_upper[32]=double(fingers_r);
                fingers_r_2counter=0;
            }

            if(q_upper[33]!=double(fingers_l)&&fingers_l_2counter<10){
                q_upper[33]=2;
                fingers_l_2counter++;
                //                qDebug()<<fingers_r_2counter;
            }
            else {
                q_upper[33]=double(fingers_l);
                fingers_l_2counter=0;
            }

//qDebug()<<q_upper[32];
//if(q_upper[32]==2){
//    qDebug()<<"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";

//}


            count_upper++;

publish_data();
loop_rate.sleep();

            //qDebug()<<q_upper[32]<<"\t"<<time_r;

        }




        res.result=1;
        return true;
    }
    else{
        return false;
    }

}

bool StopTalk(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    talkTime=double(count_upper)*.005+.5;
    stopTalkbool=true;
    qDebug()<<"stop talk";
//    trajectory_generation::handmoveRequest req;
//    trajectory_generation::handmoveResponse res;
//    req.scenario_l="home";
//    req.scenario_r="home";
//    req.scenario_hw="home";
//    move_upper_activate=false;
//    handMove(req,res);
    return true;
}

bool offlineData_start(trajectory_generation::offlineDataRequest &req, trajectory_generation::offlineDataResponse &res){

    qDebug()<<"offlineData requested!";

    if(!move_active_offlineData){

        QString address;


        address = "/home/cast/humanoid/surena4/src/trajectory_generation/src/OfflineWalkData/";

        address +=req.filename.c_str();

        if (!FileExists(address))
        {
            qWarning()<<"Invalid Robot data Path:"<<address;

        }
        QFile file(address);
        file.open(QFile::ReadWrite);
        QByteArray content;
        content = file.readAll();file.close();

        QList<QByteArray> Position= GetContent(content);
        N_data =Position.count();
        for (int i = 0; i < N_data; ++i) {
            positionmat[i]=ExtractionOfMatrix(Position[i]);
            //  matrix_view(positionmat[i]);

        }
        num_data=0;
        qDebug()<<"estimated time="<<double(N_data)*.005;
        move_active_offlineData=true;

ros::Rate loop_rate(200);
        while(move_active_offlineData){
ros::spinOnce();
            q_offlineData[0]=0.0;
            for (int i = 1; i <=12 ; ++i) {
                q_offlineData[i]= positionmat[num_data](0,i-1)-positionmat[0](0,i-1);
            }


            //        if(num%200==0){qDebug()<<q[1]<<q[2]<<q[3]<<q[4]<<q[5]<<q[6]<<q[7]<<q[8]<<q[9]<<q[10]<<q[11]<<q[12];}

            ++num_data;

            if(num_data>=N_data){move_active_offlineData=false;
                qDebug()<<"offlineDataTrajectory done!";}
            publish_data();
            loop_rate.sleep();

        }


        res.result=1;
        return true;

    }
    else{
        return false;
    }
}



bool offlineTrajectory_start(trajectory_generation::offlineTrajectoryRequest &req, trajectory_generation::offlineTrajectoryResponse &res){

    //    qDebug()<<"offlineTrajectory requested!";
    //    if(!move_active_offlineTrajectory){

    //        QString address;


    //        address = "/home/cast/humanoid/surena4/src/trajectory_generation/src/OfflineWalkData/";

    //        address +=req.filename.c_str();

    //        if (!FileExists(address))
    //        {
    //            qWarning()<<"Invalid Robot data Path:"<<address;

    //        }
    //        QFile file(address);
    //        file.open(QFile::ReadWrite);
    //        QByteArray content;
    //        content = file.readAll();file.close();

    //        QList<QByteArray> Position= GetContent(content);
    //        N_Trajectorydata =Position.count();
    //        for (int i = 0; i < N_Trajectorydata; ++i) {
    //            Trajectorymat[i]=ExtractionOfMatrix(Position[i]);

    //        }
    //        for (int i = 1; i < N_Trajectorydata; ++i) {
    //            if(Trajectorymat[i](0,15)==OnlineTaskSpace._lenghtOfAnkle&&Trajectorymat[i-1](0,15)>Trajectorymat[i](0,15)){
    //                offlineTrajectoryTend_ideal=Trajectorymat[i](0,0);
    //                qDebug()<<"offlineTrajectoryTend_ideal="<<offlineTrajectoryTend_ideal;
    //                break;
    //            }
    //        }

    //        num_Trajectorydata=0;

    //        offlineTrajectoryContact=false;
    //        offlineTrajectoryIcontact= N_Trajectorydata ;

    //        qDebug()<<"estimated time="<<double(N_Trajectorydata)*.005;


    //        move_active_offlineTrajectory=true;
    //        res.result=1;
    //        return true;
    //    }
    //    else{
    //        return false;
    //    }



    qDebug()<<"offlineTrajectory requested!";
    if(!move_active_offlineTrajectory){

        QString address;


        address = "/home/cast/humanoid/surena4/src/trajectory_generation/src/OfflineWalkData/";

        address +=req.filename.c_str();

        if (!FileExists(address))
        {
            qWarning()<<"Invalid Robot data Path:"<<address;

        }
        QFile file(address);
        file.open(QFile::ReadWrite);
        QByteArray content;
        content = file.readAll();file.close();

        QList<QByteArray> Position= GetContent(content);
        N_Trajectorydata =Position.count();
        for (int i = 0; i < N_Trajectorydata; ++i) {
            Trajectorymat[i]=ExtractionOfMatrix(Position[i]);

        }
        for (int i = 1; i < N_Trajectorydata; ++i) {
            if(Trajectorymat[i](0,15)==OnlineTaskSpace._lenghtOfAnkle&&Trajectorymat[i-1](0,15)>Trajectorymat[i](0,15)){
                offlineTrajectoryTend_ideal=Trajectorymat[i](0,0);
                break;
            }
        }

        num_Trajectorydata=0;

        offlineTrajectoryContact=false;


        qDebug()<<"estimated time="<<double(N_Trajectorydata)*.005;


        move_active_offlineTrajectory=true;
        ros::Rate looprate(200);

        while(move_active_offlineTrajectory){
            ros::spinOnce();


            double t_offline;
            double RollModified_offline_right;
            double RollModified_offline_left;

            MatrixXd PoseRootOffline(6,1);//position of pelvis respected to global coordinate
            MatrixXd PoseRFootOffline(6,1);//position of right ankle joint respected to global coordinate
            MatrixXd PoseLFootOffline(6,1);//position of left ankle joint respected to global coordinate
            MatrixXd R_P_Offline(3,3);  R_P_Offline=MatrixXd::Identity(3,3);
            MatrixXd R_F_L_Offline(3,3); R_F_L_Offline=MatrixXd::Identity(3,3);
            MatrixXd R_F_R_Offline(3,3); R_F_R_Offline=MatrixXd::Identity(3,3);

            MatrixXd R_P_Offline2(3,3);  R_P_Offline2=MatrixXd::Identity(3,3);
            MatrixXd R_F_L_Offline2(3,3); R_F_L_Offline2=MatrixXd::Identity(3,3);
            MatrixXd R_F_R_Offline2(3,3); R_F_R_Offline2=MatrixXd::Identity(3,3);

            t_offline= Trajectorymat[num_Trajectorydata](0,0);
            for (int i = 0; i < 6; ++i) {
                PoseRootOffline(i,0)=Trajectorymat[num_Trajectorydata](0,i+1);
                PoseRFootOffline(i,0)=Trajectorymat[num_Trajectorydata](0,i+7);
                PoseLFootOffline(i,0)=Trajectorymat[num_Trajectorydata](0,i+13);

            }

            ankladpt.Ankle_adaptation_update(PoseLFootOffline(2,0),PoseRFootOffline(2,0),a,b,c,d,e,f,g,h);
            PoseLFootOffline(2,0)=ankladpt.AnkleZL_output;
            PoseRFootOffline(2,0)=ankladpt.AnkleZR_output;

            RollModified_offline_left=Trajectorymat[num_Trajectorydata](0,19);
            RollModified_offline_right=Trajectorymat[num_Trajectorydata](0,20);

//            if (offlineTrajectoryContact&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold)){
//                offlineTrajectoryContact=false;
//                offlineTrajectoryIcontact=num_Trajectorydata;
//                qDebug()<<"contact detected in t="<<Trajectorymat[offlineTrajectoryIcontact](0,0)<<"\t z="<<Trajectorymat[offlineTrajectoryIcontact](0,15);
//            }
            //            if(!offlineTrajectoryContact){
            //                PoseLFootOffline(2,0)=Trajectorymat[offlineTrajectoryIcontact](0,15)+move2pose(OnlineTaskSpace._lenghtOfAnkle-Trajectorymat[offlineTrajectoryIcontact](0,15),Trajectorymat[num_Trajectorydata](0,0),Trajectorymat[offlineTrajectoryIcontact](0,0),offlineTrajectoryTend_ideal);
            //            }



            //                   R_P_Offline<<1,0,0,
            //                           0,cos(PoseRootOffline(3)*M_PI/180),-sin(PoseRootOffline(3)*M_PI/180),
            //                           0,sin(PoseRootOffline(3)*M_PI/180),cos(PoseRootOffline(3)*M_PI/180);

            //                   R_F_R_Offline<<1,0,0,
            //                           0,cos(PoseRFootOffline(3)*M_PI/180),-sin(PoseRFootOffline(3)*M_PI/180),
            //                           0,sin(PoseRFootOffline(3)*M_PI/180),cos(PoseRFootOffline(3)*M_PI/180);
            //                   R_F_L_Offline<<1,0,0,
            //                           0,cos(PoseLFootOffline(3)*M_PI/180),-sin(PoseLFootOffline(3)*M_PI/180),
            //                           0,sin(PoseLFootOffline(3)*M_PI/180),cos(PoseLFootOffline(3)*M_PI/180);


            R_P_Offline<<1,0,0,
                    0,cos(PoseRootOffline(3)*M_PI/180),-sin(PoseRootOffline(3)*M_PI/180),
                    0,sin(PoseRootOffline(3)*M_PI/180),cos(PoseRootOffline(3)*M_PI/180);

            R_F_R_Offline<<1,0,0,
                    0,cos(PoseRFootOffline(3)*M_PI/180),-sin(PoseRFootOffline(3)*M_PI/180),
                    0,sin(PoseRFootOffline(3)*M_PI/180),cos(PoseRFootOffline(3)*M_PI/180);
            R_F_L_Offline<<1,0,0,
                    0,cos(PoseLFootOffline(3)*M_PI/180),-sin(PoseLFootOffline(3)*M_PI/180),
                    0,sin(PoseLFootOffline(3)*M_PI/180),cos(PoseLFootOffline(3)*M_PI/180);


            R_P_Offline2<<cos(PoseRootOffline(4)*M_PI/180),0,sin(PoseRootOffline(4)*M_PI/180),
                    0,1,0,
                    -sin(PoseRootOffline(4)*M_PI/180), 0,cos(PoseRootOffline(4)*M_PI/180);

            R_F_R_Offline2<<cos(PoseRFootOffline(4)*M_PI/180),0,sin(PoseRFootOffline(4)*M_PI/180),
                    0,1,0,
                    -sin(PoseRFootOffline(4)*M_PI/180),0,cos(PoseRFootOffline(4)*M_PI/180);
            R_F_L_Offline2<<cos(PoseLFootOffline(4)*M_PI/180),0,sin(PoseLFootOffline(4)*M_PI/180),
                    0,1,0,
                    -sin(PoseLFootOffline(4)*M_PI/180),0,cos(PoseLFootOffline(4)*M_PI/180);

            R_P_Offline=R_P_Offline*R_P_Offline2;
            R_F_R_Offline=R_F_R_Offline*R_F_R_Offline2;
            R_F_L_Offline=R_F_L_Offline*R_F_L_Offline2;



            SURENA_Offline.doIK("LLeg_AnkleR_J6",PoseLFootOffline,R_F_L_Offline,"Body", PoseRootOffline,R_P_Offline);
            SURENA_Offline.doIK("RLeg_AnkleR_J6",PoseRFootOffline,R_F_R_Offline,"Body", PoseRootOffline,R_P_Offline);

            //                q_offlineData[0]=0.0;
            //                for (int i = 1; i <=12 ; ++i) {
            //                    q_offlineData[i]= 0;//positionmat[num_data](0,i-1)-positionmat[0](0,i-1);
            //                }


            q_offlineData[0]=0.0;
            q_offlineData[1]=SURENA_Offline.Links[1].JointAngle;
            q_offlineData[2]=SURENA_Offline.Links[2].JointAngle+(!simulation)*RollModified_offline_right*M_PI/180;
            q_offlineData[3]=SURENA_Offline.Links[3].JointAngle;
            q_offlineData[4]=SURENA_Offline.Links[4].JointAngle;
            q_offlineData[5]=saturate(SURENA_Offline.Links[5].JointAngle,-M_PI/.4,M_PI/4)+ankle_adaptation_switch*(left_first*teta_motor_R+(!left_first)*teta_motor_L);
            q_offlineData[6]=SURENA_Offline.Links[6].JointAngle+ankle_adaptation_switch*(left_first*phi_motor_R+(!left_first)*phi_motor_L);//roll
            q_offlineData[7]=SURENA_Offline.Links[7].JointAngle;
            q_offlineData[8]=SURENA_Offline.Links[8].JointAngle+(!simulation)*RollModified_offline_left*M_PI/180;
            q_offlineData[9]=SURENA_Offline.Links[9].JointAngle;
            q_offlineData[10]=SURENA_Offline.Links[10].JointAngle;
            q_offlineData[11]=saturate(SURENA_Offline.Links[11].JointAngle,-M_PI/.4,M_PI/4)+ankle_adaptation_switch*(left_first*teta_motor_L+(!left_first)*teta_motor_R);
            q_offlineData[12]=SURENA_Offline.Links[12].JointAngle+ankle_adaptation_switch*(left_first*phi_motor_L+(!left_first)*phi_motor_R);

            if(num_Trajectorydata%200==0){qDebug()<<q[1]<<q[2]<<q[3]<<q[4]<<q[5]<<q[6]<<q[7]<<q[8]<<q[9]<<q[10]<<q[11]<<q[12];}

            ++num_Trajectorydata;

            if(num_Trajectorydata>=N_Trajectorydata){move_active_offlineTrajectory=false;
                qDebug()<<"offlineTrajectory done!";}

            publish_data();
looprate.sleep();
        }



        res.result=1;
        return true;
    }
    else{
        return false;
    }
}



int main(int argc, char **argv)
{

    if(turning&&TurningRadius<.2){OnlineTaskSpace.StepLength=TurningRadius/18*M_PI;
        OnlineTaskSpace.XofAnkleMaximumHeight=OnlineTaskSpace.StepLength*1.8;
        OnlineTaskSpace.Xe=0;
        OnlineTaskSpace.Xs=0;
        OnlineTaskSpace.zmp_max=0;
        OnlineTaskSpace.zmp_min=0;
    }

    if(sidewalk||(turning&&TurningRadius<.2)){
        //        OnlineTaskSpace.Xe=0;
        //        OnlineTaskSpace.Xs=0;
        OnlineTaskSpace.side_extra_step_length=true;
        OnlineTaskSpace.CoeffArrayPelvis();
        OnlineTaskSpace.CoeffArrayAnkle();
        OnlineTaskSpace.CoeffSideStartEnd();

    }

    bump_initialize=false;
    leftzstop=false;
    rightzstop=false;

    teta_PID_L.Init(.005,.9,-.9,kp,0,0);
    teta_PID_R.Init(.005,.9,-.9,kp,0,0);
    phi_PID_L.Init(.005,.9,-.9,kp,0,0);
    phi_PID_R.Init(.005,.9,-.9,kp,0,0);
    GlobalTime=0;
    DurationOfStartPhase=3;
    DurationOfendPhase=3;
    PelvisCurrentHeight=OnlineTaskSpace.InitialPelvisHeight;
    PitchModified=0;
    PoseRoot.resize(6,1); //pelvis trajectory from taskspace_online,xyzrpy
    PoseRFoot.resize(6,1);//right ankle joint trajectory from taskspace_online,xyzrpy
    PoseLFoot.resize(6,1);//left ankle joint trajectory from taskspace_online,xyzrpy
    PoseRoot<<0,0,PelvisCurrentHeight,0,0,0;
    PoseRFoot<<0,-.115,OnlineTaskSpace._lenghtOfAnkle,0,0,0;
    PoseLFoot<<0,.115,OnlineTaskSpace._lenghtOfAnkle,0,0,0;


    //*******************This part of code is for initialization of joints of the robot for walking**********************************
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

    int count_lower = 0;

    ros::init(argc, argv, "dynamicControlNode");
    ros::NodeHandle nh;

    chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    qcinit = nh.subscribe("/surena/inc_joint_state", 100, qc_initial);
    WalkService = nh.advertiseService("Walk", StartWalk);
    StopService = nh.advertiseService("StopWalk", StopWalk);
    HandMoveService = nh.advertiseService("HandMove", handMove);
    StopTalkService = nh.advertiseService("StopTalk", StopTalk);
    //QCoffsetupdateService = nh.advertiseService("qcOffsetUpdate",update_qcOffset);

//    offlineDataService = nh.advertiseService("offlineData", offlineData_start);
//    offlineTrajectoryService = nh.advertiseService("offlineTrajectory", offlineTrajectory_start);
//    face_sub=nh.subscribe("/ai/face", 100, face_detect);
    sub = nh.subscribe("/surena/bump_sensor_state", 100, receiveFootSensor);
     //imusub = nh.subscribe("/surena/imu_state", 1000, imu_data_process);


ros::Rate loop_rate(200);
    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    //////////////////////////simulation
    if(simulation){
        pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",100);
        pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",100);
        pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",100);
        pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",100);
        pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",100);
        pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",100);
        pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",100);
        pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",100);
        pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",100);
        pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",100);
        pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",100);
        pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",100);
        pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",100);
        pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",100);
        pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",100);
        pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",100);
        pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",100);
        pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",100);
        pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",100);
        pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",100);
        pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",100);
        pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",100);
        pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",100);
        pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",100);
        pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",100);
        pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",100);
        pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",100);
        pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",100);
        pub29 = nh.advertise<std_msgs::Float64>("rrbot/joint29_position_controller/command",100);
        pub30 = nh.advertise<std_msgs::Float64>("rrbot/joint30_position_controller/command",100);
        pub31 = nh.advertise<std_msgs::Float64>("rrbot/joint31_position_controller/command",100);
    }







    for (int i = 0; i < 34; ++i) {
        q[i]=0;
        q_lower[i]=0;
        q_upper[i]=0;
        q_offlineData[i]=0;
    }
    q[32]=9;
    q[33]=9;


    links=SURENA.GetLinks();
    OnlineTaskSpace.StepNumber=1;





    qDebug()<<"start wholebodyDynamicControl2!";


    qc_initial_bool=!simulation;



    while (ros::ok())
    {

        ros::spinOnce();

       loop_rate.sleep();


    }



    return 0;
}



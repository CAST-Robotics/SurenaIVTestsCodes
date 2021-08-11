
#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
//#include"TaskSpace.h"
#include"MinimumJerkInterpolation.h"
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include <Eigen/Geometry>
#include <cstdlib>
//#include <link.h>
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
using namespace  std;
using namespace  Eigen;

bool simulation=!true;

int qc_offset[32];
int qra_offset[4];
int qla_offset[4];
bool qc_initial_bool;

int n_bottle=0;
VectorXd r_bottle_cam(3);
VectorXd r_bottle_shoulder(3);
VectorXd r_bottle_shoulder_initial(3);
double head_yaw=0;
double head_pitch=0;
double head_roll=0;

ros::Publisher pub1  ;ros::Publisher pub2  ;ros::Publisher pub3  ;ros::Publisher pub4  ;
ros::Publisher pub5  ;ros::Publisher pub6  ;ros::Publisher pub7  ;ros::Publisher pub8  ;
ros::Publisher pub9  ;ros::Publisher pub10 ;ros::Publisher pub11 ;ros::Publisher pub12 ;
ros::Publisher pub13 ;ros::Publisher pub14 ;ros::Publisher pub15 ;ros::Publisher pub16 ;
ros::Publisher pub17 ;ros::Publisher pub18 ;ros::Publisher pub19 ;ros::Publisher pub20 ;
ros::Publisher pub21 ;ros::Publisher pub22 ;ros::Publisher pub23 ;ros::Publisher pub24 ;
ros::Publisher pub25 ;ros::Publisher pub26 ;ros::Publisher pub27 ;ros::Publisher pub28 ;
ros::Publisher pub29 ;ros::Publisher pub30 ;ros::Publisher pub31 ;
bool grip_initialize=true;
int n_grip_initialize=0;
bool deliver_init=true;
// waist
double Waist2ArmZ=0.2694;
double Waist2RArmY=-0.235;
double Waist2LArmY=0.235;
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
double X_Object=320;
double Y_Object=260;

void numplot(double num,double min,double max){
    //⬛

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

double norm(VectorXd V){
    double s=0;
    for (int i = 0; i < V.rows(); ++i) {
        s+=V(i)*V(i);
    }
    return sqrt(s);
}
double d2r(double d){return d*M_PI/180;}
double saturate(double a, double min, double max){
    if(a<min){//ROS_INFO("subceeding!");
        return min;}
    else if(a>max){//ROS_INFO("exceeding!");
        return max;}
    else{return a;}
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

void qc_initial(const sensor_msgs::JointState & msg){
    if (qc_initial_bool){

        for (int i = 0; i < 32; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }

        for (int i = 12; i < 16; ++i){
            qra_offset[i-12]=int(msg.position[i+1]);
        }

        for (int i = 20; i < 24; ++i){
            qla_offset[i-20]=int(msg.position[i+1]);
        }


        qc_initial_bool=false;

        ROS_INFO("Offset_feet=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                 qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
                qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
                qc_offset[10],qc_offset[11]);
        ROS_INFO("Offset_righthand=%d\t%d\t%d\t%d",qra_offset[0],qra_offset[1], qra_offset[2],qra_offset[3]);
        ROS_INFO("Offset_lefthand=%d\t%d\t%d\t%d",qla_offset[0],qla_offset[1], qla_offset[2],qla_offset[3]);
        ROS_INFO("Initialized!");
    }
}

VectorXd r_camera2shoulder(VectorXd r_bottle_cam)
{
    double roll2pitch=.1012;
    double yaw2roll=.043;
    VectorXd r_neck(3);
    r_neck<<.00,0.235,0.04;
    VectorXd r_camera(3);
    r_camera<<0.15453-.036,0,.03161+.01768;


    MatrixXd T0(4,4);

    T0<<  cos(M_PI/12),0,sin(M_PI/12),0, 0,1,0,0, -sin(M_PI/12),0,cos(M_PI/12),0,  0,0,0,1;

    MatrixXd T1(4,4);
    T1<<1,0,0,r_camera(0),  0,1,0,r_camera(1),  0,0,1,r_camera(2),  0,0,0,1;
    MatrixXd T2(4,4);
    T2<< cos(head_pitch), 0,sin(head_pitch),0, 0,1,0,0, -sin(head_pitch),0,cos(head_pitch),0,  0,0,0,1;

    MatrixXd T3(4,4);
    T3<<  1,0,0,0,  0,cos(head_roll),-sin(head_roll),0,    0,sin(head_roll),cos(head_roll),roll2pitch,   0,0,0,1;
    MatrixXd T4(4,4);
    T4<<cos(head_yaw),-sin(head_yaw),0,0,  sin(head_yaw),cos(head_yaw),0,0,  0,0,1,yaw2roll,  0,0,0,1;
    MatrixXd T5(4,4);
    T5<<1,0,0,r_neck(0),  0,1,0,r_neck(1),  0,0,1,r_neck(2),  0,0,0,1;
    MatrixXd T(4,4);
    T=T5*T4*T3*T2*T1*T0;
    VectorXd temp(4); temp<<r_bottle_cam,1;
    temp=T*temp;

    return temp.block(0,0,3,1);

}

void object_detect(const geometry_msgs::PoseArray & msg){
    double l=640;
    double w=480;
    double a=.51;
    double b=.385;
    double X0=.493;
    double Y0,Z0,L0,X,Y,Z;
    double x=l/2;
    double y=w/2;
    double z=0;

n_bottle=0;


    int n=msg.poses.size();
    for (int i = 0; i < n; ++i) {
        if (msg.poses[i].orientation.x==39) {
            n_bottle=1;
            x=msg.poses[i].position.x;
            y=msg.poses[i].position.y;
            z=msg.poses[i].position.z;
            X_Object=x;
            Y_Object=y;
            Y0=-(x-l/2)/l*a;
            Z0=-(y-w/2)/w*b;
            L0=sqrt(X0*X0+Y0*Y0+Z0*Z0);
            X=z/L0*X0;
            Y=X*Y0/X0;
            Z=X*Z0/X0;

            if(grip_initialize){
                n_grip_initialize++;


                if(X!=0){
                    if( n_grip_initialize==1){
                        r_bottle_cam<<X,Y,Z;}
                    else{
                        VectorXd temp(3);
                        temp<<X,Y,Z;
                        r_bottle_cam=(r_bottle_cam+temp)/2;
                    }
                }

                r_bottle_shoulder_initial=r_camera2shoulder(r_bottle_cam);

                if(n_grip_initialize>10){
                    grip_initialize=false;
                    qDebug()<<"Bottle initial position is:  "<<r_bottle_shoulder_initial(0)<<", "<<r_bottle_shoulder_initial(1)<<", "<<r_bottle_shoulder_initial(2);
                }
            }


            if(X!=0){ r_bottle_cam<<X,Y,Z;}
            r_bottle_shoulder=r_camera2shoulder(r_bottle_cam);

            double min=.417;
            double max=.58+.1;

            if(norm(r_bottle_shoulder)<min){
                r_bottle_shoulder=r_bottle_shoulder*min/norm(r_bottle_shoulder);
                qDebug()<<"bottle is too close!";
            }
            if(norm(r_bottle_shoulder)>max){
                r_bottle_shoulder=r_bottle_shoulder*max/norm(r_bottle_shoulder);
                qDebug()<<"bottle is too far!";
            }
        break;
        }



    }
   // matrix_view(r_bottle_shoulder);
}

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
double safeX=.2;
double offsetY=-.05;

int main(int argc, char **argv)
{

    QByteArray joint_data;
    qc_initial_bool=!simulation;
    ros::init(argc, argv, "right_grip");
    ros::NodeHandle nh;

    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
    ros::Subscriber object_sub=nh.subscribe("/ai/object", 1000, object_detect);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    double t_r_offset=0;
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

    right_hand hand_funcs;
    right_hand hand_r;

    VectorXd r_target_r(3);
    VectorXd r_deliver_r(3);
    VectorXd r_release_r(3);
    VectorXd r_middle_target1(3);
    VectorXd r_middle_target2(3);
    VectorXd r_middle_deliver(3);
    MatrixXd R_target_r(3,3);
    MatrixXd R_grip_r(3,3);
    MatrixXd R_deliver_r(3,3);
    VectorXd r_right_palm(3);
    MatrixXd R_right_palm(3,3);
    int fingers_mode_r;
    VectorXd q0_r(7);

    double d0_r;
    double d_r ;
    double d_des_r;
    double theta_r;
    double theta_target_r;
    double sai_r;
    double sai_target_r;
    double phi_r;
    double phi_target_r;



    MinimumJerkInterpolation coef_generator;
    MatrixXd X_coef_r;
    MatrixXd Y_coef_r;
    MatrixXd Z_coef_r;
    MatrixXd X_coef_deliver;
    MatrixXd Y_coef_deliver;
    MatrixXd Z_coef_deliver;
    MatrixXd   t_r_init(1,3);
    MatrixXd P_x_r_init(1,3);
    MatrixXd V_x_r_init(1,3);
    MatrixXd A_x_r_init(1,3);
    MatrixXd P_y_r_init(1,3);
    MatrixXd V_y_r_init(1,3);
    MatrixXd A_y_r_init(1,3);
    MatrixXd P_z_r_init(1,3);
    MatrixXd V_z_r_init(1,3);
    MatrixXd A_z_r_init(1,3);

    MatrixXd   T_deliver(1,6);
    MatrixXd   t_deliver(1,7);
    MatrixXd P_x_deliver(1,7);
    MatrixXd P_y_deliver(1,7);
    MatrixXd P_z_deliver(1,7);
    MatrixXd V_x_deliver(1,7);
    MatrixXd V_y_deliver(1,7);
    MatrixXd V_z_deliver(1,7);
    MatrixXd A_x_deliver(1,7);
    MatrixXd A_y_deliver(1,7);
    MatrixXd A_z_deliver(1,7);

    MatrixXd  fingers_r(1,6);
    VectorXd q_ra;

    int q_motor_r[8];
    int q_motor_l[8];
    for (int var = 0; var < 8; ++var) {
        q_motor_r[var]=0;
        q_motor_l[var]=0;
    }

    int head_yaw_motor;
    int head_roll_motor;
    int head_pitch_motor;
    int WaistYaw_motor;
    int WaistPitch_motor;

    vector<double> q(32);

    ros::Rate loop_rate(200);
    int count = 0;
    double time=0.0;
    double time_head_init=0;
    double time_r;
    double time_deliver=0;
    double time_gripping=0;
    bool initializing=true;
    VectorXd qr_end(7);
    double WaistYaw=0;
    double WaistPitch=0;


    q0_r<<13*M_PI/180,
            -10*M_PI/180,
            0,
            -25*M_PI/180,
            0,
            0,
            0;
    q_ra=q0_r;

    double v0_r=0;
    double v_target_r =.4;
    right_hand hand0_r(q0_r,r_target_r,R_target_r,0,0);
    r_right_palm=hand0_r.r_right_palm;
    R_right_palm=hand0_r.R_right_palm;
    d0_r=hand0_r.dist;
    d_r=d0_r;
    d_des_r=hand0_r.d_des;
    theta_r=hand0_r.theta;
    theta_target_r=hand0_r.theta_target;
    sai_r=hand0_r.sai;
    sai_target_r=hand0_r.sai_target;
    phi_r=hand0_r.phi;
    phi_target_r=hand0_r.phi_target;
    hand0_r.HO_FK_right_palm(q0_r);



    if (simulation)
    {
        vector<double> q_init(31);
        for (int i = 0; i < 31; ++i) {
            q_init[i]=0;
        }
        q_init[15]=q_ra(0);
        q_init[16]=q_ra(1);
        q_init[17]=q_ra(2);
        q_init[18]=q_ra(3);
        q_init[19]=q_ra(4);
        q_init[20]=q_ra(5);
        q_init[21]=q_ra(6);

        SendGazebo(q_init);
    }

    r_bottle_shoulder<<.37,
            -0.0,
            -0.3;

    r_bottle_shoulder_initial<<.37,
            -0.0,
            -0.3;

    //QTime chronometer;
    //chronometer.start();
    qDebug()<<"start!";
    while (ros::ok())
    {

        if (qc_initial_bool) {

            ROS_INFO_ONCE("qc is initializing!");
            ros::spinOnce();
            continue;
        }

        else if (grip_initialize&&time_head_init>1){
            ros::spinOnce();
            continue;
        }


        if (time_head_init<=1){
            head_pitch=move2pose(d2r(20),time_head_init,0,1);
            time_head_init+=.005;

        }

        else{
            if(initializing){


                ROS_INFO("q0_r= %f, %f, %f, %f, %f, %f, %f",q0_r(0),q0_r(1),q0_r(2),q0_r(3),q0_r(4),q0_r(5),q0_r(6));
                r_middle_target1<<.2,
                        -.2,
                        -.33;

                r_middle_target2<<r_bottle_shoulder_initial(0)-safeX,
                        r_bottle_shoulder_initial(1)+offsetY,
                        r_bottle_shoulder_initial(2);


                r_target_r<<r_bottle_shoulder_initial(0)-safeX+.12,
                        r_bottle_shoulder_initial(1)+offsetY,
                        r_bottle_shoulder_initial(2);

                r_middle_deliver<<.37,
                        -.07,
                        -.23;

                r_deliver_r<<.37,
                        -0.15,
                        -0.29;

                r_release_r<<.22,
                        -0.15,
                        -0.29;
                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-18*M_PI/180+atan2(r_target_r(1)+offsetY-.05,r_target_r(0)),3);
                R_grip_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-18*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3)/**hand_funcs.rot(3,30*M_PI/180,3)*/;
                R_deliver_r=hand_funcs.rot(2,-100*M_PI/180,3)*hand_funcs.rot(1,atan2(r_deliver_r(1),r_deliver_r(0)),3)/**hand_funcs.rot(3,30*M_PI/180,3)*/;



                //****path generation




                MatrixXd T_right_shoulder_transpose=rightshoulder2waist(WaistYaw,WaistPitch);

                VectorXd temp(4);
                temp<<r_target_r,1;
                r_target_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);

                temp<<r_deliver_r,1;
                r_deliver_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);

                temp<<r_release_r,1;
                r_release_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);
                MatrixXd ord(1,2);
                ord<<5,5;

                t_r_init<<0,4,8;

                P_x_r_init<<hand0_r.r_right_palm(0),r_middle_target1(0),r_middle_target2(0);
                P_y_r_init<<hand0_r.r_right_palm(1),r_middle_target1(1),r_middle_target2(1);
                P_z_r_init<<hand0_r.r_right_palm(2),r_middle_target1(2),r_middle_target2(2);

                R_target_r=T_right_shoulder_transpose.block(0,0,3,3)*R_target_r;

                V_x_r_init<<0,INFINITY,0;
                V_y_r_init<<0,INFINITY,0;
                V_z_r_init<<0,INFINITY,0;
                A_x_r_init<<0,INFINITY,0;
                A_y_r_init<<0,INFINITY,0;
                A_z_r_init<<0,INFINITY,0;

                MatrixXd conx(3,3); conx<<P_x_r_init,V_x_r_init,A_x_r_init;
                MatrixXd cony(3,3); cony<<P_y_r_init,V_y_r_init,A_y_r_init;
                MatrixXd conz(3,3); conz<<P_z_r_init,V_z_r_init,A_z_r_init;

                //            X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
                //            Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
                //            Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

                X_coef_r=coef_generator.Coefficient1(t_r_init,ord,conx,.1).transpose();
                Y_coef_r=coef_generator.Coefficient1(t_r_init,ord,cony,.1).transpose();
                Z_coef_r=coef_generator.Coefficient1(t_r_init,ord,conz,.1).transpose();



                //ROS_INFO("theta_target=%f,sai_target=%f,phi_target=%f",theta_target,sai_target,phi_target);
                //ROS_INFO("\nr_target_r=\n%f\n%f\n%f",r_target_r(0),r_target_r(1),r_target_r(2));
                // ROS_INFO("\nR_target_r=\n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n",R_target_r(0,0),R_target_r(0,1),R_target_r(0,2),R_target_r(1,0),R_target_r(1,1),R_target_r(1,2),R_target_r(2,0),R_target_r(2,1),R_target_r(2,2));
                //            ROS_INFO("press any key to start!");
                //            getch();
                initializing=false;


            }


            else {


                time=double(count)*.005;
                time_r=time;

                VectorXd P_r(3); VectorXd V_r(3);

                if(time_r<=t_r_init(t_r_init.cols()-1)){

                    for(int i=0;i<t_r_init.cols()-1;i++){

                        if(time_r>=t_r_init(i)&&time_r<t_r_init(i+1)){

                            P_r<<coef_generator.GetAccVelPos(X_coef_r.row(i),time_r,0,5)(0,0),
                                    coef_generator.GetAccVelPos(Y_coef_r.row(i),time_r,0,5)(0,0),
                                    coef_generator.GetAccVelPos(Z_coef_r.row(i),time_r,0,5)(0,0);
                            V_r<<coef_generator.GetAccVelPos(X_coef_r.row(i),time_r,0,5)(0,1),
                                    coef_generator.GetAccVelPos(Y_coef_r.row(i),time_r,0,5)(0,1),
                                    coef_generator.GetAccVelPos(Z_coef_r.row(i),time_r,0,5)(0,1);

                            hand_r.update_right_hand(q_ra,V_r,r_middle_target2,R_target_r);
                            r_right_palm=hand_r.r_right_palm;

                            R_right_palm=hand_r.R_right_palm;
                            hand_r.doQP(q_ra);
                            q_ra=hand_r.q_next;
                            d_r=hand_r.dist;
                            theta_r=hand_r.theta;
                            sai_r=hand_r.sai;
                            phi_r=hand_r.phi;
                            qr_end=q_ra;
                            fingers_mode_r=3;
                        }
                    }


                }

                else if (time_gripping<30&&(deliver_init&&(fabs(r_bottle_shoulder(0)-r_right_palm(0)-safeX)>.015||fabs(r_bottle_shoulder(1)+offsetY-r_right_palm(1))>.015||fabs(r_bottle_shoulder(2)-r_right_palm(2))>.015))){
                    // else if (true){
                    double V_factor=1.1;
                    V_r<<V_factor*(r_bottle_shoulder(0)-r_right_palm(0)-safeX),
                            V_factor*(r_bottle_shoulder(1)+offsetY-r_right_palm(1)),
                            V_factor*(r_bottle_shoulder(2)-r_right_palm(2));

                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-18*M_PI/180+atan2(r_bottle_shoulder(1)-offsetY,r_bottle_shoulder(0)),3);
                    R_grip_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-18*M_PI/180+atan2(r_bottle_shoulder(1),r_bottle_shoulder(0)),3)/**hand_funcs.rot(3,30*M_PI/180,3)*/;

                    hand_r.update_right_hand(q_ra,V_r,r_bottle_shoulder,R_target_r);
                    r_right_palm=hand_r.r_right_palm;
                    qDebug()<<"time_gripping: "<<time_gripping;
                    qDebug()<<"r_target_bottle";
                    VectorXd temp(3);
                    temp<<1*(r_bottle_shoulder(0)-safeX),
                            1*(r_bottle_shoulder(1)+offsetY),
                            1*(r_bottle_shoulder(2));
                    matrix_view(temp);
                    qDebug()<<"r_right_palm";
                    matrix_view(r_right_palm);
                    qDebug()<<"V_r";
                    matrix_view(V_r);



                    R_right_palm=hand_r.R_right_palm;
                    hand_r.doQP(q_ra);
                    q_ra=hand_r.q_next;
                    d_r=hand_r.dist;
                    theta_r=hand_r.theta;
                    sai_r=hand_r.sai;
                    phi_r=hand_r.phi;
                    qr_end=q_ra;
                    fingers_mode_r=3;
                    deliver_init=true;
                    r_target_r<<r_bottle_shoulder(0),
                            r_bottle_shoulder(1)+offsetY,
                            r_bottle_shoulder(2);

                    r_middle_deliver=(r_target_r+r_deliver_r)/2;
                    r_middle_deliver(2)+=.1;
                    time_deliver=0;
                    time_gripping+=.005;

                }

                else{

                    if(deliver_init){
                        qDebug()<<"start girping";

                        MatrixXd ord(1,6);
                        ord.fill(5);
//                        t_deliver<<0,1.5,7.5,9,10.5,11.5,13;
                        T_deliver<<2,6,1.5,1.5,1,1.5;
                        t_deliver(0,0)=0;
                        for (int i = 1; i < t_deliver.cols(); ++i) {
                            t_deliver(0,i)=t_deliver(0,i-1)+T_deliver(0,i-1);
                        }


                        P_x_deliver<< hand_r.r_right_palm(0),r_target_r(0),r_target_r(0),r_middle_deliver(0),r_deliver_r(0),r_deliver_r(0),r_release_r(0);
                        P_y_deliver<< hand_r.r_right_palm(1),r_target_r(1),r_target_r(1),r_middle_deliver(1),r_deliver_r(1),r_deliver_r(1),r_release_r(1);
                        P_z_deliver<< hand_r.r_right_palm(2),r_target_r(2),r_target_r(2),r_middle_deliver(2),r_deliver_r(2),r_deliver_r(2),r_release_r(2);
                        V_x_deliver<<0,0,0,INFINITY,0,0,0;
                        V_y_deliver<<0,0,0,INFINITY,0,0,0;
                        V_z_deliver<<0,0,0,INFINITY,0,0,0;
                        A_x_deliver<<0,0,0,INFINITY,0,0,0;
                        A_y_deliver<<0,0,0,INFINITY,0,0,0;
                        A_z_deliver<<0,0,0,INFINITY,0,0,0;


                        MatrixXd conx(3,7); conx<<P_x_deliver,V_x_deliver,A_x_deliver;
                        MatrixXd cony(3,7); cony<<P_y_deliver,V_y_deliver,A_y_deliver;
                        MatrixXd conz(3,7); conz<<P_z_deliver,V_z_deliver,A_z_deliver;

                        X_coef_deliver=coef_generator.Coefficient1(t_deliver,ord,conx,.1).transpose();
                        Y_coef_deliver=coef_generator.Coefficient1(t_deliver,ord,cony,.1).transpose();
                        Z_coef_deliver=coef_generator.Coefficient1(t_deliver,ord,conz,.1).transpose();

                        fingers_r<<3,4,4,4,3,3;




                        deliver_init=false;
                    }

                    else{

                        for(int i=0;i<t_deliver.cols()-1;i++){
                            if(time_deliver>=t_deliver(i)&&time_deliver<t_deliver(i+1)){
                                P_r<<   coef_generator.GetAccVelPos(X_coef_deliver.row(i),time_deliver,0,5)(0,0),
                                        coef_generator.GetAccVelPos(Y_coef_deliver.row(i),time_deliver,0,5)(0,0),
                                        coef_generator.GetAccVelPos(Z_coef_deliver.row(i),time_deliver,0,5)(0,0);
                                V_r<<   coef_generator.GetAccVelPos(X_coef_deliver.row(i),time_deliver,0,5)(0,1),
                                        coef_generator.GetAccVelPos(Y_coef_deliver.row(i),time_deliver,0,5)(0,1),
                                        coef_generator.GetAccVelPos(Z_coef_deliver.row(i),time_deliver,0,5)(0,1);
                                if(i<2){
                                    hand_r.update_right_hand(q_ra,V_r,r_middle_deliver,R_grip_r);
                                }
                                else{
                                    hand_r.update_right_hand(q_ra,V_r,r_middle_deliver,R_deliver_r);
                                }
                                r_right_palm=hand_r.r_right_palm;
                                R_right_palm=hand_r.R_right_palm;
                                hand_r.doQP(q_ra);
                                q_ra=hand_r.q_next;
                                d_r=hand_r.dist;
                                theta_r=hand_r.theta;
                                sai_r=hand_r.sai;
                                phi_r=hand_r.phi;
                                qr_end=q_ra;
                                fingers_mode_r=fingers_r(i);
                            }

                        }
                        if(time_deliver>=t_deliver(t_deliver.cols()-1)){
                            for (int i = 0; i < 7; ++i) {
                                q_ra(i)=q0_r(i)+move2zero(q_ra(i)-q0_r(i),time_deliver-t_deliver(t_deliver.cols()-1),4);
                            }
                            head_yaw=move2zero(head_yaw,time_deliver-t_deliver(t_deliver.cols()-1),4);
                            head_pitch=move2zero(head_pitch,time_deliver-t_deliver(t_deliver.cols()-1),4);

                            fingers_mode_r=9;

                        }
                        time_deliver+=.005;
                    }

                }





            }
            ++count;
        }

  if(n_bottle!=0 && time_deliver<t_deliver(t_deliver.cols()-1)){
        head_yaw+=.00001*(320-X_Object);
        head_pitch-=.00001*(260-Y_Object);
//   qDebug()<<X_Object<<"\t"<<Y_Object;

        head_yaw=saturate(head_yaw,d2r(-45),d2r(45));
        head_pitch=saturate(head_pitch,d2r(-10),d2r(25));
}
  if(time_deliver>=t_deliver(t_deliver.cols()-1)+4){break;}
        for (int i = 0; i < 31; ++i) {
            q[i]=0;
        }


        q[15]=q_ra(0);
        q[16]=q_ra(1);
        q[17]=q_ra(2);
        q[18]=q_ra(3);
        q[19]=q_ra(4);
        q[20]=q_ra(5);
        q[21]=q_ra(6);
        q[14]=0;
        q[13]=0;
        q[29]=head_yaw;
        q[30]=head_roll;
        q[31]=head_pitch;

        //qDebug()<<q[31];

        if(simulation){SendGazebo(q);}

        q_motor_r[0]=-int(10*(q_ra(0)-q0_r(0))*180/M_PI*120/60)+qra_offset[0];
        q_motor_r[1]=int(10*(q_ra(1)-q0_r(1))*180/M_PI*120/60)+qra_offset[1];
        q_motor_r[2]=-int(7*(q_ra(2)-q0_r(2))*180/M_PI*100/60)+qra_offset[2];
        q_motor_r[3]=int(7*(q_ra(3)-q0_r(3))*180/M_PI*100/60)+qra_offset[3];
        q_motor_r[4]=int((q_ra(4)-q0_r(4))*(2048)/M_PI);
        q_motor_r[5]=int((q_ra(5)-q0_r(5))*(4000-2050)/(23*M_PI/180));
        q_motor_r[6]=int((q_ra(6)-q0_r(6))*(4000-2050)/(23*M_PI/180));

        if(q_motor_r[7]!=fingers_mode_r&&q_motor_r[7]!=2){
            q_motor_r[7]=2;

        }
        else{
            q_motor_r[7]=fingers_mode_r;
        }


      //  qDebug()<<q_motor_r[7];

        q_motor_l[0]=qla_offset[0];
        q_motor_l[1]=qla_offset[1];
        q_motor_l[2]=qla_offset[2];
        q_motor_l[3]=qla_offset[3];
        q_motor_l[4]=0;
        q_motor_l[5]=0;
        q_motor_l[6]=0;
        q_motor_l[7]=9;

        head_yaw_motor=int((head_yaw*180/M_PI)/30*342);//head_yaw
        head_roll_motor=int((head_roll*180/M_PI)/30*342);//head_roll
        head_pitch_motor=int((head_pitch*180/M_PI)/30*342);//head_pitch
        WaistYaw_motor=-int((WaistYaw*180/M_PI)/30*342*4);//WaistYaw
        WaistPitch_motor=-int((WaistPitch)*(1/(2*M_PI))*(2304)*100+qc_offset[28]);//WaistPitch ratio 100


        msg.data.clear();
        for(int  i = 0;i < 12;i++)
        {
            msg.data.push_back(qc_offset[i]);
        }
        //right hand epose
        msg.data.push_back(q_motor_r[0]);//12 -y  a,z
        msg.data.push_back(q_motor_r[1]);//13 +x
        msg.data.push_back(q_motor_r[2]);//14 -z
        msg.data.push_back(q_motor_r[3]);//15 +y
        //right hand dynamixel + fingers
        msg.data.push_back(q_motor_r[4]);//16
        msg.data.push_back(q_motor_r[5]);//17
        msg.data.push_back(q_motor_r[6]);//18
        msg.data.push_back(q_motor_r[7]);//19
        //left hand epose
        msg.data.push_back(q_motor_l[0]);//20 +y
        msg.data.push_back(q_motor_l[1]);//21 +x
        msg.data.push_back(q_motor_l[2]);//22 -z
        msg.data.push_back(q_motor_l[3]);//23 -y
        //left hand dynamixel + fingers
        msg.data.push_back(q_motor_l[4]);//24
        msg.data.push_back(q_motor_l[5]);//25
        msg.data.push_back(q_motor_l[6]);//26
        msg.data.push_back(q_motor_l[7]);//27

        msg.data.push_back(WaistPitch_motor+qc_offset[28]);//waist pitch 28
        msg.data.push_back(WaistYaw_motor);//waist yaw 29
        msg.data.push_back(head_pitch_motor);//head pitch 30
        msg.data.push_back(head_roll_motor);//head roll 31
        msg.data.push_back(head_yaw_motor);//head yaw 32

        if(!simulation){chatter_pub.publish(msg);}


        ros::spinOnce();
        loop_rate.sleep();
    }




    return 0;
}

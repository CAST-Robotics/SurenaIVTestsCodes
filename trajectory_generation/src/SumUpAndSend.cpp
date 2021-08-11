
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



using namespace  std;
using namespace  Eigen;

bool simulation=!true;
vector<double> q(34);
vector<double> q_upper(34);
vector<double> q_lower(34);
vector<double> q_offlineData(34);
int qc_offset[40];
bool qc_initial_bool;
QCgenerator QC;
ros::Publisher pub1  ;ros::Publisher pub2  ;ros::Publisher pub3  ;ros::Publisher pub4  ;
ros::Publisher pub5  ;ros::Publisher pub6  ;ros::Publisher pub7  ;ros::Publisher pub8  ;
ros::Publisher pub9  ;ros::Publisher pub10 ;ros::Publisher pub11 ;ros::Publisher pub12 ;
ros::Publisher pub13 ;ros::Publisher pub14 ;ros::Publisher pub15 ;ros::Publisher pub16 ;
ros::Publisher pub17 ;ros::Publisher pub18 ;ros::Publisher pub19 ;ros::Publisher pub20 ;
ros::Publisher pub21 ;ros::Publisher pub22 ;ros::Publisher pub23 ;ros::Publisher pub24 ;
ros::Publisher pub25 ;ros::Publisher pub26 ;ros::Publisher pub27 ;ros::Publisher pub28 ;
ros::Publisher pub29 ;ros::Publisher pub30 ;ros::Publisher pub31 ;



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

void get_trajectory_data_lower(const std_msgs::Float32MultiArray & msg){
    for (int i = 0; i <= 33; ++i) {
        q_lower[i]= msg.data[i];
    }
}
void get_trajectory_data_offlineData(const std_msgs::Float32MultiArray & msg){
    for (int i = 0; i <= 33; ++i) {
        q_offlineData[i]= msg.data[i];
    }
}

void get_trajectory_data_upper(const std_msgs::Float32MultiArray & msg){
    for (int i = 0; i <= 33; ++i) {
        q_upper[i]= msg.data[i];
    }
    VectorXd q0_r(7);
    VectorXd q0_l(7);
    q0_r<<13*M_PI/180, -10*M_PI/180, 0, -25*M_PI/180, 0, 0, 0;
    q0_l<<13*M_PI/180, 10*M_PI/180, 0, -25*M_PI/180, 0, 0, 0;
if(simulation){
    q_upper[15]+=q0_r(0);
    q_upper[16]+=q0_r(1);
    q_upper[18]+=q0_r(3);
    q_upper[22]+=q0_l(0);
    q_upper[23]+=q0_l(1);
    q_upper[25]+=q0_l(3);
}

}

void qc_initial(const sensor_msgs::JointState & msg){
    if (qc_initial_bool){

        for (int i = 0; i <= 31; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }


        qc_initial_bool=false;

        ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                 qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
                qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
                qc_offset[10],qc_offset[11],qc_offset[12],qc_offset[13],qc_offset[14],qc_offset[15],qc_offset[20],qc_offset[21],qc_offset[22],qc_offset[23]);

        ROS_INFO("Initialized!");
    }
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

int main(int argc, char **argv)
{

    qc_initial_bool=!simulation;

    ros::init(argc, argv, "sumup");
    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
    ros::Subscriber get_trajectory_data_lower_sub = nh.subscribe("/trajectory_data_lower", 1000,get_trajectory_data_lower );
    ros::Subscriber get_trajectory_data_upper_sub = nh.subscribe("/trajectory_data_upper", 1000,get_trajectory_data_upper );
    ros::Subscriber get_trajectory_data_offlineData_sub = nh.subscribe("/trajectory_data_offlineData", 1000,get_trajectory_data_offlineData );

    std_msgs::MultiArrayDimension msg_dim;

    for (int i = 0; i < 34; ++i) {
        q[i]=0;
        q_lower[i]=0;
        q_upper[i]=0;
        q_offlineData[i]=0;
    }
    q[32]=9;
    q[33]=9;
    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
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

    int q_motor_r[8];
    int q_motor_l[8];
    vector<int> qref(12);
    int head_yaw_motor,head_pitch_motor,head_roll_motor,WaistYaw_motor,WaistPitch_motor;

    qDebug()<<"start!";
QByteArray mylog;
QByteArray mylog2;
    while (ros::ok())
    {


        if (qc_initial_bool) {
            ROS_INFO_ONCE("qc is initializing!");
            ros::spinOnce();
            continue;
        }

        for (int i = 0; i <= 33; ++i) {
            q[i]=q_lower[i]+q_upper[i]+q_offlineData[i];
        }
        //qDebug()<<q[0]<<","<<q[1]<<","<<q[2]<<","<<q[3]<<","<<q[4]<<","<<q[5]<<","<<q[6]<<","<<q[7]<<","<<q[8]<<","<<q[9]<<","<<q[10]
        //             <<","<<q[11]<<","<<q[12]<<","<<q[13]<<","<<q[14]<<","<<q[15]<<","<<q[16]<<","<<q[17]<<","<<q[18]<<","<<q[19]<<","<<q[20]
        //              <<","<<q[21]<<","<<q[22]<<","<<q[23]<<","<<q[24]<<","<<q[25]<<","<<q[26]<<","<<q[27]<<","<<q[28]<<","<<q[29]<<","<<q[30];


        if(simulation){
            SendGazebo(q);
        }



            qref=QC.ctrldata2qc(q);


            head_yaw_motor=int((q[29]*180/M_PI)/30*342);//head_yaw
            head_roll_motor=int((q[30]*180/M_PI)/30*342);//head_roll
            head_pitch_motor=int((q[31]*180/M_PI)/30*342);//head_pitch
            WaistYaw_motor=int((q[13]*180/M_PI)/30*342/4);//WaistYaw
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
//            if(q_motor_r[7]==2){
//            qDebug()<<"panje 2";}



//            for (int var = 0; var < msg.data.size()-1; ++var) {
//                mylog.append(QString::number(msg.data[var])+",");
//            }
//            mylog.append(QString::number(msg.data[msg.data.size()-1])+"\n");


//            for (int var = 1; var < msg.data.size(); ++var) {
//                mylog2.append(QString::number(q[var])+",");
//            }
//            mylog2.append(QString::number(q[msg.data.size()])+"\n");


if(!simulation){
            chatter_pub.publish(msg);

//VectorXd temp(33);
//for (int i = 0; i < 33; ++i) {
//    temp(i)=msg.data[i];
//}
//matrix_view(temp);
           }
      ros::spinOnce();
        loop_rate.sleep();
    }
//QFile myfile("/media/cast/UBUNTU1604/scenarioDataCheck.txt");
//myfile.remove();
//myfile.open(QFile::ReadWrite);
//myfile.write(mylog);
//myfile.close();
//QFile myfile2("/media/cast/UBUNTU1604/scenarioAnglesCheck.txt");
//myfile2.remove();
//myfile2.open(QFile::ReadWrite);
//myfile2.write(mylog2);
//myfile2.close();


    return 0;
}

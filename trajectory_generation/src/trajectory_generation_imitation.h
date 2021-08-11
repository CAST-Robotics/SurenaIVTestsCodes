#ifndef IMITATION_H
#define IMITATION_H

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
#include<std_msgs/Int32.h>
#include<std_msgs/Float32MultiArray.h>
#include<geometry_msgs/PoseArray.h>
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
#include "ankle_adaptation.h"

using namespace  std;
using namespace  Eigen;

bool simulation=false;
bool mirror=!false;

double lengthOfThigh=0.3700;
double lengthOfShank=0.3600;
double lenghtOfAnkle=0.112;
double lengthOfHip=0.10900;
double pelvisLength=0.23;
double ReferencePelvisHeight=0.91;
double InitialPelvisHeight=0.9510;
double t_foot=0;

vector<double> q(34);
double q_lower[34];
double q_upper[34];
double GlobalTime=0;
double t_initial=0;
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
ros::Publisher chatter_pub;
std_msgs::Int32MultiArray msg;
std_msgs::MultiArrayDimension msg_dim;
int a,b,c,d,e,f,g,h;//data of left foot sensor
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

VectorXd q_rh(7);
VectorXd q_lh(7);
VectorXd q_ra(7);
VectorXd q_la(7);
double head_yaw=0;
double head_pitch=0;
double head_roll=0;
double WaistYaw=0;
double WaistPitch=0;
int q_motor_r[8];
int q_motor_l[8];
vector<int> qref(12);
int head_yaw_motor,head_pitch_motor,head_roll_motor,WaistYaw_motor,WaistPitch_motor;
Robot SURENA;

double RollModified_offline_right;
double RollModified_offline_left;
MatrixXd PoseRoot(6,1);//position of pelvis respected to global coordinate
MatrixXd PoseRFoot(6,1);//position of right ankle joint respected to global coordinate
MatrixXd PoseLFoot(6,1);//position of left ankle joint respected to global coordinate
MatrixXd R_P(3,3);
MatrixXd R_F_L(3,3);
MatrixXd R_F_R(3,3);

MatrixXd R_P2(3,3);
MatrixXd R_F_L2(3,3);
MatrixXd R_F_R2(3,3);
calc cal;

int rhID=0;
int legID=0;
int lhID=0;

double trhf=0;
double trhs=0;
double trhh=0;
double tlhf=0;
double tlhs=0;
double tlhh=0;

double t_idle=0;
double t_home=0;



bool bump_initialize=true;
int bump_pushed[8];
int bump_notpushed[8];


Ankle_adaptation ankladpt;
double Ymax=.15;
double Ypmax=.135;
double Zmax=.07;
double Alphamax=10;
double Rollmodmax=4;
bool leg_motion_is_done=true;
int motion_ID;
double X_face=320;
double Y_face=200;
int n_people;



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



    bump_pushed[0]=1104;bump_pushed[1]= 834;bump_pushed[2]=3135;bump_pushed[3]=3003;
    bump_pushed[4]=3119;bump_pushed[5]=2917;bump_pushed[6]=1203;bump_pushed[7]=911;

    bump_notpushed[0]=1010;bump_notpushed[1]= 929;bump_notpushed[2]=3033;bump_notpushed[3]=3097;
    bump_notpushed[4]=3034;bump_notpushed[5]=3006;bump_notpushed[6]=1107;bump_notpushed[7]=1015;


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

void face_detect(const geometry_msgs::PoseArray & msg){
    n_people=msg.poses.size();

    double temp_x,temp_y;
    double d=480*480+640*640;


    if(n_people!=0) {

        for (int i = 0; i < n_people; ++i) {
         temp_x=msg.poses[i].position.x;
         temp_y=msg.poses[i].position.y;
         if(d>(pow(X_face-temp_x,2)+pow(Y_face-temp_y,2))){
             d=pow(X_face-temp_x,2)+pow(Y_face-temp_y,2);
             X_face=msg.poses[i].position.x;
             Y_face=msg.poses[i].position.y;
         }


        }
    }

    else{
        X_face+=1e-6*(320-X_face);
        Y_face+= 1e-6*(200-X_face);
    }

}

double saturate(double a, double min, double max){
    if(a<min){//ROS_INFO("subceeding!");
        return min;}
    else if(a>max){//ROS_INFO("exceeding!");
        return max;}
    else{return a;}
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

void publish_data(){

        for (int i = 0; i <= 33; ++i) {
            q[i]=q_upper[i]+q_lower[i];
        }

//        q[2]=cal.saturate(q[2],cal.d2r(-18),cal.d2r(12));
//        q[8]=cal.saturate(q[8],cal.d2r(-12),cal.d2r(18));
//        q[3]=cal.saturate(q[3],cal.d2r(-38),cal.d2r(30));
//        q[9]=cal.saturate(q[9],cal.d2r(-38),cal.d2r(30));
//        q[5]=cal.saturate(q[5],cal.d2r(-36),cal.d2r(45));
//        q[11]=cal.saturate(q[11],cal.d2r(-36),cal.d2r(45));


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




//    ros::Rate loop_rate(200);
//    loop_rate.sleep();
}

void motion_detect(const geometry_msgs::PoseArray& msg){
   if(t_initial>3){
       int code=int(msg.poses[0].position.x);
       int temp;
       //legs
       temp=code%100;
       if(temp==0||temp==1||temp==2||temp==10||temp==20){
           if(legID!=temp&&leg_motion_is_done){
               if(legID==0&&temp==1){motion_ID=1;}
               if(legID==0&&temp==2){motion_ID=2;}
               if(legID==0&&temp==10){motion_ID=3;}
               if(legID==0&&temp==20){motion_ID=4;}

               if(legID==1&&temp==0){motion_ID=5;}
               if(legID==2&&temp==0){motion_ID=6;}
               if(legID==10&&temp==0){motion_ID=7;}
               if(legID==20&&temp==0){motion_ID=8;}

               if(legID==1&&temp==2){motion_ID=9;}
               if(legID==2&&temp==1){motion_ID=10;}
               if(legID==10&&temp==20){motion_ID=11;}
               if(legID==20&&temp==10){motion_ID=12;}

               legID=temp;
               t_foot=0;
           }

    }
       //hands
       temp=((code-code%100)%1000)/100;
       if(rhID!=temp && temp==0){trhh=0;}
       if(rhID!=temp && temp==1){trhs=0;}
       if(rhID!=temp && temp==2){trhf=0;}

       rhID=temp;
       temp=((code-code%1000)%10000)/1000;
       if(lhID!=temp && temp==0){tlhh=0;}
       if(lhID!=temp && temp==1){tlhs=0;}
       if(lhID!=temp && temp==2){tlhf=0;}
       lhID=temp;




//qDebug()<<"code: "<<code<<"\tlegID: "<<legID<<"\trhID: "<<rhID<<"\tlhID: "<<lhID;
   }
}

void left_right_switch_hands(){
    for (int i = 0; i < 5; ++i) {
        double temp=q_upper[i+15];
        q_upper[i+15]=q_upper[i+22];
        q_upper[i+22]=temp;
    }
    q_upper[16]=-q_upper[16];
    q_upper[17]=-q_upper[17];
    q_upper[19]=-q_upper[19];
    q_upper[23]=-q_upper[23];
    q_upper[24]=-q_upper[24];
    q_upper[26]=-q_upper[26];
}


void left_right_switch_legs(){

    for (int i = 0; i < 6; ++i) {
        double temp=PoseLFoot(i,0);
        PoseLFoot(i,0)=PoseRFoot(i,0);
        PoseRFoot(i,0)=temp;
    }

   PoseRoot(1,0)=-PoseRoot(1,0);
   PoseRoot(3,0)=-PoseRoot(3,0);
   PoseRoot(5,0)=-PoseRoot(5,0);
   PoseRFoot(1,0)=-PoseRFoot(1,0);
   PoseRFoot(3,0)=-PoseRFoot(3,0);
   PoseRFoot(5,0)=-PoseRFoot(5,0);
   PoseLFoot(1,0)=-PoseLFoot(1,0);
   PoseLFoot(3,0)=-PoseLFoot(3,0);
   PoseLFoot(5,0)=-PoseLFoot(5,0);


   RollModified_offline_left=RollModified_offline_left+RollModified_offline_right;
   RollModified_offline_right=RollModified_offline_left-RollModified_offline_right;
   RollModified_offline_left=RollModified_offline_left-RollModified_offline_right;
   RollModified_offline_left=-RollModified_offline_left;
   RollModified_offline_right=-RollModified_offline_right;

}


void foot_move(bool side,bool right,bool home,double T){
    leg_motion_is_done=t_foot>=T;
    if(!leg_motion_is_done){
        double t1_p,t2_p,t1_a,t2_a,t1rm,t2rm;
        if(!home){
            t1_p=0;
            t2_p=.6*T;
            t1_a=.6*T;
            t2_a=T;
            t1rm=.5*T;//t1_a;
            t2rm=.8*T;//t2_a;
        }
        else{
            t1_p=.4*T;
            t2_p=T;
            t1_a=0;
            t2_a=.4*T;
            t1rm=.48*T;
            t2rm=.8*T;
        }

if(!right){left_right_switch_legs();}

        PoseRoot(0,0)=0;
        PoseRoot(1,0)=(Ypmax*!home)+cal.move2zero(PoseRoot(1,0)-(Ypmax*!home),t_foot-t1_p,t2_p-t1_p);//cal.move2pose(Ypmax,t_foot,t0,t1);
        PoseRoot(2,0)=ReferencePelvisHeight;
        PoseRoot(3,0)=0;//for side zhangooler add sth
        PoseRoot(4,0)=0;
        PoseRoot(5,0)=0;


        PoseLFoot(0,0)=0;
        PoseLFoot(1,0)=.115;
        PoseLFoot(2,0)=lenghtOfAnkle;
        PoseLFoot(3,0)=0;
        PoseLFoot(4,0)=0;
        PoseLFoot(5,0)=0;



        if(side){
            PoseRFoot(0,0)=cal.move2zero(PoseRFoot(0,0),t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(1,0)=-.115-(Ymax*!home)+cal.move2zero(PoseRFoot(1,0)-(-.115-(Ymax*!home)),t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(3,0)=(-Alphamax*!home)+cal.move2zero(PoseRFoot(3,0)-(-Alphamax*!home),t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(4,0)=cal.move2zero(PoseRFoot(4,0),t_foot,1);
        }
        else{
            PoseRFoot(0,0)=(Ymax*!home)+cal.move2zero(PoseRFoot(0,0)-(Ymax*!home),t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(1,0)=-.115+cal.move2zero(PoseRFoot(1,0)+.115,t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(3,0)=cal.move2zero(PoseRFoot(3,0),t_foot,1);
            PoseRFoot(4,0)=(-Alphamax*!home)+cal.move2zero(PoseRFoot(4,0)-(-Alphamax*!home),t_foot-t1_a,t2_a-t1_a);
        }

        PoseRFoot(2,0)=lenghtOfAnkle+(Zmax*!home)+cal.move2zero(PoseRFoot(2,0)-(lenghtOfAnkle+(Zmax*!home)),t_foot-t1_a,t2_a-t1_a);
        PoseRFoot(5,0)=0;

        //RollModified_offline_left=cal.move2pose(Rollmodmax,t_foot,t1rm,t2rm);

        RollModified_offline_left=(Rollmodmax*!home)+cal.move2zero(RollModified_offline_left-(Rollmodmax*!home),t_foot-t1rm,t2rm-t1rm);
//left_right_switch_legs();
////left_right_switch_legs();
///
//qDebug()<<"z_a= "<<PoseRFoot(2,0);
//qDebug()<<"z_p= "<<PoseRoot(2,0);
        if(!right){left_right_switch_legs();}

//        qDebug()<<"PoseRoot,PoseRFoot,PoseLFoot";
//        MatrixXd tempp;
//        tempp=PoseRoot.transpose();        cal.matrix_view(tempp);
//        tempp=PoseRFoot.transpose();       cal.matrix_view(tempp);
//        tempp=PoseLFoot.transpose();       cal.matrix_view(tempp);
    }
}

void foot_move2(bool front2side,bool right,double T){
leg_motion_is_done=t_foot>=T;
if(!leg_motion_is_done){
if(!right){left_right_switch_legs();}
if(front2side){
    PoseRFoot(0,0)=cal.move2zero(PoseRFoot(0,0),t_foot,T);
    PoseRFoot(1,0)=-.115-(Ymax)+cal.move2zero(PoseRFoot(1,0)-(-.115-(Ymax)),t_foot,T);
    PoseRFoot(3,0)=(-Alphamax)+cal.move2zero(PoseRFoot(3,0)-(-Alphamax),t_foot,T);
    PoseRFoot(4,0)=cal.move2zero(PoseRFoot(4,0),t_foot,1);
}
else{
    PoseRFoot(0,0)=(Ymax)+cal.move2zero(PoseRFoot(0,0)-(Ymax),t_foot,T);
    PoseRFoot(1,0)=-.115+cal.move2zero(PoseRFoot(1,0)+.115,t_foot,T);
    PoseRFoot(3,0)=cal.move2zero(PoseRFoot(3,0),t_foot,1);
    PoseRFoot(4,0)=(-Alphamax)+cal.move2zero(PoseRFoot(4,0)-(-Alphamax),t_foot,T);
}
 if(!right){left_right_switch_legs();}
}

}

#endif //IMITATION_H

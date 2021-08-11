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
#include "trajectory_generation/walk.h"
#include "trajectory_generation/walkRequest.h"
#include "trajectory_generation/walkResponse.h"

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include "trajectory_generation/handmove.h"
#include "trajectory_generation/handmoveRequest.h"
#include "trajectory_generation/handmoveResponse.h"
#include "trajectory_generation/offlineData.h"
#include "trajectory_generation/offlineDataRequest.h"
#include "trajectory_generation/offlineDataResponse.h"


using namespace  std;
using namespace  Eigen;

QVector<std::string> scenario_l;
QVector<std::string> scenario_r;
QVector<std::string> scenario_hw;
QVector<bool> lowerbodyReq;
QVector<bool> upperbodyReq;
QVector<double> duration;
QVector<int> stepCount;
QVector<double>stepLength;
QVector<int>motionID;
QVector<bool>leftFirst;
QVector<bool>endPhase;

void scenario_append(bool upperbodyRequest,bool lowerbodyRequest,double _duration, std::string scenario_left,std::string scenario_right,std::string scenario_headwaist,int motion_id,double _steplegth,int _stepcount,bool left_first,bool endphase){

   upperbodyReq.append(upperbodyRequest);
   lowerbodyReq.append(lowerbodyRequest);
   duration.append(_duration);

   scenario_l.append(scenario_left);
   scenario_r.append(scenario_right);
   scenario_hw.append(scenario_headwaist);
   stepCount.append(_stepcount);
   stepLength.append(_steplegth);
   motionID.append(motion_id);
   leftFirst.append(left_first);
   endPhase.append(endphase);
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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "scenario_call");
    ros::NodeHandle nh;

    ros::ServiceClient walkClient=nh.serviceClient<trajectory_generation::walk>("/Walk");
    ros::ServiceClient handClient=nh.serviceClient<trajectory_generation::handmove>("/HandMove");
ros::Rate loop_rate(200);

 trajectory_generation::handmove srvHand;
 trajectory_generation::walk srvWalk;





scenario_append(true,false,2,"null","null","start0",0,0,0,true,true);//0
scenario_append(true,false,2,"null","null","home",0,0,0,true,true);//1
scenario_append(true,false,8,"handRecog","handRecog","footRecog",0,0,0,true,true);//2
scenario_append(true,false,4,"null","null","lookAtHands",0,0,0,true,true);//3
scenario_append(true,false,4,"lookAtHorizon","homePath","lookAtHorizon",0,0,0,true,true);//4
scenario_append(true,false,4,"home","home","home",0,0,0,true,true);//5
scenario_append(true,false,4,"null","null","wh(comeDown)",0,0,0,true,true);//5
scenario_append(false,true,3,"","","",5,0.25,3,true,false);//6
//wholebody
scenario_append(true,true,4,"wh(comeDown)","wh(comeDown)","",5,0.25,3,true,false);
scenario_append(true,false,4,"wh(force)","wh(force)","",5,0.25,3,true,false);
scenario_append(true,false,6,"wh(bringUp)","wh(bringUp)","wh(look)",5,0.25,3,true,false);
scenario_append(true,false,4,"null","null","wh(comeUp)",5,0.25,3,true,false);//balloon
scenario_append(true,false,4,"wh(release)","wh(release)","wh(release)",5,0.25,3,true,false);//balloon
scenario_append(true,false,4,"home","home","home",5,0.25,3,true,false);//balloon
//scenario_append(true,false,4,"null","null","wh2",5,0.25,3,true,false);
//scenario_append(false,true,15,"","","",2,0.25,2,true,false);////////
//scenario_append(true,false,4,"wh4","wh4","wh(comeDown)",5,0.25,3,true,false);
//scenario_append(true,false,4,"wh5","wh5","",5,0.25,3,true,false);
//scenario_append(true,false,4,"home","home","wh2",5,0.25,3,true,false);
//scenario_append(false,true,15,"","","",2,0.25,2,false,false);////////
scenario_append(false,true,20,"","","",0,0.25,3,true,false);//6    +1
scenario_append(true,false,2,"home","home","face",0,0,0,true,true);//7   +1
scenario_append(true,false,8,"null","shakeHands","face",0,0,0,true,true);//8   +1
scenario_append(true,false,1,"null","openFingers","face",0,0,0,true,true);//8   +1
scenario_append(true,false,4,"null","home","face",0,0,0,true,true);//9   +1
scenario_append(true,false,4,"null","getMic","getMic",0,0,0,true,true);//10   +1
scenario_append(true,false,4,"null","micDown","home",0,0,0,true,true);//11   +1
scenario_append(false,true,20,"","","",2,0.25,2,true,false);//12   +1
scenario_append(true,false,4,"home","micUp","",0,0,0,true,true);//13     +1
scenario_append(true,false,12,"talking","null","smoothMove",0,0,0,true,true);//14    +1
scenario_append(true,false,4,"home","micDown","lookAtPresentor",0,0,0,true,true);//15    +1
scenario_append(true,false,4,"null","null","face",0,0,0,true,true);//9
scenario_append(true,false,4,"null","micUp","home",0,0,0,true,true);//16   +1
scenario_append(true,false,12,"talking","null","smoothMove",0,0,0,true,true);//17    +1
scenario_append(true,false,4,"home","null","lookAtPresentor",0,0,0,true,true);//18    +1
scenario_append(true,false,4,"null","null","face",0,0,0,true,true);//9
scenario_append(true,false,4,"null","null","confirm",0,0,0,true,true);//19    +1
scenario_append(true,false,4,"null","giveMicBack","",0,0,0,true,true);//20     +1
scenario_append(true,false,4,"null","home","home",0,0,0,true,true);//21    +1
//zhangooler                                                       +1
scenario_append(true,false,16,"talking","talking","smoothMove",0,0,0,true,true);//22   +1
scenario_append(true,false,4,"home","home","respect",0,0,0,true,true);//23
scenario_append(true,false,4,"null","byebye","home",0,0,0,true,true);//24    +1
scenario_append(true,true,20,"null","home","",1,0.15,2,true,false);//25  +1
scenario_append(false,true,20,"","","",0,0.25,2,true,false);//26   +1
scenario_append(false,true,20,"","","",2,0.25,4,false,true);//27   +1






int num=-1;
double time=0;
    while (ros::ok())
    {
if(time==0){
        num++;
        if( num>=scenario_l.count()){
            qDebug()<<"finish!";
            break;
        }
        qDebug()<<"press any key!";
        getch();

        srvWalk.request.motionID=motionID[num];
        srvWalk.request.leftFirst=leftFirst[num];
        srvWalk.request.stepCount=stepCount[num];
        srvWalk.request.stepLength=stepLength[num];
        srvWalk.request.endPhase=endPhase[num];
        srvHand.request.scenario_l=scenario_l[num];
        srvHand.request.scenario_r=scenario_r[num];
        srvHand.request.scenario_hw=scenario_hw[num];
        srvHand.request.talkTime=duration[num];
        if(lowerbodyReq[num]){
           qDebug()<<num<< "stepCount="<<stepCount[num]<<"\tstepLength="<<stepLength[num]<<"motionID="<<motionID[num];
          walkClient.call(srvWalk);
        }
        if(upperbodyReq[num]){
            qDebug()<<num<<"\tscenario_l"<<scenario_l[num].c_str()<<"\tscenario_r"<<scenario_r[num].c_str()<<"\tscenario_hw"<<scenario_hw[num].c_str();
        handClient.call(srvHand);
        }
}
time+=.005;
if(time>duration[num]+.5){time=0;}
ros::spinOnce();
loop_rate.sleep();
    }


    return 0;
}



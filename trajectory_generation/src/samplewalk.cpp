#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <QEventLoop>
#include <qdebug.h>
#include <QTimer>
#include <QApplication>
#include <std_msgs/Int32MultiArray.h>

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

bool StartWalk(trajectory_generation::walkRequest &req,trajectory_generation::walkResponse &res)
{
  ros::Rate loop_rate(1);
    for(int i=0;i<10;i++){
           ROS_INFO("walk!");
    loop_rate.sleep();
        ros::spinOnce();

    }


}
//void Epos::WaitMs(int ms)
//{
//    QEventLoop q;
//    QTimer tT;
//    tT.setSingleShot(true);
//    connect(&tT, SIGNAL(timeout()), &q, SLOT(quit()));
//    tT.start(ms);
//    q.exec();
//    if(tT.isActive()){

//        tT.stop();
//    } else {

//    }


//}
int main(int argc, char **argv)
{
    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);

   // w.show();
//   app.connect(&w, SIGNAL(, &app, SLOT(quit()));
    ros::init(argc, argv, "wlaksample");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    ros::ServiceServer WalkService = nh.advertiseService("Walk", StartWalk);

    ROS_INFO("started!");
qDebug()<<"start ros loop";
    while (ros::ok())
    {

            ROS_INFO("working!");
        ros::spinOnce();
        loop_rate.sleep();
    }
    qDebug()<<"end ros loop";
   return 0;
   //return app.exec();


}

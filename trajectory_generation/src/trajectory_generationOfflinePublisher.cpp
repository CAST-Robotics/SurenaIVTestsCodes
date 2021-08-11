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
#include<trajectory_generation/offlineData.h>
#include<trajectory_generation/offlineDataRequest.h>
#include<trajectory_generation/offlineDataResponse.h>


using namespace  std;
using namespace  Eigen;


int N_data;
MatrixXd positionmat[10000];
int num_data = 0;

bool move_active_offlineData=false;
double saturate(double a, double min, double max){
    if(a<min){return min;ROS_INFO("subceeding!");}
    else if(a>max){return max;ROS_INFO("exceeding!");}
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
res.result=1;
        return true;
    }
    else{
        return false;
    }
}

int main(int argc, char **argv)
{



    //*******************This part of code is for initialization of joints of the robot for walking**********************************
double q_dataOffline[33];
for (int i = 0; i < 33; ++i) {
    q_dataOffline[i]=0;
}

    ros::init(argc, argv, "offlineDataicNode");
    ros::NodeHandle nh;
    std_msgs::Float32MultiArray trajectory_data;
   ros::ServiceServer offlineDataService = nh.advertiseService("offlineData", offlineData_start);
   ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Float32MultiArray>("trajectory_data_offlineData",100);

    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

    ROS_INFO("started!");

    while (ros::ok())
    {
if(move_active_offlineData){
        //  for robot test musbe uncommented



        q_dataOffline[0]=0.0;
        for (int i = 1; i <=12 ; ++i) {
            q_dataOffline[i]= positionmat[num_data](0,i-1)-positionmat[0](0,i-1);
        }
//        if(num%200==0){qDebug()<<q[1]<<q[2]<<q[3]<<q[4]<<q[5]<<q[6]<<q[7]<<q[8]<<q[9]<<q[10]<<q[11]<<q[12];}

    trajectory_data.data.clear();
    for (int i = 0; i <= 33; ++i) {
        trajectory_data.data.push_back(q_dataOffline[i]);

    }
    trajectory_data_pub.publish(trajectory_data);
++num_data;

    if(num_data>=N_data){move_active_offlineData=false;
        qDebug()<<"offlineData done!";}



}


ros::spinOnce();
loop_rate.sleep();
    }
    return 0;
}



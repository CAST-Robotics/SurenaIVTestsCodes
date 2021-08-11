#include "surenastateestimation.h"
#include "Eigen/Dense"
#include <qdebug.h>
#include <vector>
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include"physic.h"

using namespace  Eigen;

SurenaStateEstimation::SurenaStateEstimation()
{
    qDebug()<<"creating SurenaStateEstimation";
}

MatrixXd SurenaStateEstimation::MatrixRotationPelvis(double q0, double q1, double q2, double q3)
{
    double q0s, q1s, q2s, q3s;
    q0s = q0*q0;
    q1s = q1*q1;
    q2s = q2*q2;
    q3s = q3*q3;
    MatrixXd rotationMatrix(3,3);
    rotationMatrix << 1-2*(q2s+q3s), 2*(q1*q2-q0*q3), 2*(q0*q2-q1*q3),
            2*(q1*q2+q0*q3), 1-2*(q1s+q3s), 2*(q2*q3-q0*q1),
            2*(-q0*q2+q1*q3), 2*(q0*q1+q2*q3), 1-2*(q1s+q2s);
    return rotationMatrix;
}

MatrixXd SurenaStateEstimation::MatrixRotationPelvisRPY(double roll, double pitch, double yaw)
{
    //sai = yaw = z (-pi to pi)
    //teta = pitch = y (-pi/2 to pi/2)
    //phi = roll = x (-pi to pi)
    double sai,teta,phi;
    sai = yaw*M_PI/180;
    teta = pitch*M_PI/180;
    phi = roll*M_PI/180;
    MatrixXd rotationMatrix(3,3);
    rotationMatrix << cos(teta)*cos(sai),
            sin(phi)*sin(teta)*cos(sai)-cos(phi)*sin(sai),
            cos(phi)*sin(teta)*cos(sai)+sin(phi)*sin(sai),
            cos(teta)*sin(sai),
            sin(phi)*sin(teta)*sin(sai)+cos(phi)*cos(sai),
            cos(phi)*sin(teta)*sin(sai)-sin(phi)*cos(sai),
            -sin(teta),
            sin(phi)*cos(teta),
            cos(phi)*cos(teta);
    return rotationMatrix;
}

MatrixXd SurenaStateEstimation::PelvisLocalChange(MatrixXd rotationMatrix, MatrixXd localPosition)
{
    MatrixXd localHeightChange(3,1);
    localHeightChange = rotationMatrix*localPosition;
    return localHeightChange;
}

double SurenaStateEstimation::IIRFilter(QList<double> filterCoefs, QList<double> dataBuffer, double newData)
{
    double result;
    if(filterCoefs.size()!=dataBuffer.size()+2){
        qDebug()<<"Error in SurenaStateEstimation::IIRFilter: Size of Data is not correct";
        return 0;
    }
    if(filterCoefs.size() % 2){
        qDebug()<<"Error in SurenaStateEstimation::IIRFilter: Size of Filter is not correct";
        return 0;
    }
    NPole = (filterCoefs.size())/2;
    result = filterCoefs.at(0)*newData;
    for(int i=1;i<NPole;i++){
        result = result + filterCoefs.at(i)*dataBuffer.at(i-1);
        result = result - filterCoefs.at(i+NPole)*dataBuffer.at(i+NPole-2);
    }
    result = result/filterCoefs.at(NPole);
    return result;
}

double SurenaStateEstimation::IMUAngularVelocityFilter(double newData){
    double result;
    //filter coefs: b0, b1, b2, . . ., a0, a1, a2, . . .
    FilterCoefs.clear();
//    FilterCoefs.append(0.07295965727);//b0
//    FilterCoefs.append(0.07295965727);//b1
//    FilterCoefs.append(1);//a0
//    FilterCoefs.append(-0.85408068546);//a1
    FilterCoefs.append(0.00024135904);//b0
    FilterCoefs.append(0.00048271810);//b1
    FilterCoefs.append(0.00024135904);//b2
    FilterCoefs.append(1);//a0
    FilterCoefs.append(-1.95557824031);//a1
    FilterCoefs.append(0.956543676511);//a2
    if(BufferIMUAngularVelocityData.isEmpty()){
        qDebug()<<"Warning in SurenaStateEstimation::IMUAngularVelocityFilter: AVBuffer is empty (should be happened only once)";
        for(int i=1;i<FilterCoefs.size()-1;i++){
            BufferIMUAngularVelocityData.append(newData);
            qDebug()<<"Buffer is now updating";
        }
    }
    result = IIRFilter(FilterCoefs,BufferIMUAngularVelocityData,newData);
    BufferIMUAngularVelocity(newData,result);
    return result;
}

double SurenaStateEstimation::IMUAngleFilter(double newData)
{
    double result;
    //filter coefs: b0, b1, b2, . . ., a0, a1, a2, . . .
    FilterCoefs.clear();
    FilterCoefs.append(0.07295965727);//b0
    FilterCoefs.append(0.07295965727);//b1
    FilterCoefs.append(1);//a0
    FilterCoefs.append(-0.85408068546);//a1
//    FilterCoefs.append(0.00024135904);//b0
//    FilterCoefs.append(0.00048271810);//b1
//    FilterCoefs.append(0.00024135904);//b2
//    FilterCoefs.append(1);//a0
//    FilterCoefs.append(-1.95557824031);//a1
//    FilterCoefs.append(0.956543676511);//a2
    if(BufferIMUAngleData.isEmpty()){
        qDebug()<<"Warning in SurenaStateEstimation::IMUAngleFilter: ABuffer is empty (should be happened only once)";
        for(int i=1;i<FilterCoefs.size()-1;i++){
            BufferIMUAngleData.append(newData);
            qDebug()<<"Buffer is now updating";
        }
    }
    result = IIRFilter(FilterCoefs,BufferIMUAngleData,newData);
    BufferIMUAngle(newData,result);
    return result;
}

void SurenaStateEstimation::BufferIMUAngularVelocity(double newData, double newFilterData){
    QList<double> bufferData;
    bufferData = BufferIMUAngularVelocityData;
    BufferIMUAngularVelocityData.clear();
    BufferIMUAngularVelocityData.append(newData);
    for(int i=1;i<(bufferData.size())/2;i++){
        BufferIMUAngularVelocityData.append(bufferData.at(i-1));
    }
    BufferIMUAngularVelocityData.append(newFilterData);
    for(int i=((bufferData.size()/2)+1);i<bufferData.size();i++){
        BufferIMUAngularVelocityData.append(bufferData.at(i-1));
    }
}

void SurenaStateEstimation::BufferIMUAngle(double newData, double newFilterData){
    QList<double> bufferData;
    bufferData = BufferIMUAngleData;
    BufferIMUAngleData.clear();
    BufferIMUAngleData.append(newData);
    for(int i=1;i<(bufferData.size())/2;i++){
        BufferIMUAngleData.append(bufferData.at(i-1));
    }
    BufferIMUAngleData.append(newFilterData);
    for(int i=((bufferData.size()/2)+1);i<bufferData.size();i++){
        BufferIMUAngleData.append(bufferData.at(i-1));
    }
}

double SurenaStateEstimation::CP_IMU(double angularVelocityPitch, double anglePitch){
    double z_com = 0.981;
    double cp_IMU;
    double omega = GravityAcceleration/z_com;
    double angularVelocityPitchFiltered;
    angularVelocityPitchFiltered = IMUAngularVelocityFilter(angularVelocityPitch);
    cp_IMU = anglePitch + 180*angularVelocityPitchFiltered/(M_PI*(qSqrt(omega)));
    return cp_IMU;
}

double SurenaStateEstimation::AnglePitchPlus(double angularVelocityPitch, double anglePitch, double dt)
{
    double result;
    result = IMUAngularVelocityFilter(angularVelocityPitch*180/M_PI)*dt + IMUAngleFilter(anglePitch);
    return result;
}

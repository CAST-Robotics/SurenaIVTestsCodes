#ifndef SURENASTATEESTIMATION_H
#define SURENASTATEESTIMATION_H
#include "Eigen/Dense"
#include <qdebug.h>
#include <vector>
#include <qmath.h>
#include <cstring>
#include<qdebug.h>

using namespace  Eigen;

class SurenaStateEstimation
{
public:
    SurenaStateEstimation();
    MatrixXd MatrixRotationPelvis(double q0, double q1, double q2, double q3);
    MatrixXd PelvisLocalChange(MatrixXd RotationMatrix, MatrixXd localPosition);
    MatrixXd MatrixRotationPelvisRPY(double roll, double pitch, double yaw);
    double IIRFilter(QList<double> filterCoefs, QList<double> dataBuffer, double newData);
    double IMUAngularVelocityFilter(double newData);
    double IMUAngleFilter(double newData);
    void BufferIMUAngularVelocity(double newData, double newFilterData);
    void BufferIMUAngle(double newData, double newFilterData);
    double CP_IMU(double angularVelocityPitch, double anglePitch);
    double AnglePitchPlus(double angularVelocityPitch, double anglePitch, double dt);

    int NPole;
    QList<double> FilterCoefs;
    QList<double> BufferIMUAngularVelocityData;
    QList<double> BufferIMUAngleData;
};

#endif // SURENASTATEESTIMATION_H

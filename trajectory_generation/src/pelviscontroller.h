#ifndef PELVISCONTROLLER_H
#define PELVISCONTROLLER_H
#include "Eigen/Dense"
#include <qdebug.h>
#include <vector>
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include"surenastateestimation.h"
#include "qcgenerator.h"

#define relativeSaturationLimit 0.5 //degrees
#define maxSaturationLimit 1 //degrees
#define minSaturationLimit -1 //degrees

using namespace  Eigen;


class PelvisController
{
public:
    SurenaStateEstimation estimateCP;
    QCgenerator AnkleAngleCalc;
    PelvisController();
    double CPController(double imuAngularVelocity, double imuAnglePitch, double kController, double stepTime);
    void CPOffsetFinder(double imuAngularVelocity, double imuAnglePitch);
    double PIDController(double kp, double kd, double ki, double e, double ed, double ei);
    void SudoTorqueController(double imuAngularVelocity, double imuAnglePitch, double kController, double stepTime, int rightInc, int leftInc, int rightQcOffset, int leftQcOffset);
    void SudoTorqueController2(double imuAngularVelocity, double imuAnglePitch, double kController, double stepTime, int rightInc, int leftInc, int rightQcOffset, int leftQcOffset);
    double PreviousControlInput = 0;
    double PreviousAngleError = 0;
    double PrePreviousAngleError = 0;
    double IntegralError = 0;

    double PreviousAngleErrorRight = 0;
    double PreviousAngleErrorLeft = 0;
    double PrePreviousAngleErrorRight = 0;
    double PrePreviousAngleErrorLeft = 0;
    double IntegralErrorRight = 0;
    double IntegralErrorLeft = 0;

    double OffsetCP = 0;
    double OffsetPitch = 0;
    double RightSudoTorqueControllerOutput;
    double LeftSudoTorqueControllerOutput;
};

#endif // PELVISCONTROLLER_H

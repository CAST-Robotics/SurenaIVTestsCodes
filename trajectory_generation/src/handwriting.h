#ifndef HANDWRITING_H
#define HANDWRITING_H
#include <qdebug.h>
#include <qmath.h>
#include <iostream>
#include <cstdlib>
#include<math.h>
#include<qtimer.h>
#include <QElapsedTimer>
#include <QTime>
using namespace std;

class handWriting
{
public:
    double a;
    double b;
    double c;
    double d;
    double P_start;
    double P_end;
    double dt;
    double t;
    double T;
    double L;
    double V;
    double P;
    double V_x;
    double V_y;
    double V_z;
    QMap<QString,double>Z_start;
    QMap<QString,double>Y_start;
    QMap<QString,double>Z_end;
    QMap<QString,double>Y_end;
    handWriting();
    double saturate(double a, double min, double max);
    double move2pose(double max, double t_local, double T_start, double T_end);
    double move_to_diff(double max, double t_local, double T_start, double T_end);
    double Velocity(double _L, double _t, double _T);
    double Position(double _L, double _t, double _T);
    void move2next(QString current, QString next);
    void defineLetter(QString letter, double z_start, double y_start, double z_end, double y_end);

    void Write_S();
    void Write_U();
    void Write_R();
    void Write_E();
    void Write_N();
    void Write_A();

    void moveback();
};

#endif // HANDWRITING_H

#ifndef GRIPING_H
#define GRIPING_H
#include <QList>
#include "ankle_adaptation.h"
#include <qmath.h>
#include <cstring>
#include <math.h>
#include"Eigen/Dense"
#include "Robot.h"
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include "right_hand.h"
#include"MinimumJerkInterpolation.h"
class griping
{
public:
    int n_bottle=0;
    VectorXd r_bottle_cam;
    VectorXd r_bottle_shoulder;
    VectorXd r_bottle_with_distance;
    VectorXd r_bottle_for_grip;
    VectorXd r_bottle_shoulder_initial;
    double safe_distance=.15;
    double grip_distance=.07;
    double Theta_grip=-18*M_PI/180;

    double head_yaw=0;
    double head_pitch=0;
    double head_roll=0;
    bool grip_initialize=true;
    int n_grip_initialize=0;
    bool deliver_init=true;
    double Waist2ArmZ=0.2694;
    double Waist2RArmY=-0.235;
    double Waist2LArmY=0.235;
    double X_Object=320;
    double Y_Object=260;

    double safeX=.2;
    double offsetY=-.0;
    double offsetZ=.03;
    double offsetTh=-.05;
    right_hand hand0_r;
    VectorXd r_target_r;
    VectorXd r_deliver_r;
    VectorXd r_release_r;
    VectorXd r_middle_target1;
    VectorXd r_middle_target2;
    VectorXd r_middle_deliver;
    MatrixXd R_target_r;
    MatrixXd R_grip_r;
    MatrixXd R_deliver_r;
    VectorXd r_right_palm;
    MatrixXd R_right_palm;

    right_hand hand_funcs;
    right_hand hand_r;


    int fingers_mode_r;
    VectorXd q0_r;

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


    MatrixXd   t_r_init;
    MatrixXd P_x_r_init;
    MatrixXd V_x_r_init;
    MatrixXd A_x_r_init;
    MatrixXd P_y_r_init;
    MatrixXd V_y_r_init;
    MatrixXd A_y_r_init;
    MatrixXd P_z_r_init;
    MatrixXd V_z_r_init;
    MatrixXd A_z_r_init;
    MatrixXd   T_deliver;
    MatrixXd   t_deliver;
    MatrixXd P_x_deliver;
    MatrixXd P_y_deliver;
    MatrixXd P_z_deliver;
    MatrixXd V_x_deliver;
    MatrixXd V_y_deliver;
    MatrixXd V_z_deliver;
    MatrixXd A_x_deliver;
    MatrixXd A_y_deliver;
    MatrixXd A_z_deliver;
    MatrixXd  fingers_r;
    VectorXd q_ra;



    double q[34];
    int count = 0;
    double time=0.0;
    double time_head_init=0;
    double time_r;
    double time_deliver=0;
    double time_gripping=0;
    bool initializing=true;
    VectorXd qr_end;
    double WaistYaw=0;
    double WaistPitch=0;
    double v0_r=0;
    double v_target_r =.4;
    calc cal;

    griping();
    void object_detect(geometry_msgs::PoseArray msg);
    MatrixXd rightshoulder2waist(double WaistYaw, double WaistPitch);
    MatrixXd leftshoulder2waist(double WaistYaw, double WaistPitch);
    double norm(VectorXd V);
    void Updade(geometry_msgs::PoseArray msg, bool simulation);
    void Init();
    VectorXd r_camera2shoulder(VectorXd r_bottle_cam);
};

#endif // GRIPING_H

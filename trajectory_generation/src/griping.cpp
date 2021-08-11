#include "griping.h"

griping::griping()
{

    r_bottle_cam.resize(3);
    r_bottle_shoulder.resize(3);
    r_bottle_with_distance.resize(3);
    r_bottle_for_grip.resize(3);
    r_bottle_shoulder_initial.resize(3);

   r_target_r.resize(3);
   r_deliver_r.resize(3);
   r_release_r.resize(3);
   r_middle_target1.resize(3);
   r_middle_target2.resize(3);
   r_middle_deliver.resize(3);
   R_target_r.resize(3,3);
   R_grip_r.resize(3,3);
   R_deliver_r.resize(3,3);
   r_right_palm.resize(3);
   R_right_palm.resize(3,3);
    q0_r.resize(7);



      t_r_init.resize(1,3);
    P_x_r_init.resize(1,3);
    V_x_r_init.resize(1,3);
    A_x_r_init.resize(1,3);
    P_y_r_init.resize(1,3);
    V_y_r_init.resize(1,3);
    A_y_r_init.resize(1,3);
    P_z_r_init.resize(1,3);
    V_z_r_init.resize(1,3);
    A_z_r_init.resize(1,3);
      T_deliver.resize(1,6);
      t_deliver.resize(1,7);
    P_x_deliver.resize(1,7);
    P_y_deliver.resize(1,7);
    P_z_deliver.resize(1,7);
    V_x_deliver.resize(1,7);
    V_y_deliver.resize(1,7);
    V_z_deliver.resize(1,7);
    A_x_deliver.resize(1,7);
    A_y_deliver.resize(1,7);
    A_z_deliver.resize(1,7);
     fingers_r.resize(1,6);


    qr_end.resize(7);
    q0_r<<13*M_PI/180,
            -10*M_PI/180,
            0,
            -25*M_PI/180,
            0,
            0,
            0;
    q_ra=q0_r;

}


void griping::Init(){

    n_bottle=0;
    head_yaw=0;
    head_pitch=0;
    head_roll=0;
    count = 0;
    time=0.0;
    time_head_init=0;
    time_r;
    time_deliver=0;
    time_gripping=0;
    initializing=true;
        WaistYaw=0;
    WaistPitch=0;


    q0_r<<13*M_PI/180,
            -10*M_PI/180,
            0,
            -25*M_PI/180,
            0,
            0,
            0;
    q_ra=q0_r;

    v0_r=0;
    v_target_r =.4;

    right_hand temp(q0_r,r_target_r,R_target_r,0,0);
    hand0_r=temp;
    r_right_palm=hand0_r.r_right_palm;
    R_right_palm=hand0_r.R_right_palm;
    d0_r=hand0_r.dist;
    d_r=d0_r;
    d_des_r=hand0_r.d_des;
    theta_r=hand0_r.theta;
    theta_target_r=hand0_r.theta_target;
    sai_r=hand0_r.sai;
    sai_target_r=hand0_r.sai_target;
    phi_r=hand0_r.phi;
    phi_target_r=hand0_r.phi_target;
    hand0_r.HO_FK_right_palm(q0_r);

    r_bottle_shoulder<<.55,
            -0.0,
            -0.2;

    r_bottle_shoulder_initial<<.55,
            -0.0,
            -0.2;
    double xynorm=sqrt(pow(r_bottle_shoulder(0),2)+pow(r_bottle_shoulder(1),2));
    Vector3d xy_ofsset;
    xy_ofsset<<safe_distance*r_bottle_shoulder(0)/xynorm,safe_distance*r_bottle_shoulder(1)/xynorm,0;
    r_bottle_with_distance=r_bottle_shoulder-xy_ofsset;

    for (int i = 0; i < 34; ++i) {
        q[i]=0;
    }
q[32]=9;
q[33]=9;
}

void griping::Updade(geometry_msgs::PoseArray msg,bool simulation){

object_detect(msg);
    if (time_head_init<=1){
        head_pitch=cal.move2pose(cal.d2r(20),time_head_init,0,1);
        time_head_init+=.005;

    }

    else{
        if(initializing){


            //ROS_INFO("q0_r= %f, %f, %f, %f, %f, %f, %f",q0_r(0),q0_r(1),q0_r(2),q0_r(3),q0_r(4),q0_r(5),q0_r(6));
            r_middle_target1<<.2,
                    -.2,
                    -.33;

            r_middle_target2=r_bottle_with_distance;
            double xynrm=sqrt(pow(r_bottle_shoulder(0),2)+pow(r_bottle_shoulder(1),2));
            Vector3d xyofsset;
            xyofsset<<grip_distance*r_bottle_shoulder(0)/xynrm,grip_distance*r_bottle_shoulder(1)/xynrm,0;
            r_target_r=r_bottle_shoulder-xyofsset;

            r_middle_deliver<<.37,
                    -.07,
                    -.23;

            r_deliver_r<<.37,
                    -0.15,
                    -0.29;

            r_release_r<<.22,
                    -0.15,
                    -0.29;
            R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,Theta_grip+atan2(r_bottle_shoulder(1)-r_bottle_with_distance(1),r_bottle_shoulder(0)-r_bottle_with_distance(0)),3);
            R_grip_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,Theta_grip+atan2(r_bottle_shoulder(1)-r_bottle_with_distance(1),r_bottle_shoulder(0)-r_bottle_with_distance(0)),3);

            R_deliver_r=hand_funcs.rot(2,-100*M_PI/180,3)*hand_funcs.rot(1,atan2(r_deliver_r(1),r_deliver_r(0)),3)/**hand_funcs.rot(3,30*M_PI/180,3)*/;



            //****path generation




            MatrixXd T_right_shoulder_transpose=rightshoulder2waist(WaistYaw,WaistPitch);

            VectorXd temp(4);
            temp<<r_target_r,1;
            r_target_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);

            temp<<r_deliver_r,1;
            r_deliver_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);

            temp<<r_release_r,1;
            r_release_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);
            MatrixXd ord(1,2);
            ord<<5,5;

            t_r_init<<0,4,8;

            P_x_r_init<<hand0_r.r_right_palm(0),r_middle_target1(0),r_middle_target2(0);
            P_y_r_init<<hand0_r.r_right_palm(1),r_middle_target1(1),r_middle_target2(1);
            P_z_r_init<<hand0_r.r_right_palm(2),r_middle_target1(2),r_middle_target2(2);

            R_target_r=T_right_shoulder_transpose.block(0,0,3,3)*R_target_r;

            V_x_r_init<<0,INFINITY,0;
            V_y_r_init<<0,INFINITY,0;
            V_z_r_init<<0,INFINITY,0;
            A_x_r_init<<0,INFINITY,0;
            A_y_r_init<<0,INFINITY,0;
            A_z_r_init<<0,INFINITY,0;

            MatrixXd conx(3,3); conx<<P_x_r_init,V_x_r_init,A_x_r_init;
            MatrixXd cony(3,3); cony<<P_y_r_init,V_y_r_init,A_y_r_init;
            MatrixXd conz(3,3); conz<<P_z_r_init,V_z_r_init,A_z_r_init;

            //            X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
            //            Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
            //            Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

            X_coef_r=coef_generator.Coefficient1(t_r_init,ord,conx,.1).transpose();
            Y_coef_r=coef_generator.Coefficient1(t_r_init,ord,cony,.1).transpose();
            Z_coef_r=coef_generator.Coefficient1(t_r_init,ord,conz,.1).transpose();



            //ROS_INFO("theta_target=%f,sai_target=%f,phi_target=%f",theta_target,sai_target,phi_target);
            //ROS_INFO("\nr_target_r=\n%f\n%f\n%f",r_target_r(0),r_target_r(1),r_target_r(2));
            // ROS_INFO("\nR_target_r=\n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n",R_target_r(0,0),R_target_r(0,1),R_target_r(0,2),R_target_r(1,0),R_target_r(1,1),R_target_r(1,2),R_target_r(2,0),R_target_r(2,1),R_target_r(2,2));
            //            ROS_INFO("press any key to start!");
            //            getch();
            initializing=false;


        }


        else {


            time=double(count)*.005;
            time_r=time;

            VectorXd P_r(3); VectorXd V_r(3);

            if(time_r<=t_r_init(t_r_init.cols()-1)){

                for(int i=0;i<t_r_init.cols()-1;i++){

                    if(time_r>=t_r_init(i)&&time_r<t_r_init(i+1)){

                        P_r<<coef_generator.GetAccVelPos(X_coef_r.row(i),time_r,0,5)(0,0),
                                coef_generator.GetAccVelPos(Y_coef_r.row(i),time_r,0,5)(0,0),
                                coef_generator.GetAccVelPos(Z_coef_r.row(i),time_r,0,5)(0,0);
                        V_r<<coef_generator.GetAccVelPos(X_coef_r.row(i),time_r,0,5)(0,1),
                                coef_generator.GetAccVelPos(Y_coef_r.row(i),time_r,0,5)(0,1),
                                coef_generator.GetAccVelPos(Z_coef_r.row(i),time_r,0,5)(0,1);

                        hand_r.update_right_hand(q_ra,V_r,r_middle_target2,R_target_r);
                        r_right_palm=hand_r.r_right_palm;

                        R_right_palm=hand_r.R_right_palm;
                        hand_r.doQP(q_ra);
                        q_ra=hand_r.q_next;
                        d_r=hand_r.dist;
                        theta_r=hand_r.theta;
                        sai_r=hand_r.sai;
                        phi_r=hand_r.phi;
                        qr_end=q_ra;
                        fingers_mode_r=3;
                    }
                }


            }


//                else if (time_gripping<30&&(deliver_init&&(fabs(r_bottle_shoulder(0)-r_right_palm(0)-safeX)>.015||fabs(r_bottle_shoulder(1)+offsetY-r_right_palm(1))>.015||fabs(r_bottle_shoulder(2)-r_right_palm(2))>.015))){
//                    // else if (true){
//                    double V_factor=.8;//1.1;
//                    V_r<<V_factor*(r_bottle_shoulder(0)-r_right_palm(0)-safeX),
//                            V_factor*(r_bottle_shoulder(1)+offsetY-r_right_palm(1)),
//                            V_factor*(r_bottle_shoulder(2)+offsetZ-r_right_palm(2));

//                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,Theta_grip+atan2(r_bottle_shoulder(1)+offsetTh,r_bottle_shoulder(0)),3);
//                    R_grip_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,Theta_grip+atan2(r_bottle_shoulder(1),r_bottle_shoulder(0)),3)/**hand_funcs.rot(3,30*M_PI/180,3)*/;


            else if (time_gripping<30&&(deliver_init&&(norm(r_bottle_with_distance-r_right_palm)>.015))){
                // else if (true){
                double V_factor=1.1;//.8;
                V_r=V_factor*(r_bottle_with_distance-r_right_palm);
//matrix_view(V_r);
                double xynorm=sqrt(pow(r_bottle_shoulder(0),2)+pow(r_bottle_shoulder(1),2));
                Vector3d xy_ofsset;
                xy_ofsset<<grip_distance*r_bottle_shoulder(0)/xynorm,grip_distance*r_bottle_shoulder(1)/xynorm,0;
                r_target_r=r_bottle_shoulder-xy_ofsset;

                r_middle_deliver=(r_target_r+r_deliver_r)/2;
                r_middle_deliver(2)+=.1;

                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,Theta_grip+atan2(r_bottle_shoulder(1)-r_bottle_with_distance(1),r_bottle_shoulder(0)-r_bottle_with_distance(0)),3);
                R_grip_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,Theta_grip+atan2(r_bottle_shoulder(1)-r_bottle_with_distance(1),r_bottle_shoulder(0)-r_bottle_with_distance(0)),3);


                hand_r.update_right_hand(q_ra,V_r,r_bottle_with_distance,R_grip_r);
                r_right_palm=hand_r.r_right_palm;

                R_right_palm=hand_r.R_right_palm;
                hand_r.doQP(q_ra);
                q_ra=hand_r.q_next;
                d_r=hand_r.dist;
                theta_r=hand_r.theta;
                sai_r=hand_r.sai;
                phi_r=hand_r.phi;
                qr_end=q_ra;
                fingers_mode_r=3;
                deliver_init=true;
//                    r_target_r<<r_bottle_shoulder(0)-.07,
//                            r_bottle_shoulder(1)+offsetY,
//                            r_bottle_shoulder(2)+offsetZ;

                time_deliver=0;
                time_gripping+=.005;

            }

            else{

                if(deliver_init){
                    qDebug()<<"start girping";

                    MatrixXd ord(1,6);
                    ord.fill(5);
//                        t_deliver<<0,1.5,7.5,9,10.5,11.5,13;
                    T_deliver<<2,6,3,3,1,1.5;
                    t_deliver(0,0)=0;
                    for (int i = 1; i < t_deliver.cols(); ++i) {
                        t_deliver(0,i)=t_deliver(0,i-1)+T_deliver(0,i-1);
                    }


                    P_x_deliver<< hand_r.r_right_palm(0),r_target_r(0),r_target_r(0),r_middle_deliver(0),r_deliver_r(0),r_deliver_r(0),r_release_r(0);
                    P_y_deliver<< hand_r.r_right_palm(1),r_target_r(1),r_target_r(1),r_middle_deliver(1),r_deliver_r(1),r_deliver_r(1),r_release_r(1);
                    P_z_deliver<< hand_r.r_right_palm(2),r_target_r(2),r_target_r(2),r_middle_deliver(2),r_deliver_r(2),r_deliver_r(2),r_release_r(2);
                    V_x_deliver<<0,0,0,INFINITY,0,0,0;
                    V_y_deliver<<0,0,0,INFINITY,0,0,0;
                    V_z_deliver<<0,0,0,INFINITY,0,0,0;
                    A_x_deliver<<0,0,0,INFINITY,0,0,0;
                    A_y_deliver<<0,0,0,INFINITY,0,0,0;
                    A_z_deliver<<0,0,0,INFINITY,0,0,0;


                    MatrixXd conx(3,7); conx<<P_x_deliver,V_x_deliver,A_x_deliver;
                    MatrixXd cony(3,7); cony<<P_y_deliver,V_y_deliver,A_y_deliver;
                    MatrixXd conz(3,7); conz<<P_z_deliver,V_z_deliver,A_z_deliver;

                    X_coef_deliver=coef_generator.Coefficient1(t_deliver,ord,conx,.1).transpose();
                    Y_coef_deliver=coef_generator.Coefficient1(t_deliver,ord,cony,.1).transpose();
                    Z_coef_deliver=coef_generator.Coefficient1(t_deliver,ord,conz,.1).transpose();

                    fingers_r<<3,4,4,4,3,3;




                    deliver_init=false;
                }

                else{

                    for(int i=0;i<t_deliver.cols()-1;i++){
                        if(time_deliver>=t_deliver(i)&&time_deliver<t_deliver(i+1)){
                            P_r<<   coef_generator.GetAccVelPos(X_coef_deliver.row(i),time_deliver,0,5)(0,0),
                                    coef_generator.GetAccVelPos(Y_coef_deliver.row(i),time_deliver,0,5)(0,0),
                                    coef_generator.GetAccVelPos(Z_coef_deliver.row(i),time_deliver,0,5)(0,0);
                            V_r<<   coef_generator.GetAccVelPos(X_coef_deliver.row(i),time_deliver,0,5)(0,1),
                                    coef_generator.GetAccVelPos(Y_coef_deliver.row(i),time_deliver,0,5)(0,1),
                                    coef_generator.GetAccVelPos(Z_coef_deliver.row(i),time_deliver,0,5)(0,1);
                            if(i<2){
                                hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
                            }

                            else{
                                hand_r.update_right_hand(q_ra,V_r,r_deliver_r,R_deliver_r);
                            }
                            r_right_palm=hand_r.r_right_palm;
                            R_right_palm=hand_r.R_right_palm;
                            hand_r.doQP(q_ra);
                            q_ra=hand_r.q_next;
                            d_r=hand_r.dist;
                            theta_r=hand_r.theta;
                            sai_r=hand_r.sai;
                            phi_r=hand_r.phi;
                            qr_end=q_ra;
                            fingers_mode_r=fingers_r(i);
                        }

                    }
                    if(time_deliver>=t_deliver(t_deliver.cols()-1)){
                        for (int i = 0; i < 7; ++i) {
                            q_ra(i)=q0_r(i)+cal.move2zero(q_ra(i)-q0_r(i),time_deliver-t_deliver(t_deliver.cols()-1),4);
                        }
                        head_yaw=cal.move2zero(head_yaw,time_deliver-t_deliver(t_deliver.cols()-1),4);
                        head_pitch=cal.move2zero(head_pitch,time_deliver-t_deliver(t_deliver.cols()-1),4);

                        fingers_mode_r=9;

                    }
                    time_deliver+=.005;
                }

            }





        }
        ++count;
    }
//qDebug()<<n_bottle<<"\t"<<time_deliver;
if(n_bottle!=0 && time_deliver<max(1.0,t_deliver(t_deliver.cols()-1))){
    head_yaw+=.00001*(320-X_Object);
    head_pitch-=.00001*(260-Y_Object);
//   qDebug()<<"x,y "<<X_Object<<"\t"<<Y_Object;
// qDebug()<<"goh";
    head_yaw=cal.saturate(head_yaw,cal.d2r(-45),cal.d2r(45));
    head_pitch=cal.saturate(head_pitch,cal.d2r(-10),cal.d2r(25));
}
///////////////////if(time_deliver>=t_deliver(t_deliver.cols()-1)+4){break;}
    for (int i = 0; i < 31; ++i) {
        q[i]=0;
    }


    q[15]=q_ra(0)-q0_r(0);
    q[16]=q_ra(1)-q0_r(1);
    q[17]=q_ra(2)-q0_r(2);
    q[18]=q_ra(3)-q0_r(3);
    q[19]=q_ra(4)-q0_r(4);
    q[20]=q_ra(5)-q0_r(5);
    q[21]=q_ra(6)-q0_r(6);
    q[14]=0;
    q[13]=0;
    q[29]=head_yaw;
    q[30]=head_roll;
    q[31]=head_pitch;

    if(q[32]!=fingers_mode_r&&q[32]!=2){
        q[32]=2;

    }
    else{
        q[32]=fingers_mode_r;
    }



}



void griping::object_detect(geometry_msgs::PoseArray msg){
    double l=640;
    double w=480;
    double a=.51;
    double b=.385;
    double X0=.493;
    double Y0,Z0,L0,X,Y,Z;
    double x=l/2;
    double y=w/2;
    double z=0;

n_bottle=0;


    int n=msg.poses.size();
    for (int i = 0; i < n; ++i) {
        if (msg.poses[i].orientation.x==39) {
            n_bottle=1;
            x=msg.poses[i].position.x;
            y=msg.poses[i].position.y;
            z=msg.poses[i].position.z;
            X_Object=x;
            Y_Object=y;
            Y0=-(x-l/2)/l*a;
            Z0=-(y-w/2)/w*b;
            L0=sqrt(X0*X0+Y0*Y0+Z0*Z0);
            X=z/L0*X0;
            Y=X*Y0/X0;
            Z=X*Z0/X0;

            if(grip_initialize){
                n_grip_initialize++;


                if(X!=0){
                    if( n_grip_initialize==1){
                        r_bottle_cam<<X,Y,Z;}
                    else{
                        VectorXd temp(3);
                        temp<<X,Y,Z;
                        r_bottle_cam=(r_bottle_cam+temp)/2;
                    }
                }

                r_bottle_shoulder_initial=r_camera2shoulder(r_bottle_cam);

                if(n_grip_initialize>10){
                    grip_initialize=false;
                    qDebug()<<"Bottle initial position is:  "<<r_bottle_shoulder_initial(0)<<", "<<r_bottle_shoulder_initial(1)<<", "<<r_bottle_shoulder_initial(2);
                }
            }


            if(X!=0){ r_bottle_cam<<X,Y,Z;}
            r_bottle_shoulder=r_camera2shoulder(r_bottle_cam);

            double min=.417;
            double max=.58+.1;

            if(norm(r_bottle_shoulder)<min){
                r_bottle_shoulder=r_bottle_shoulder*min/norm(r_bottle_shoulder);
                qDebug()<<"bottle is too close!";
            }
            if(norm(r_bottle_shoulder)>max){
                r_bottle_shoulder=r_bottle_shoulder*max/norm(r_bottle_shoulder);
                qDebug()<<"bottle is too far!";
            }
        break;
        }



    }
    double xynorm=sqrt(pow(r_bottle_shoulder(0),2)+pow(r_bottle_shoulder(1),2));
    Vector3d xy_ofsset;
    xy_ofsset<<safe_distance*r_bottle_shoulder(0)/xynorm,safe_distance*r_bottle_shoulder(1)/xynorm,0;
    r_bottle_with_distance=r_bottle_shoulder-xy_ofsset;




}


MatrixXd griping::rightshoulder2waist(double WaistYaw, double WaistPitch){
    MatrixXd T(4,4);
    MatrixXd R(3,3);
    VectorXd P(3);
    R<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
            -sin(WaistPitch),              0,               cos(WaistPitch);

    P<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2RArmY*sin(WaistYaw),
            Waist2RArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2RArmY,
            Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;

    T<<R.transpose(),-R.transpose()*P,0,0,0,1;

    return T;
}

MatrixXd griping::leftshoulder2waist(double WaistYaw, double WaistPitch){
    MatrixXd T(4,4);
    MatrixXd R(3,3);
    VectorXd P(3);
    R<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
            -sin(WaistPitch),              0,               cos(WaistPitch);

    P<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2LArmY*sin(WaistYaw),
            Waist2LArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2LArmY,
            Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;
    T<<R.transpose(),-R.transpose()*P,0,0,0,1;
    return T;
}


VectorXd griping::r_camera2shoulder(VectorXd r_bottle_cam)
{
    double roll2pitch=.1012;
    double yaw2roll=.043;
    VectorXd r_neck(3);
    r_neck<<.00,0.235,0.04;
    VectorXd r_camera(3);
    r_camera<<0.15453-.036,0,.03161+.01768;


    MatrixXd T0(4,4);

    T0<<  cos(M_PI/12),0,sin(M_PI/12),0, 0,1,0,0, -sin(M_PI/12),0,cos(M_PI/12),0,  0,0,0,1;

    MatrixXd T1(4,4);
    T1<<1,0,0,r_camera(0),  0,1,0,r_camera(1),  0,0,1,r_camera(2),  0,0,0,1;
    MatrixXd T2(4,4);
    T2<< cos(head_pitch), 0,sin(head_pitch),0, 0,1,0,0, -sin(head_pitch),0,cos(head_pitch),0,  0,0,0,1;

    MatrixXd T3(4,4);
    T3<<  1,0,0,0,  0,cos(head_roll),-sin(head_roll),0,    0,sin(head_roll),cos(head_roll),roll2pitch,   0,0,0,1;
    MatrixXd T4(4,4);
    T4<<cos(head_yaw),-sin(head_yaw),0,0,  sin(head_yaw),cos(head_yaw),0,0,  0,0,1,yaw2roll,  0,0,0,1;
    MatrixXd T5(4,4);
    T5<<1,0,0,r_neck(0),  0,1,0,r_neck(1),  0,0,1,r_neck(2),  0,0,0,1;
    MatrixXd T(4,4);
    T=T5*T4*T3*T2*T1*T0;
    VectorXd temp(4); temp<<r_bottle_cam,1;
    temp=T*temp;

    return temp.block(0,0,3,1);

}


double griping::norm(VectorXd V){
    double s=0;
    for (int i = 0; i < V.rows(); ++i) {
        s+=V(i)*V(i);
    }
    return sqrt(s);
}

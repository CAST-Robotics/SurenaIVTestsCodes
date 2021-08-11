#include "ankle_adaptation.h"

Ankle_adaptation::Ankle_adaptation()
{
    double kp=.02;
    teta_PID_L.Init(.005,.9,-.9,kp,0,0);
    teta_PID_R.Init(.005,.9,-.9,kp,0,0);
    phi_PID_L.Init(.005,.9,-.9,kp,0,0);
    phi_PID_R.Init(.005,.9,-.9,kp,0,0);
}

void Ankle_adaptation::ankleOrientationAdaptationLeft(){
    //parameters of ankle adaptation
    if(a>threshold2 &&b>threshold2&&c>threshold2&&d>threshold2){
        //  theta_motor_L->0,phi_motor_L->0
        teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
        phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
    }
    else if(a<threshold &&b<threshold &&c<threshold &&d<threshold){
        //  theta_motor_L->0,phi_motor_L->0
        teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
        phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
    }

    else if (a>threshold ||b>threshold ||c>threshold ||d>threshold){//left
        //-----------------Pitch left ankle motor control---------------//
        if (abs(b-a)>=abs(c-d)) {
            if (abs(a-b)<120) {teta_motor_L=1*teta_motor_L+k1*(a-b);}
        }
        else {
            if (abs(d-c)<120) {teta_motor_L=1*teta_motor_L+k1*(d-c);}
        }

        //----------------Roll left ankle motor control---------------//
        if (abs(c-b)>=abs(d-a)) {
            if (abs(c-b)<120) {phi_motor_L=1*phi_motor_L+k2*(c-b);}
        }
        else {
            if (abs(a-d)<120) {phi_motor_L=1*phi_motor_L+k2*(d-a);}
        }
    }
    phi_motor_L=cal.saturate(phi_motor_L,-M_PI/90,M_PI/90);
    teta_motor_L=cal.saturate(teta_motor_L,-M_PI/90,M_PI/90);
}

void Ankle_adaptation::ankleOrientationAdaptationRight(){
    if(e>threshold2 &&f>threshold2&&g>threshold2&&h>threshold2){
        //  theta_motor_L->0,phi_motor_L->0
        teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
        phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
    }
    else if(e<threshold &&f<threshold &&g<threshold &&h<threshold){
        //  theta_motor_L->0,phi_motor_L->0
        teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
        phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
    }
    else if(e>threshold ||f>threshold ||g>threshold ||h>threshold){//right

        //-----------------Pitch left ankle motor control---------------//
        if (abs(f-e)>=abs(g-h)) {
            if (abs(e-f)<120) {teta_motor_R=1*teta_motor_R+k3*(e-f);}
        }
        else {
            if (abs(h-g)<120) {teta_motor_R=1*teta_motor_R+k3*(h-g);}
        }


        //----------------Roll left ankle motor control---------------//
        if (abs(g-f)>=abs(h-e)) {
            if (abs(g-f)<120) {phi_motor_R=1*phi_motor_R+k4*(g-f);}
        }
        else {
            if (abs(e-h)<120) {phi_motor_R=1*phi_motor_R+k4*(h-e);}
        }

    }


    phi_motor_R=cal.saturate(phi_motor_R,-M_PI/90,M_PI/90);
    teta_motor_R=cal.saturate(teta_motor_R,-M_PI/90,M_PI/90);

}


void Ankle_adaptation::Ankle_adaptation_update(double z_l,double z_r,int _a, int _b, int _c, int _d, int _e, int _f, int _g, int _h)
{
    timer+=dt;
    a=_a;
    b=_b;
    c=_c;
    d=_d;
    e=_e;
    f=_f;
    g=_g;
    h=_h;

    if(z_r<AnkleZR_input){rightGoingDown=true;}
    if(z_r>AnkleZR_input){rightGoingDown=false;}
    if(z_l<AnkleZL_input){leftGoingDown=true;}
    if(z_l>AnkleZL_input){leftGoingDown=false;}

    AnkleZR_input=z_r;
    AnkleZL_input=z_l;
    AnkleZL_output= AnkleZL_input;
    AnkleZR_output= AnkleZR_input;

    if(!rightGoingDown){
        rightzstop=false;
        AnkleZ_offsetR=0;
    }
    else{
        if(!rightzstop&&(e>bump_threshold_max||f>bump_threshold_max||g>bump_threshold_max||h>bump_threshold_max)){
            rightzstop=true;
            AnkleZ_offsetR=AnkleZR_input-lenghtOfAnkle;
            t_contact=timer;
            qDebug()<<"right foot contact: t_contact= "<<t_contact<<" AnkleZ_offsetR="<<AnkleZ_offsetR;
        }
        if(rightzstop){
            AnkleZ_offsetR=cal.move2zero(AnkleZ_offsetR,timer-t_contact,t_evac/*t_end-t_contact*/);
            AnkleZR_output=lenghtOfAnkle+AnkleZ_offsetR;
         //   qDebug()<<"AnkleZR_output="<<AnkleZR_output;
        }

    }

    if(!leftGoingDown){
        leftzstop=false;
        AnkleZ_offsetL=0;
    }
    else{
        if(!leftzstop&&(a>bump_threshold_max||b>bump_threshold_max||c>bump_threshold_max||d>bump_threshold_max)){
            leftzstop=true;
            AnkleZ_offsetL=AnkleZL_input-lenghtOfAnkle;
            t_contact=timer;
            qDebug()<<"left foot contact: t_contact= "<<t_contact<<" AnkleZ_offsetL="<<AnkleZ_offsetL;
        }
        if(leftzstop){
            AnkleZ_offsetL=cal.move2zero(AnkleZ_offsetL,timer-t_contact,t_evac/*t_end-t_contact*/);
            AnkleZL_output=lenghtOfAnkle+AnkleZ_offsetL;
            //qDebug()<<"AnkleZL_output="<<AnkleZL_output;
        }

    }
ankleOrientationAdaptationRight();
ankleOrientationAdaptationLeft();

    timer+=dt;
}

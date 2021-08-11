#include "imitation.h"

imitation::imitation()
{
    PoseRoot.resize(6,1);//position of pelvis respected to global coordinate
    PoseRFoot.resize(6,1);//position of right ankle joint respected to global coordinate
    PoseLFoot.resize(6,1);//position of left ankle joint respected to global coordinate
    R_P.resize(3,3);
    R_F_L.resize(3,3);
    R_F_R.resize(3,3);

    R_P2.resize(3,3);
    R_F_L2.resize(3,3);
    R_F_R2.resize(3,3);
    PoseRoot(0,0)=0;
    PoseRoot(1,0)=0;
    PoseRoot(2,0)=InitialPelvisHeight;
    PoseRoot(3,0)=0;//for side zhangooler add sth
    PoseRoot(4,0)=0;
    PoseRoot(5,0)=0;


    PoseLFoot(0,0)=0;
    PoseLFoot(1,0)=.115;
    PoseLFoot(2,0)=lenghtOfAnkle;
    PoseLFoot(3,0)=0;
    PoseLFoot(4,0)=0;
    PoseLFoot(5,0)=0;

    PoseRFoot(0,0)=0;
    PoseRFoot(1,0)=-.115;
    PoseRFoot(2,0)=lenghtOfAnkle;
    PoseRFoot(3,0)=0;
    PoseRFoot(4,0)=0;
    PoseRFoot(5,0)=0;

    for (int i = 0; i < 34; ++i) {
        q[i]=0;
    }
    
    q_rh.resize(7);
    q_lh.resize(7);
    q_ra.resize(7);
    q_la.resize(7);


    GlobalTime=0;
    q_rh<<13*M_PI/180,-10*M_PI/180,0,-25*M_PI/180,0,0,0;
    q_lh<<13*M_PI/180,10*M_PI/180,0,-25*M_PI/180,0,0,0;
    q_ra=q_rh;
    q_la=q_lh;
    
}

void imitation::imitationInit()
{
  GlobalTime=0;
  t_initial=0;
  t_foot=0;
  trhf=0;
  trhs=0;
  trhh=0;
  tlhf=0;
  tlhs=0;
  tlhh=0;
  t_idle=0;
  t_home=0;

  PoseRoot(0,0)=0;
  PoseRoot(1,0)=0;
  PoseRoot(2,0)=InitialPelvisHeight;
  PoseRoot(3,0)=0;//for side zhangooler add sth
  PoseRoot(4,0)=0;
  PoseRoot(5,0)=0;


  PoseLFoot(0,0)=0;
  PoseLFoot(1,0)=.115;
  PoseLFoot(2,0)=lenghtOfAnkle;
  PoseLFoot(3,0)=0;
  PoseLFoot(4,0)=0;
  PoseLFoot(5,0)=0;

  PoseRFoot(0,0)=0;
  PoseRFoot(1,0)=-.115;
  PoseRFoot(2,0)=lenghtOfAnkle;
  PoseRFoot(3,0)=0;
  PoseRFoot(4,0)=0;
  PoseRFoot(5,0)=0;
  q_ra=q_rh;
  q_la=q_lh;

  head_yaw=0;
  head_pitch=0;
  head_roll=0;
  WaistYaw=0;
  WaistPitch=0;

  for (int i = 0; i < 34; ++i) {
      q[i]=0;
  }
}

void imitation::imitationUpdate(int code,bool simulation, int a,int b,int c,int d,int e,int f,int g,int h){

    motion_detect(code);

    if(t_initial<=3){
        q_ra(0)=cal.move2zero(q_ra(0),t_initial,2);
        q_ra(3)=cal.move2zero(q_ra(3),t_initial,2);
        q_la(0)=cal.move2zero(q_la(0),t_initial,2);
        q_la(3)=cal.move2zero(q_la(3),t_initial,2);

        PoseRoot(2,0)=InitialPelvisHeight+cal.move2pose(ReferencePelvisHeight-InitialPelvisHeight,t_initial,0,2) ;
q[14]=cal.move2pose(cal.d2r(-2),GlobalTime,0,2);

        t_initial+=.005;
    }

 else if(t_idle<idleTimeout){


        switch (motion_ID) {
        case 1://right to side
            foot_move(true,true,false,3);
            break;
        case 2://right to front
            foot_move(false,true,false,3);
            break;
        case 3://left to side
            foot_move(true,false,false,3);
            break;
        case 4://left to front
            foot_move(false,false,false,3);
            break;

        case 5://right side home
            foot_move(true,true,true,3);
            break;
        case 6://right front home
            foot_move(false,true,true,3);
            break;
        case 7://left side home
            foot_move(true,false,true,3);
            break;
        case 8://left front home
            foot_move(false,false,true,3);
            break;
        case 9://right side to front
            foot_move2(false,true,2);
            break;
        case 10://right front to side
            foot_move2(true,true,2);
            break;
        case 11://left side to front
            foot_move2(false,false,2);
            break;
        case 12://left front to side
            foot_move2(true,false,2);
            break;


        default:
            break;
        }





//         q_ra(0)
//         q_ra(1)
//         q_la(0)
//         q_la(1)





//right hand
    if(rhID==0){//home
       q_ra(0) =cal.move2zero(q_ra(0),trhh,3);
       q_ra(1)=q_rh(1)+cal.move2zero( q_ra(1)-q_rh(1),trhh,3);
       q_ra(2)=cal.move2zero(q_ra(2),trhh,3);
       q_ra(4)=cal.move2zero(q_ra(4),trhh,3);
    }

    if(rhID==1){//side
        q_ra(0)=cal.move2zero(q_ra(0),trhs,3);
        q_ra(1)=-M_PI/2+cal.move2zero(q_ra(1)+M_PI/2,trhs,3);
        q_ra(2)=cal.move2zero(q_ra(2),trhs,3);
        q_ra(4)=cal.move2zero(q_ra(4),trhs,3);
    }

    if(rhID==2){//front
        q_ra(0)=-M_PI/2+cal.move2zero(q_ra(0)+M_PI/2,trhf,3);
        q_ra(1)=q_rh(1)+cal.move2zero( q_ra(1)-q_rh(1),trhf,3);
        q_ra(2)=M_PI/4+cal.move2zero(q_ra(2)-M_PI/4,trhf,3);
        q_ra(4)=M_PI/4+cal.move2zero(q_ra(4)-M_PI/4,trhf,3);
    }

//left hand

    if(lhID==0){//home
       q_la(0) =cal.move2zero(q_la(0),tlhh,3);
       q_la(1)=q_lh(1)+cal.move2zero( q_la(1)-q_lh(1),tlhh,3);
       q_la(2)=cal.move2zero(q_la(2),tlhh,3);
       q_la(4)=cal.move2zero(q_la(4),tlhh,3);
    }

    if(lhID==1){//side
        q_la(0)=cal.move2zero(q_la(0),tlhs,3);
        q_la(1)=M_PI/2+cal.move2zero(q_la(1)-M_PI/2,tlhs,3);
        q_la(2)=cal.move2zero(q_la(2),tlhs,3);
        q_la(4)=cal.move2zero(q_la(4),tlhs,3);
    }

    if(lhID==2){//flont
        q_la(0)=-M_PI/2+cal.move2zero(q_la(0)+M_PI/2,tlhf,3);
        q_la(1)=q_lh(1)+cal.move2zero( q_la(1)-q_lh(1),tlhf,3);
        q_la(2)=-M_PI/4+cal.move2zero(q_la(2)+M_PI/4,tlhf,3);
        q_la(4)=-M_PI/4+cal.move2zero(q_la(4)+M_PI/4,tlhf,3);
    }






    GlobalTime+=.005;
    trhf+=.005;
    trhs+=.005;
    trhh+=.005;
    tlhf+=.005;
    tlhs+=.005;
    tlhh+=.005;
    t_foot+=.005;

    if(rhID==0 &&lhID==0 && legID==0){
        t_idle+=.005;
    }
    else{
       t_idle=0;
    }

    if(n_people!=0){
        head_yaw+=.00001*(320-X_face);
        head_pitch-=.00001*(200-Y_face);
    }
    else{
       // head_yaw-=.005*head_yaw;
        //head_pitch-=.005*head_pitch;
    }

    head_yaw=cal.saturate(head_yaw,-45*M_PI/180,45*M_PI/180);
    head_pitch=cal.saturate(head_pitch,-15*M_PI/180,15*M_PI/180);


}

    else{
        //if(t_home>3){break;}
        q_ra(0)=q_rh(0)+cal.move2zero(q_ra(0)-q_rh(0),t_home,3);
        q_ra(3)=q_rh(3)+cal.move2zero(q_ra(3)-q_rh(3),t_home,3);
        q_la(0)=q_lh(0)+cal.move2zero(q_la(0)-q_lh(0),t_home,3);
        q_la(3)=q_lh(3)+cal.move2zero(q_la(3)-q_lh(3),t_home,3);
        head_yaw=cal.move2zero(head_yaw,t_home,3);
        head_pitch=cal.move2zero(head_pitch,t_home,3);
        q[14]=cal.move2zero(q[14],t_home,3);

        PoseRoot(2,0)=InitialPelvisHeight+cal.move2zero(PoseRoot(2,0)-InitialPelvisHeight,t_home,3) ;

        t_home+=.005;

    }


     ankladpt.Ankle_adaptation_update(PoseLFoot(2,0),PoseRFoot(2,0),a,b,c,d,e,f,g,h);
     PoseLFoot(2,0)=ankladpt.AnkleZL_output;
     PoseRFoot(2,0)=ankladpt.AnkleZR_output;
    if(mirror){left_right_switch_legs();}


    R_P<<1,0,0,
            0,cos(PoseRoot(3)*M_PI/180),-sin(PoseRoot(3)*M_PI/180),
            0,sin(PoseRoot(3)*M_PI/180),cos(PoseRoot(3)*M_PI/180);
    R_F_R<<1,0,0,
            0,cos(PoseRFoot(3)*M_PI/180),-sin(PoseRFoot(3)*M_PI/180),
            0,sin(PoseRFoot(3)*M_PI/180),cos(PoseRFoot(3)*M_PI/180);
    R_F_L<<1,0,0,
            0,cos(PoseLFoot(3)*M_PI/180),-sin(PoseLFoot(3)*M_PI/180),
            0,sin(PoseLFoot(3)*M_PI/180),cos(PoseLFoot(3)*M_PI/180);

   R_P2<<cos(PoseRoot(4)*M_PI/180),0,sin(PoseRoot(4)*M_PI/180),
            0,1,0,
            -sin(PoseRoot(4)*M_PI/180), 0,cos(PoseRoot(4)*M_PI/180);
    R_F_R2<<cos(PoseRFoot(4)*M_PI/180),0,sin(PoseRFoot(4)*M_PI/180),
            0,1,0,
            -sin(PoseRFoot(4)*M_PI/180),0,cos(PoseRFoot(4)*M_PI/180);
    R_F_L2<<cos(PoseLFoot(4)*M_PI/180),0,sin(PoseLFoot(4)*M_PI/180),
            0,1,0,
            -sin(PoseLFoot(4)*M_PI/180),0,cos(PoseLFoot(4)*M_PI/180);

    R_P=R_P*R_P2;
    R_F_R=R_F_R*R_F_R2;
    R_F_L=R_F_L*R_F_L2;


//qDebug()<<"xl";
//cal.numplot(PoseLFoot(0,0),0,Ymax);
//qDebug()<<"yl";
//cal.numplot(PoseLFoot(1,0),.115,.115+Ymax);

//qDebug()<<"xr";
//cal.numplot(PoseRFoot(0,0),0,Ymax);
//qDebug()<<"-yr";
//cal.numplot(-PoseRFoot(1,0),.115,.115+Ymax);

    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,R_F_L,"Body", PoseRoot,R_P);
    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,R_F_R,"Body", PoseRoot,R_P);





    //                q[0]=0.0;
    //                for (int i = 1; i <=12 ; ++i) {
    //                    q[i]= 0;//positionmat[num_data](0,i-1)-positionmat[0](0,i-1);
    //                }


    q[0]=0.0;
    q[1]=SURENA.Links[1].JointAngle;
    q[2]=SURENA.Links[2].JointAngle+(!simulation)*RollModified_offline_right*M_PI/180;
    q[3]=SURENA.Links[3].JointAngle;
    q[4]=SURENA.Links[4].JointAngle;
    q[5]=cal.saturate(SURENA.Links[5].JointAngle,-M_PI/.4,M_PI/4);//+ankle_adaptation_switch*(left_first*teta_motor_R+(!left_first)*teta_motor_L);
    q[6]=SURENA.Links[6].JointAngle;//+ankle_adaptation_switch*(left_first*phi_motor_R+(!left_first)*phi_motor_L);//roll
    q[7]=SURENA.Links[7].JointAngle;
    q[8]=SURENA.Links[8].JointAngle+(!simulation)*RollModified_offline_left*M_PI/180;
    q[9]=SURENA.Links[9].JointAngle;
    q[10]=SURENA.Links[10].JointAngle;
    q[11]=cal.saturate(SURENA.Links[11].JointAngle,-M_PI/.4,M_PI/4);//+ankle_adaptation_switch*(left_first*teta_motor_L+(!left_first)*teta_motor_R);
    q[12]=SURENA.Links[12].JointAngle;//+ankle_adaptation_switch*(left_first*phi_motor_L+(!left_first)*phi_motor_R);
    if(mirror){left_right_switch_legs();}


    q[15]=q_ra(0)-q_rh(0);
    q[16]=q_ra(1)-q_rh(1);
    q[17]=q_ra(2)-q_rh(2);
    q[18]=q_ra(3)-q_rh(3);
    q[19]=q_ra(4)-q_rh(4);
    q[20]=q_ra(5)-q_rh(5);
    q[21]=q_ra(6)-q_rh(6);
    q[22]=q_la(0)-q_lh(0);
    q[23]=q_la(1)-q_lh(1);
    q[24]=q_la(2)-q_lh(2);
    q[25]=q_la(3)-q_lh(3);
    q[26]=q_la(4)-q_lh(4);
    q[27]=q_la(5)-q_lh(5);
    q[28]=q_la(6)-q_lh(6);
    q[29]=head_yaw;
    q[30]=head_roll;
    q[31]=head_pitch;

if(mirror){left_right_switch_hands();}

}

void imitation::motion_detect(int code){
   if(t_initial>3){
     
       int temp;
       //legs
       temp=code%100;
       if(temp==0||temp==1||temp==2||temp==10||temp==20){
           if(legID!=temp&&leg_motion_is_done){
               if(legID==0&&temp==1){motion_ID=1;}
               if(legID==0&&temp==2){motion_ID=2;}
               if(legID==0&&temp==10){motion_ID=3;}
               if(legID==0&&temp==20){motion_ID=4;}

               if(legID==1&&temp==0){motion_ID=5;}
               if(legID==2&&temp==0){motion_ID=6;}
               if(legID==10&&temp==0){motion_ID=7;}
               if(legID==20&&temp==0){motion_ID=8;}

               if(legID==1&&temp==2){motion_ID=9;}
               if(legID==2&&temp==1){motion_ID=10;}
               if(legID==10&&temp==20){motion_ID=11;}
               if(legID==20&&temp==10){motion_ID=12;}

               legID=temp;
               t_foot=0;
           }

    }
       //hands
       temp=((code-code%100)%1000)/100;
       if(rhID!=temp && temp==0){trhh=0;}
       if(rhID!=temp && temp==1){trhs=0;}
       if(rhID!=temp && temp==2){trhf=0;}

       rhID=temp;
       temp=((code-code%1000)%10000)/1000;
       if(lhID!=temp && temp==0){tlhh=0;}
       if(lhID!=temp && temp==1){tlhs=0;}
       if(lhID!=temp && temp==2){tlhf=0;}
       lhID=temp;




//qDebug()<<"code: "<<code<<"\tlegID: "<<legID<<"\trhID: "<<rhID<<"\tlhID: "<<lhID;
   }
}

void imitation::left_right_switch_hands(){
    for (int i = 0; i < 5; ++i) {
        double temp=q[i+15];
        q[i+15]=q[i+22];
        q[i+22]=temp;
    }
    q[16]=-q[16];
    q[17]=-q[17];
    q[19]=-q[19];
    q[23]=-q[23];
    q[24]=-q[24];
    q[26]=-q[26];
}


void imitation::left_right_switch_legs(){

    for (int i = 0; i < 6; ++i) {
        double temp=PoseLFoot(i,0);
        PoseLFoot(i,0)=PoseRFoot(i,0);
        PoseRFoot(i,0)=temp;
    }

   PoseRoot(1,0)=-PoseRoot(1,0);
   PoseRoot(3,0)=-PoseRoot(3,0);
   PoseRoot(5,0)=-PoseRoot(5,0);
   PoseRFoot(1,0)=-PoseRFoot(1,0);
   PoseRFoot(3,0)=-PoseRFoot(3,0);
   PoseRFoot(5,0)=-PoseRFoot(5,0);
   PoseLFoot(1,0)=-PoseLFoot(1,0);
   PoseLFoot(3,0)=-PoseLFoot(3,0);
   PoseLFoot(5,0)=-PoseLFoot(5,0);


   RollModified_offline_left=RollModified_offline_left+RollModified_offline_right;
   RollModified_offline_right=RollModified_offline_left-RollModified_offline_right;
   RollModified_offline_left=RollModified_offline_left-RollModified_offline_right;
   RollModified_offline_left=-RollModified_offline_left;
   RollModified_offline_right=-RollModified_offline_right;

}


void imitation::foot_move(bool side,bool right,bool home,double T){
    leg_motion_is_done=t_foot>=T;
    if(!leg_motion_is_done){
        double t1_p,t2_p,t1_a,t2_a,t1rm,t2rm;
        if(!home){
            t1_p=0;
            t2_p=.6*T;
            t1_a=.6*T;
            t2_a=T;
            t1rm=.5*T;//t1_a;
            t2rm=.8*T;//t2_a;
        }
        else{
            t1_p=.4*T;
            t2_p=T;
            t1_a=0;
            t2_a=.4*T;
            t1rm=.48*T;
            t2rm=.8*T;
        }

if(!right){left_right_switch_legs();}

        PoseRoot(0,0)=0;
        PoseRoot(1,0)=(Ypmax*!home)+cal.move2zero(PoseRoot(1,0)-(Ypmax*!home),t_foot-t1_p,t2_p-t1_p);//cal.move2pose(Ypmax,t_foot,t0,t1);
        PoseRoot(2,0)=ReferencePelvisHeight;
        PoseRoot(3,0)=0;//for side zhangooler add sth
        PoseRoot(4,0)=0;
        PoseRoot(5,0)=0;


        PoseLFoot(0,0)=0;
        PoseLFoot(1,0)=.115;
        PoseLFoot(2,0)=lenghtOfAnkle;
        PoseLFoot(3,0)=0;
        PoseLFoot(4,0)=0;
        PoseLFoot(5,0)=0;



        if(side){
            PoseRFoot(0,0)=cal.move2zero(PoseRFoot(0,0),t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(1,0)=-.115-(Ymax*!home)+cal.move2zero(PoseRFoot(1,0)-(-.115-(Ymax*!home)),t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(3,0)=(-Alphamax*!home)+cal.move2zero(PoseRFoot(3,0)-(-Alphamax*!home),t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(4,0)=cal.move2zero(PoseRFoot(4,0),t_foot,1);
        }
        else{
            PoseRFoot(0,0)=(Ymax*!home)+cal.move2zero(PoseRFoot(0,0)-(Ymax*!home),t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(1,0)=-.115+cal.move2zero(PoseRFoot(1,0)+.115,t_foot-t1_a,t2_a-t1_a);
            PoseRFoot(3,0)=cal.move2zero(PoseRFoot(3,0),t_foot,1);
            PoseRFoot(4,0)=(-Alphamax*!home)+cal.move2zero(PoseRFoot(4,0)-(-Alphamax*!home),t_foot-t1_a,t2_a-t1_a);
        }

        PoseRFoot(2,0)=lenghtOfAnkle+(Zmax*!home)+cal.move2zero(PoseRFoot(2,0)-(lenghtOfAnkle+(Zmax*!home)),t_foot-t1_a,t2_a-t1_a);
        PoseRFoot(5,0)=0;

        //RollModified_offline_left=cal.move2pose(Rollmodmax,t_foot,t1rm,t2rm);

        RollModified_offline_left=(Rollmodmax*!home)+cal.move2zero(RollModified_offline_left-(Rollmodmax*!home),t_foot-t1rm,t2rm-t1rm);
//left_right_switch_legs();
////left_right_switch_legs();
///
//qDebug()<<"z_a= "<<PoseRFoot(2,0);
//qDebug()<<"z_p= "<<PoseRoot(2,0);
        if(!right){left_right_switch_legs();}

//        qDebug()<<"PoseRoot,PoseRFoot,PoseLFoot";
//        MatrixXd tempp;
//        tempp=PoseRoot.transpose();        cal.matrix_view(tempp);
//        tempp=PoseRFoot.transpose();       cal.matrix_view(tempp);
//        tempp=PoseLFoot.transpose();       cal.matrix_view(tempp);
    }
}

void imitation::foot_move2(bool front2side,bool right,double T){
leg_motion_is_done=t_foot>=T;
if(!leg_motion_is_done){
if(!right){left_right_switch_legs();}
if(front2side){
    PoseRFoot(0,0)=cal.move2zero(PoseRFoot(0,0),t_foot,T);
    PoseRFoot(1,0)=-.115-(Ymax)+cal.move2zero(PoseRFoot(1,0)-(-.115-(Ymax)),t_foot,T);
    PoseRFoot(3,0)=(-Alphamax)+cal.move2zero(PoseRFoot(3,0)-(-Alphamax),t_foot,T);
    PoseRFoot(4,0)=cal.move2zero(PoseRFoot(4,0),t_foot,1);
}
else{
    PoseRFoot(0,0)=(Ymax)+cal.move2zero(PoseRFoot(0,0)-(Ymax),t_foot,T);
    PoseRFoot(1,0)=-.115+cal.move2zero(PoseRFoot(1,0)+.115,t_foot,T);
    PoseRFoot(3,0)=cal.move2zero(PoseRFoot(3,0),t_foot,1);
    PoseRFoot(4,0)=(-Alphamax)+cal.move2zero(PoseRFoot(4,0)-(-Alphamax),t_foot,T);
}
 if(!right){left_right_switch_legs();}
}

}




#include "calc.h"

calc::calc()
{

}



double calc::saturate(double a, double min, double max){
    if(a<min){//ROS_INFO("subceeding!");
        qWarning()<<"subceeding! mag:"<<a<<"  min:"<<min;
        return min;}
    else if(a>max){//ROS_INFO("exceeding!");
        qWarning()<<"exceeding! mag:"<<a<<"  max:"<<max;
        return max;}
    else{return a;}
}

double calc::d2r(double d){
    return d*M_PI/180;
}
void calc::matrix_view(MatrixXd M){

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

void calc::matrix_view(VectorXd M){
    QString str;
    for (int i = 0; i <M.rows() ; ++i) {str+=QString::number(M(i));str+="   ";}
    qDebug()<<str;
    qDebug()<<"";
}
double calc::move_rest_back(double max,double t_local,double T_start ,double T_move,double T_rest,double T_back){
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double c3_r=(10*max)/pow(T_back,3);
    double c4_r=-(15*max)/pow(T_back,4);
    double c5_r=(6*max)/pow(T_back,5);
    double T_end=T_start+T_move+T_rest+T_back;
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_start+T_move){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else if (t_local<T_start+T_move+T_rest){theta=max;}
    else if (t_local<T_start+T_move+T_rest+T_back){theta=c3_r*pow(T_end-t_local,3)+c4_r*pow(T_end-t_local,4)+c5_r*pow(T_end-t_local,5);}
    return theta;
}

double calc::move2pose(double max,double t_local,double T_start ,double T_end){
    double T_move=T_end-T_start;
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_end){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else{theta=max;}
    return theta;
}

double calc::move2pose_diff(double max,double t_local,double T_start ,double T_end){
    double T_move=T_end-T_start;
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_end){theta=c3*pow(t_local-T_start,2)*3+c4*pow(t_local-T_start,3)*4+c5*pow(t_local-T_start,4)*5;}
    else{theta=0;}
    return theta;
}

double calc::move2zero(double theta,double t,double T_home){
    double c3=10/pow(T_home,3);
    double c4=-15/pow(T_home,4);
    double c5=6/pow(T_home,5);
    double theta0;
    if(t<=0){return theta;}
    if(t>=T_home){return 0;}
    double dt=0.005;
    if(fabs(1-c3*pow(t-dt,3)-c4*pow(t-dt,4)-c5*pow(t-dt,5))>1e-6){
        theta0=theta/(1-c3*pow(t-dt,3)-c4*pow(t-dt,4)-c5*pow(t-dt,5));
        // qDebug()<<theta0;
        return theta0*(1-c3*pow(t,3)-c4*pow(t,4)-c5*pow(t,5));
    }


}


MatrixXd calc::rot(int axis , double q ,int dim){
    if (dim==3){
        MatrixXd R(3,3);
    if (axis==1){
        R<<1,0,0,
                0,cos(q),-sin(q),
                0,sin(q),cos(q);
    }

    if (axis==2){
        R<<cos(q),0,sin(q),
                0,1,0 ,
                -sin(q),0,cos(q);
    }

    if (axis==3){
                R<<cos(q),-sin(q),0,
                sin(q),cos(q),0,
                0,0,1;
    }
    return R;
    }

    if(dim==4){
                    MatrixXd R(4,4);
        if (axis==1){
            R<<1,0,0,0,
                    0,cos(q),-sin(q),0,
                    0,sin(q),cos(q),0,
                    0,0,0,1;
        }

        if (axis==2){
            R<<cos(q),0,sin(q),0,
                    0,1,0,0,
                    -sin(q),0,cos(q),0,
                    0,0,0,1;
        }

        if (axis==3){
            R<<cos(q),-sin(q),0,0,
                    sin(q),cos(q),0,0,
                    0,0,1,0,
                    0,0,0,1;
        }
        return R;
    }

}

MatrixXd calc::trans(int axis, double d){
    MatrixXd H(4,4);
    H=MatrixXd::Identity(4,4);
    H(axis-1,3)=d;
    return H;
}

MatrixXd calc::trans(Vector3d d){
    MatrixXd H(4,4);
    H=MatrixXd::Identity(4,4);
    H.block(0,3,3,1)=d;
    return H;
}

double calc::distance(VectorXd V1,VectorXd V2){
    double d;
    d=sqrt(pow(V1(0)-V2(0),2)+pow(V1(1)-V2(1),2)+pow(V1(2)-V2(2),2));
    return d;
}


void calc::numplot(double num,double min,double max){
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


double calc::norm(VectorXd V){
    double s=0;
    for (int i = 0; i < V.rows(); ++i) {
        s+=V(i)*V(i);
    }
    return sqrt(s);
}

double calc::Velocity(double _L,double _t, double _T){

    double _V=30*_L/pow(_T,3)*pow(_t,2)-60*_L/pow(_T,4)*pow(_t,3)+30*_L/pow(_T,5)*pow(_t,4);
    return _V;
}

double calc::Position(double _L,double _t, double _T){
    double _P=10*_L/pow(_T,3)*pow(_t,3)-15*_L/pow(_T,4)*pow(_t,4)+6*_L/pow(_T,5)*pow(_t,5);
    return _P;
}

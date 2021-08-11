#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include "walk_test/command.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/Trajectory.h"
#include <ctime>

using namespace std;



class WalkTest{
  public:
    WalkTest(ros::NodeHandle *n){

        motorDataPub_ = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
        absSub_ = n->subscribe("/surena/abs_joint_state",100, &WalkTest::absReader, this);
        incSub_ = n->subscribe("/surena/inc_joint_state",100, &WalkTest::qcInitial, this);
        jointCommand_ = n->advertiseService("joint_command", &WalkTest::sendCommand, this);
        trajectoryGenerator_ = n->serviceClient<trajectory_planner::Trajectory>("/traj_gen");
        jointAngles_ = n->serviceClient<trajectory_planner::JntAngs>("/jnt_angs");
        absPrinter_ = n->advertiseService("print_absolute", &WalkTest::absPrinter, this);
        walkService_ = n->advertiseService("walk_service", &WalkTest::walk, this);

        qcInitialBool_ = true;
        //rate_ = new ros::Rate(0.5);
        int temp_ratio[12] = {100, 100, 50, 80, 100, 100, 50, 80, 120, 120, 120, 120};
        for (int i=0; i<12; i++){
            harmonicRatio_[i] = temp_ratio[i];
        }
    }

    void qcInitial(const sensor_msgs::JointState & msg){

        if (qcInitialBool_){

            for (int i = 0; i <= 31; ++i) {
                qcOffset_[i]=int(msg.position[i+1]);


            }
            qcInitialBool_=false;
            ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                     qcOffset_[0],qcOffset_[1],qcOffset_[2],qcOffset_[3],qcOffset_[4],
                    qcOffset_[5],qcOffset_[6],qcOffset_[7],qcOffset_[8],qcOffset_[9],
                    qcOffset_[10],qcOffset_[11],qcOffset_[12],qcOffset_[13],qcOffset_[14],qcOffset_[15],qcOffset_[20],qcOffset_[21],qcOffset_[22],qcOffset_[23]);
        
        }
    }

    bool sendCommand(walk_test::command::Request &req, walk_test::command::Response &res){
        for (int i = 0; i < 32; i++){
            motorCommand_.data.push_back(qcOffset_[i]);
            cout << motorCommand_.data[i] << "\t";
        }

        motorCommand_.data[req.motor_id] += req.angle/2/M_PI*2304*harmonicRatio_[req.motor_id];
        motorDataPub_.publish(motorCommand_);
        //motorDataPub_.publish(motorCommand_);
        cout << "----------------------"<<endl;
        
    }

    void absReader(const sensor_msgs::JointState &msg){
        for(int i=0; i<32; i++){
            absData_[i] = msg.position[i];
        }
    }

    bool absPrinter(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        for(int i=0; i<32; i++){
            cout<<" Sensor ID: "<< i;
            cout<<" value: "<< absData_[i]<<endl;
        }
    }

    bool walk(trajectory_planner::Trajectory::Request  &req,
            trajectory_planner::Trajectory::Response &res){
        ros::Rate rate_(200);
        trajectory_planner::Trajectory traj_srv;
        traj_srv.request.alpha = req.alpha;
        traj_srv.request.t_double_support = req.t_double_support;
        traj_srv.request.t_step = req.t_step;
        traj_srv.request.COM_height = req.COM_height;
        traj_srv.request.step_length = req.step_length;
        traj_srv.request.step_count = req.step_count;
        traj_srv.request.ankle_height = req.ankle_height;
        trajectoryGenerator_.call(traj_srv);

        if(traj_srv.response.result){

            for (int i = 0; i < 32; i++){
                motorCommand_.data.push_back(qcOffset_[i]);
            }
            int i = 0;
            while(i < 2000){
                
                trajectory_planner::JntAngs jnt_srv;
                jnt_srv.request.iter = i;
                jointAngles_.call(jnt_srv);
                int amani[] = {5, 4, 3, 2, 10, 11, 9, 8, 0, 1, 7, 6};
                //cout << i << ", ";
                for(int j=0; j < 12; j++){
                    int modification = +1;
                    if (j == 1 || j == 6 || j == 3 || j == 8 || j == 11)
                        modification = -1;
                    
                    motorCommand_.data[j] = jnt_srv.response.jnt_angs[amani[j]]/2/M_PI*2304*harmonicRatio_[j] * modification + qcOffset_[j];
                    //cout << jnt_srv.response.jnt_angs[j] << ", ";
                    
                    //cout << jnt_srv.response.jnt_angs[j] << endl;
                    /*if (i==0 && j == 0){
                        motorCommand_.data[j] += -0.15/2/M_PI*2304*harmonicRatio_[j] * modification;
                        time_t now = time(0);
                        char* dt = ctime(&now);
                        cout << "iteration = " << i << ", motor id = " << j << ", time = " << dt << endl;
                    }else if (i==1 && j == 1)
                    {
                        motorCommand_.data[j] += -0.15/2/M_PI*2304*harmonicRatio_[j] * modification;
                        time_t now = time(0);
                        char* dt = ctime(&now);
                        cout << "iteration = " << i << ", motor id = " << j << ", time = " << dt << endl;
                    }else if (i==2 && j == 2)
                    {
                        motorCommand_.data[j] += 0.15/2/M_PI*2304*harmonicRatio_[j] * modification;
                        time_t now = time(0);
                        char* dt = ctime(&now);
                        cout << "iteration = " << i << ", motor id = " << j << ", time = " << dt << endl;
                    }*/
                        
                }
                //cout << endl;    
                motorDataPub_.publish(motorCommand_);
                rate_.sleep();
                i++;
            }
            res.result = true;
            return true;
        }
        else{
            ROS_INFO("Joint angles were not sent\n");
            res.result = false;
            return false;
        }
    }

private:
    ros::Publisher motorDataPub_;
    ros::Subscriber incSub_;
    ros::Subscriber absSub_;
    ros::ServiceServer jointCommand_;
    ros::ServiceServer absPrinter_;
    ros::ServiceClient trajectoryGenerator_;
    ros::ServiceClient jointAngles_;
    ros::ServiceServer walkService_;
    bool qcInitialBool_;
    int qcOffset_[32];
    std_msgs::Int32MultiArray motorCommand_;
    int harmonicRatio_[12];
    float absData_[32];
};

int main(int argc, char **argv){
    ros::init(argc, argv, "eth_subscriber");
    ros::NodeHandle n;
    WalkTest wt(&n);

    /*motorDataPub_ = n.advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
    ros::Subscriber incSub_ = n.subscribe("/surena/inc_joint_state",100, qcInitial);
    
    trajectoryGenerator_ = n.serviceClient<trajectory_planner::Trajectory>("/traj_gen");
    jointAngles_ = n.serviceClient<trajectory_planner::JntAngs>("/jnt_angs");
    ros::ServiceServer walkService_ = n.advertiseService("walk_service", walk);
    
    //WalkTest dcm_walk(&n);*/
    ros::spin();
	return 0;
}

#include <iostream>
#include <ros/ros.h>

#include <thread>
#include <chrono>
#include <Eigen/Core>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <std_srvs/Empty.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fpvbot_discription/data.h>

//! Defining global variables.. 
geometry_msgs::PoseStamped position;
// trajectory_msgs::MultiDOFJointTrajectory movement;
tf2::Quaternion setQuat;
bool flagX=1, flagY=1;
int choice=1, posError=20, cnt=160, angleTurn=30, mult=1, turn=0;
double velGain=0.01, roll=0, pitch=0, yaw=0;        //-0.05

// double calcDepth(){

// }

enum finiteMachine{
    DETECT, PASS, ROTATE
};
finiteMachine currentState = DETECT;

//! Subscriber Call Back function
void subCallback(fpvbot_discription::data msg){

    switch (currentState){
        case DETECT:
            if (msg.y>400+posError || msg.y<400-posError){
                if (400>msg.y){
                    position.pose.position.z += velGain;
                } else {
                    position.pose.position.z -= velGain;
                }
                flagY=0;
            }
            if (msg.x>400+posError || msg.x<400-posError){ 
                if (400 > msg.x){
                    position.pose.position.x -= velGain*sin(yaw);
                    position.pose.position.y += velGain*cos(yaw);
                } else {
                    position.pose.position.x += velGain*sin(yaw);
                    position.pose.position.y -= velGain*cos(yaw);
                }
                flagX=0;
            }
            if (flagX && flagY){
                mult = 2;
            }
            position.pose.position.x += mult*velGain*cos(yaw);
            position.pose.position.y += mult*velGain*sin(yaw);
            if (msg.distance>180 && flagX && flagY){
                currentState = PASS;
            }
            mult = 1;
            flagX = 1;
            flagY = 1;
            break;
        
        case PASS:
            position.pose.position.x += 3*velGain*cos(yaw);
            position.pose.position.y += 3*velGain*sin(yaw);
            cnt--;
            if (cnt<0){
                cnt = 160;
                currentState = ROTATE;
            }
            break;

        case ROTATE:
            yaw -= 0.05;
            setQuat.setRPY(roll, pitch, yaw);
            position.pose.orientation.w = setQuat.getW();
            position.pose.orientation.x = setQuat.getX();
            position.pose.orientation.y = setQuat.getY();
            position.pose.orientation.z = setQuat.getZ();
            if (yaw <= (-1.50 + turn*1.5707)){
                currentState = DETECT;
                turn -= 1;
                yaw = turn*1.5707;
            }
            break;
    }

    return;
} 

int main(int argc, char** argv){
    ros::init(argc, argv, "cv_controller");
    ros::NodeHandle nh;
    
    ros::Subscriber dataSub = nh.subscribe<fpvbot_discription::data>("/iris/image_center_location", 30, subCallback);
    ros::Publisher velPub = nh.advertise<geometry_msgs::PoseStamped>("/iris/command/pose", 30);

    ros::Rate loopRate(30);
    

    while(ros::ok()){
        ros::spinOnce();

        velPub.publish(position);

        loopRate.sleep();
    }

    return 0;
}
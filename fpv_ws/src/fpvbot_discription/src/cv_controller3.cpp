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
tf2::Quaternion setQuat;
bool flagX=1, flagY=1;
int choice=1, posError=20, cnt=90, angleTurn=30;
double velGain=0.01, roll=0, pitch=0, yaw=-0.05, xArea, yArea, rectArea;

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
            if (msg.x>400+posError || msg.x<400-posError){
                if (400 > msg.x)
                    position.pose.position.y += 0.015;
                else
                    position.pose.position.y -= 0.015;
                flagX = 0;
            }
            if (msg.y>400+posError || msg.y<400-posError){
                if (400 > msg.y)
                    position.pose.position.z += 0.015;
                else
                    position.pose.position.z -= 0.015;
                flagY = 0;
            }
            position.pose.position.x += 0.01;
            if (flagX && flagY){
                position.pose.position.x += 0.02;
            }
            flagX = 1;
            flagY = 1;
            if (msg.distance>150){
                currentState = PASS;
            }
            break;
        
        case PASS:
            position.pose.position.x += 0.03;
            cnt--;
            if (cnt<=0){
                cnt = 90;
                currentState = ROTATE;
            }
            break;

        case ROTATE:
            setQuat.setRPY(roll, pitch, yaw);
            position.pose.orientation.w = setQuat.getW();
            position.pose.orientation.x = setQuat.getX();
            position.pose.orientation.y = setQuat.getY();
            position.pose.orientation.z = setQuat.getZ();
            yaw -= 0.05;

            position.pose.position.x += 0.02;

            rectArea = msg.xDist*msg.yDist;
            xArea = msg.xDist*msg.xDist;
            yArea = msg.yDist*msg.yDist;

            // if (msg.xDist>100 && msg.yDist>100){
            //     yaw = -0.1;
            //     currentState = DETECT;
            // }

            if (rectArea>=std::min(xArea,yArea) && rectArea<=std::max(xArea,yArea) && rectArea>2500){
                yaw = -0.05;
                currentState = DETECT;
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
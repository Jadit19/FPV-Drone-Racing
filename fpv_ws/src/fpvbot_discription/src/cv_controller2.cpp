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
int posError=20, angleError=10, count=90;
double roll=0, pitch=0, yaw=-0.03;
bool angleCheck=1, flag=1, flagx=1, flagy=1, flagCount=1;

//! Subscriber Call Back function..
//! Still in trial mode..
void subCallback(fpvbot_discription::data msg){
    
    if (msg.distance<150 && flag){
        if (msg.x>400+posError || msg.x<400-posError){ 
            if (400 > msg.x)
                position.pose.position.y += 0.015;
            else
                position.pose.position.y -= 0.015;
            flagx = 0;
        }

        if (msg.y>400+posError || msg.y<400-posError){
            if (400 > msg.y)
                position.pose.position.z += 0.015;
            else
                position.pose.position.z -= 0.015;
            flagy = 0;
        }

        position.pose.position.x += 0.01;
        if (flagx && flagy){
            position.pose.position.x += 0.02;
        }

        flagx = 1;
        flagy = 1;
    } else if (count>0 && flagCount){
        flag = 0;
        position.pose.position.x += 0.03;
        count--;
    } else {
        flagCount = 0;
        count = 90;

        if (msg.xDist<100 || msg.yDist<100){
            setQuat.setRPY(roll, pitch, yaw);
            position.pose.orientation.w = setQuat.getW();
            position.pose.orientation.x = setQuat.getX();
            position.pose.orientation.y = setQuat.getY();
            position.pose.orientation.z = setQuat.getZ();
            yaw -= 0.03;
        } else {
            flag = 1;
            yaw = -0.03;
        }

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
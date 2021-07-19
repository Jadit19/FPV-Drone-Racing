#include <thread>
#include <chrono>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <fpvbot_discription/data.h>
#include <geometry_msgs/PointStamped.h>

//! Defining global variables.. 
geometry_msgs::PoseStamped position;
bool flagx=0, flagy=0;
int error = 30;

//! Subscriber Callback function
void subCallback(fpvbot_discription::data msg){
    if (msg.distance < 200){
        if (msg.x>400+error || msg.x<400-error){ 
            flagx = 0;
            if (400 > msg.x)
                position.pose.position.y += 0.01;
            else
                position.pose.position.y -= 0.01;
        } else {
            flagx = 1;
        }

        if (msg.y>400+error || msg.y<400-error){
            flagy = 0;
            if (400 > msg.y)
                position.pose.position.z += 0.01;
            else
                position.pose.position.z -= 0.01;
        } else {
            flagy = 1;
        }

        if (flagy && flagx){
            position.pose.position.x += 0.1;
        }
    } else {
        position.pose.position.x += 0.05;
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
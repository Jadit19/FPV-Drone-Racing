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
bool flagx=1, flagy=1, flagz=1, flag=1;
int posError = 10; int count = 120;
double gain;
double gainx, gainy, gainz;

//! Subscriber Callback function
//! Safe and works..
// void subCallback(fpvbot_discription::data msg){
//     if (msg.distance < 200){
//         if (msg.x>400+posError || msg.x<400-posError){ 
//             flagx = 0;
//             if (400 > msg.x)
//                 position.pose.position.y += 0.01;
//             else
//                 position.pose.position.y -= 0.01;
//         } else {
//             flagx = 1;
//         }

//         if (msg.y>400+posError || msg.y<400-posError){
//             flagy = 0;
//             if (400 > msg.y)
//                 position.pose.position.z += 0.01;
//             else
//                 position.pose.position.z -= 0.01;
//         } else {
//             flagy = 1;
//         }

//         if (flagy && flagx){
//             position.pose.position.x += 0.02;
//         }
//     } else {
//         position.pose.position.x += 0.03;
//     }

//     return;
// }

//! Still in trial mode..
// void subCallback(fpvbot_discription::data msg){
//     if (msg.distance<120 && flag){
//         if (msg.x>400+posError || msg.x<400-posError){ 
//             if (400 > msg.x)
//                 position.pose.position.y += 0.01;
//             else
//                 position.pose.position.y -= 0.01;
//         }

//         if (msg.y>400+posError || msg.y<400-posError){
//             if (400 > msg.y)
//                 position.pose.position.z += 0.01;
//             else
//                 position.pose.position.z -= 0.01;
//         }

//         position.pose.position.x += 0.01;

//     } else if (count > 0){
//         flag = 0;
//         position.pose.position.x += 0.03;
//         count--;
//     } else {
//         count = 120;
//         flag = 1;
//     }

//     return;
// }

//! Still in trial mode..
void subCallback(fpvbot_discription::data msg){
    gain = (1 - msg.distance/150)*0.035;
    if (msg.distance<150 && flag){
        if (msg.x>400+posError || msg.x<400-posError){ 
            if (400 > msg.x)
                position.pose.position.y += 0.01;
            else
                position.pose.position.y -= 0.01;
            flagx=0;
        }

        if (msg.y>400+posError || msg.y<400-posError){
            if (400 > msg.y)
                position.pose.position.z += 0.01;
            else
                position.pose.position.z -= 0.01;
            flagy=0;
        }

        position.pose.position.x += gain;
        if (flagx && flagy){
            position.pose.position.x += 0.01;
        }
        flagx = 1;
        flagy = 1;

    } else if (count > 0){
        flag = 0;
        position.pose.position.x += 0.03;
        count--;
    } else {
        count = 120;
        flag = 1;
    }
    
    return;
}

//! Still in trial mode..
// posError = 5;
// void subCallback(fpvbot_discription::data msg){
//     gainy = (1.02 - msg.x/400) * 0.03;
//     gainz = (1.02 - msg.y/400) * 0.03;
//     gainx = (1.1 - msg.distance/150) * 0.03;

//     if (msg.distance<150 && flag){
//         if (msg.x>400+posError || msg.x<400-posError){ 
//             position.pose.position.y += gainy;
//             flagx=0;
//         }

//         if (msg.y>400+posError || msg.y<400-posError){
//             position.pose.position.z += gainz;
//             flagy=0;
//         }

//         position.pose.position.x += gainx;
//         if (flagx && flagy){
//             position.pose.position.x += 0.01;
//         }
//         flagx = 1;
//         flagy = 1;

//     } else if (count > 0){
//         flag = 0;
//         position.pose.position.x += 0.03;
//         count--;
//     } else {
//         count = 120;
//         flag = 1;
//     }

//     return;
// }

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
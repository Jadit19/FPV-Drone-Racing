#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fpvbot_discription/data.h>

static const std::string OPENCV_WINDOW = "Image window";
sensor_msgs::Image modImg;
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;
std::vector<cv::Point> pts(4);
cv::Scalar whiteColor = cv::Scalar(255,255,255);
cv::Mat drawing, red_filtered;
int maxHullIndex, secondMaxHullIndex;
float biggestContourArea, ctArea;
fpvbot_discription::data msg;

void drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points, cv::Scalar blueColor= cv::Scalar(255,255,255), int radius=3, int thickness=2) {

    //! Draw green circles on recognised points..
    for (int i=0; i<4; i++){
        cv::circle(image, points[i], 4, whiteColor, 2);
    }

    //! Draw center point..
    cv::Point2f center;
    for (int i=0; i<4; i++){
        center.x += points[i].x;
        center.y += points[i].y;
    } 
    center.x /= 4;
    center.y /= 4;
    cv::circle(image, center, 2, whiteColor, 2);

    msg.x = center.x;
    msg.y = center.y;
    msg.distance = sqrt(pow((pts[0].x - center.x), 2) + pow(pts[0].y - center.y, 2));

    return;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //! Selecting only the red..
    cv_ptr->image.convertTo(cv_ptr->image,-1,2,0 );
    cv::inRange(cv_ptr->image, cv::Scalar(0, 0, 100), cv::Scalar(75, 75, 255), red_filtered);
    cv::GaussianBlur(red_filtered, red_filtered, cv::Size(11, 11), 4);
    // red_filtered = Dilation(red_filtered, 0, 0);

    //! Finding contours for the image..
    cv::findContours(red_filtered, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    //! Find Convex Hull..
    std::vector<std::vector<cv::Point>> hull(contours.size());
    for (int i=0; i<contours.size(); i++){
        cv::convexHull(cv::Mat(contours[i]), hull[i], false);
    }

    //! Drawing the Convex Hull with maximum area..
    drawing = cv::Mat::zeros(red_filtered.size(), CV_8UC3);
    cv::cvtColor(drawing, drawing, CV_BGR2GRAY);
    int maxHullIndex = hull.size()-1;
    int secondMaxHullIndex = hull.size()-1;
    float biggestContourArea = 0, ctArea;

    // for (int i=0; i<contours.size(); i++){

    //     //! Finding the hull with maximum area 
    //     ctArea = cv::contourArea(hull[i]);
    //     if (ctArea > biggestContourArea){
    //         biggestContourArea = ctArea;
    //         secondMaxHullIndex = maxHullIndex;
    //         maxHullIndex = i;
    //     }

    //     // cv::drawContours(drawing, hull, i, whiteColor, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    // }
    cv::drawContours(drawing, hull, contours.size() -1, whiteColor, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    cv::GaussianBlur(drawing, drawing, cv::Size(5,5), 0);
    //! Getting edge points..
    cv::goodFeaturesToTrack(drawing, pts, 4, 0.01, 10);

    //! Draw on image..
    drawOnImage(drawing, pts);

    //! Outputting XD..
    cv_ptr->image = drawing;
    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2RGB);
    modImg = *cv_ptr->toImageMsg();

    //! Clearing the vectors.. 
    contours.clear();
    hierarchy.clear();
    pts.clear();

    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "image_converter1");
    ros::NodeHandle nh;

    ros::Subscriber image_sub_ = nh.subscribe("/iris/camera_red_iris/image_raw", 30, imageCb);
    ros::Publisher pubModImg = nh.advertise<sensor_msgs::Image>("/iris/camera_red_iris/image_output", 30);
    ros::Publisher centerPub = nh.advertise<fpvbot_discription::data>("/iris/image_center_location", 30);

    ros::Rate loopRate(30);
    msg.message.data = "Center Point Location on Image";

    while (ros::ok()){
        ros::spinOnce();

        pubModImg.publish(modImg);
        centerPub.publish(msg);

        loopRate.sleep();
    }
    
    return 0;
}
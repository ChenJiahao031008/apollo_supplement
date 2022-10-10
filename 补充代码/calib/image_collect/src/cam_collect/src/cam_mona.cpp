#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Twist.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <string>
#include "get_char_input.h"

using namespace std;
using namespace cv;

std::string rootPath;
std::string rgbPath;

void image_callback(const sensor_msgs::ImageConstPtr& msgImg)
{
    // ros->opencv 的常用套路
    cv_bridge::CvImagePtr cvImgPtr;
    try{
        cvImgPtr   = cv_bridge::toCvCopy(msgImg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e){
        ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
        return;
    }


    cv::Mat cvColorImgMat=cvImgPtr->image;

    //  /* 测试图像，使用时注释掉 */
    cv::Mat camera_up;
    cv::resize(cvColorImgMat,camera_up,cv::Size(640,480));

    string titile;
    cv::putText(camera_up,titile,cv::Point(20.40),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(100,0,0),2);



    cv::imshow("Color", camera_up);
    cv::waitKey(10);
    // double timeStamp = msgImg->header.stamp.toSec()/1000; //把时间戳转化成浮点型格式
    // std::cout << "timeStamp: " << msgImg->header.stamp << std::endl;

    char ch =kbhit();
    cout << ch;
    if (ch == 'q') return;
    if (ch == 32){
        static int i = 1;
        stringstream ss;
        ss << i++;
        std::string tmp;
        ss >> tmp;

        std::string fileName = tmp +".png";
        std::cout << "catch rgb image"<< std::endl;
        std::cout << rgbPath+fileName <<std::endl;
        cv::imwrite((rgbPath+fileName),   cvColorImgMat);
    }
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vedioControl");
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &image_callback);
    rootPath = argv[1];
    rgbPath = rootPath + "/camera_0/";

    ros::spin();

    return 0;
}

#include "ros/ros.h"
#include "image_transport/image_transport.h"
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

// 多线程相关
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> SyncPolicy;

using namespace std;
using namespace cv;

class ImageConverter
{
public:
    ros::NodeHandle nh_; // 节点名
    image_transport::ImageTransport it_; // 图像转换

    message_filters::Subscriber<sensor_msgs::Image>* image_sub_color; // 彩色图像接受
    message_filters::Subscriber<sensor_msgs::Image>* image_sub_depth; // 深度图像接受
    message_filters::Synchronizer<SyncPolicy>* sync_; // 信息同步器

    std::string rootPath;
    std::string depthPath;
    std::string rgbPath;


public:
  ImageConverter(char* address): it_(nh_),rootPath(address)
  {
    // 初始化
    image_sub_color = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/usb_cam/image_raw", 1);
    image_sub_depth = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/usb_cam1/image_raw", 1);
    sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *image_sub_color, *image_sub_depth);
    sync_->registerCallback(boost::bind(&ImageConverter::callback,this, _1, _2));
    depthPath = rootPath + "/camera_1/";
    rgbPath = rootPath + "/camera_0/";

    ROS_INFO("SUCCESS TO READ PARAM!");
  }

  ~ImageConverter()
  {}

  // 回调函数
  // 输入：RBG彩色图像和深度图像
  void callback(const sensor_msgs::ImageConstPtr& msgImg,const sensor_msgs::ImageConstPtr& msgDepth)
  {
    // ros->opencv 的常用套路
    cv_bridge::CvImagePtr cvImgPtr, cvDepthPtr;
    try{
        cvImgPtr   = cv_bridge::toCvCopy(msgImg,sensor_msgs::image_encodings::BGR8);
        cvDepthPtr = cv_bridge::toCvCopy(msgDepth,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e){
        ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
        return;
    }

    // 数据类型转换，得到彩色图、灰度图和深度图
    cv::Mat cvColorImgMat=cvImgPtr->image;
//    cv::Mat CurrentGray;
//    cv::cvtColor(cvColorImgMat,CurrentGray,CV_BGR2GRAY);
    cv::Mat cvDepthMat=cvDepthPtr->image;

    //  /* 测试图像，使用时注释掉 */
    cv::Mat camera_up,camera_down;
    cv::resize(cvColorImgMat,camera_up,cv::Size(640,480));
    cv::resize(cvDepthMat,camera_down,cv::Size(640,480));
    cv::imshow("Color", camera_up);
    cv::imshow("Depth", camera_down);
    cv::waitKey(10);
    // double timeStamp = msgImg->header.stamp.toSec()/1000; //把时间戳转化成浮点型格式
    std::cout << "timeStamp: " << msgImg->header.stamp << std::endl;

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
      std::cout << "catch rgb and depth image"<< std::endl;
      cv::imwrite((rgbPath+fileName),   cvColorImgMat);
      cv::imwrite((depthPath+fileName), cvDepthMat);
    }
  }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vedioControl");
    ImageConverter ic(argv[1]);
    ros::spin();
    return 0;
}

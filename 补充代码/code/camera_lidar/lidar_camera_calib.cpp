/******************************************************************************
 * Copyright 2022 The Shenlan Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "gflags/gflags.h"
#include "yaml-cpp/yaml.h"


DEFINE_string(pcd_file, "", "pcd file");
DEFINE_string(img_file, "", "image file");
DEFINE_string(cam_lidar_extrinsic_file, "",
              "provide camera extrinsic file path");
DEFINE_string(cam_intrinsic_file, "", "provide camera intrinsic file path");

double g_yaw = 0;
double g_pitch = 0;
double g_roll = 0;
double g_x = 0;
double g_y = 0;
double g_z = 0;
cv::Mat K, D;
Eigen::Affine3d cam_extrinsic;
cv::Mat image;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZI>());

bool LoadIntrinsic(const std::string &intrinsics_path, cv::Mat *dist_coeffs,
                   cv::Mat *intrisic_mat) {
  if (!(boost::filesystem::exists(intrinsics_path))) {
    return false;
  }
  YAML::Node config = YAML::LoadFile(intrinsics_path);
  if (config["K"] && config["D"]) {
    std::vector<double> K = config["K"].as<std::vector<double>>();
    std::vector<double> D = config["D"].as<std::vector<double>>();
    *intrisic_mat = cv::Mat(3, 3, cv::DataType<double>::type);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        intrisic_mat->at<double>(i, j) = K[i * 3 + j];
      }
    }
    *dist_coeffs = cv::Mat(5, 1, cv::DataType<double>::type);
    for (int i = 0; i < 5; i++) {
      dist_coeffs->at<double>(i) = D[i];
    }
  }

  return true;
}

void Project() {
  Eigen::Affine3d Tdelta = Eigen::Affine3d::Identity();
  Tdelta.translation() = Eigen::Vector3d(g_x, g_y, g_z);
  Tdelta.linear() =
      Eigen::AngleAxisd(g_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(g_pitch, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(g_roll, Eigen::Vector3d::UnitY()).toRotationMatrix();

  Eigen::Affine3d cam_extrinsic_final = cam_extrinsic * Tdelta;
  std::cout << "cam_extrinsic_final translation\n"
            << cam_extrinsic_final.translation() << std::endl;
  std::cout << "cam_extrinsic_final rotation\n"
            << cam_extrinsic_final.linear() << std::endl;
  Eigen::Quaterniond q(cam_extrinsic_final.linear());
  std::cout << "cam_extrinsic_final quaternion qx qy qz qw\n"
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
            << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cam(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud, *cloud_cam, cam_extrinsic_final.inverse());
  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;
  for (const auto &point : cloud_cam->points) {
    object_points.emplace_back(point.x, point.y, point.z);
  }

  cv::projectPoints(object_points, cv::Mat::zeros(3, 1, CV_32FC1),
                    cv::Mat::zeros(3, 1, CV_32FC1), K, D, image_points);

  cv::Mat test_image = image.clone();

  for (size_t i = 0; i < image_points.size(); ++i) {
    int col = static_cast<int>(std::round(image_points[i].x));
    int row = static_cast<int>(std::round(image_points[i].y));
    const auto &x = cloud_cam->points[i].x;
    const auto &y = cloud_cam->points[i].y;
    const auto &z = cloud_cam->points[i].z;
    auto dist = x * x + y * y + z * z;
    if (z <= 0 || col < 0 || col >= image.cols || row < 0 ||
        row >= image.rows || dist > 80 * 80) {
      continue;
    }

    cv::circle(test_image, cv::Point(col, row), 2, cv::Vec3b(0, 0, 255));

    cv::imshow("mainWin", test_image);
    cv::waitKey(100);
  }
}

void UpdateYaw(int value, void *) {
  g_yaw = (value - 18000) * 0.01 * M_PI / 360;
  Project();
}

void UpdatePitch(int value, void *) {
  g_pitch = (value - 18000) * 0.01 * M_PI / 360;
  Project();
}

void UpdateRoll(int value, void *) {
  g_roll = (value - 18000) * 0.01 * M_PI / 360;
  Project();
}

void UpdateX(int value, void *) {
  g_x = (value - 50) * 0.01;
  Project();
}

void UpdateY(int value, void *) {
  g_y = (value - 50) * 0.01;
  Project();
}

void UpdateZ(int value, void *) {
  g_z = (value - 50) * 0.01;
  Project();
}

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string pcd_file = FLAGS_pcd_file;
  std::string img_file = FLAGS_img_file;
  std::string cam_lidar_extrinsic_file = FLAGS_cam_lidar_extrinsic_file;
  std::string cam_intrinsic_file = FLAGS_cam_intrinsic_file;

  bool success = LoadExtrinsic(cam_lidar_extrinsic_file, &cam_extrinsic);
  if (!success) {
    AERROR << "Load camera extrinsic failed.";
    return 1;
  }

  success = LoadIntrinsic(cam_intrinsic_file, &D, &K);
  if (!success) {
    AERROR << "Load camera intrinsics failed.";
    return 1;
  }

  image = cv::imread(img_file);

  if (pcl::io::loadPCDFile(pcd_file, *cloud) == -1) {
    AWARN << "couldn't read file" << pcd_file;
    return -1;
  }

  int value_yaw = 18000;
  int value_pitch = 18000;
  int value_roll = 18000;
  int value_x = 50;
  int value_y = 50;
  int value_z = 50;
  cv::namedWindow("mainWin");
  cv::moveWindow("mainWin", 200, 200);
  cv::createTrackbar("yaw", "mainWin", &value_yaw, 36000, UpdateYaw);
  cv::createTrackbar("pitch", "mainWin", &value_pitch, 36000, UpdatePitch);
  cv::createTrackbar("roll", "mainWin", &value_roll, 36000, UpdateRoll);
  cv::createTrackbar("x", "mainWin", &value_x, 100, UpdateX);
  cv::createTrackbar("y", "mainWin", &value_y, 100, UpdateY);
  cv::createTrackbar("z", "mainWin", &value_z, 100, UpdateZ);

  Project();

  while (1) {
    cv::waitKey(100);
  }

  return 0;
}

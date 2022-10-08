/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once

#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <string>
#include <Eigen/Eigen>

void LoadExtrinsic(const std::string &filename, Eigen::Matrix4d &extrinsic) {
  YAML::Node config = YAML::LoadFile(filename);

  Eigen::Vector3d translation;
  Eigen::Quaterniond quat;

  if (config["transform"]) {
      if (config["transform"]["translation"]) {
          translation(0) =
              - config["transform"]["translation"]["x"].as<double>();
          translation(1) =
              - config["transform"]["translation"]["y"].as<double>();
          translation(2) =
              - config["transform"]["translation"]["z"].as<double>();
          if (config["transform"]["rotation"]) {
              double qx = config["transform"]["rotation"]["x"].as<double>();
              double qy = config["transform"]["rotation"]["y"].as<double>();
              double qz = config["transform"]["rotation"]["z"].as<double>();
              double qw = config["transform"]["rotation"]["w"].as<double>();
              quat = Eigen::Quaterniond(qw, -qx, -qy, -qz).toRotationMatrix();
          }
      }
  }

  extrinsic.block<3,3>(0,0) = quat.toRotationMatrix();
  extrinsic.block<3,1>(0,3) = quat * translation;
  
  return;
}
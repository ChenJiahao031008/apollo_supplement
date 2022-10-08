/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <string>

void LoadIntrinsic(const std::string &filename, Eigen::Matrix3d &K_mat,
                   std::vector<double> &dist) {
  YAML::Node config = YAML::LoadFile(filename);

  if (config["K"] && config["D"])
  {
      std::vector<double> K = config["K"].as<std::vector<double>>();
      std::vector<double> D = config["D"].as<std::vector<double>>();
      
      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              K_mat(i, j) = K[i * 3 + j];
          }
      }
      dist = D;
  }

  return;
}

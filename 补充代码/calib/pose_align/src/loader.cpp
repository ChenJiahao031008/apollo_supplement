#include "pose_align/loader.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <iostream>

#include "pose_align/sensors.h"
#include "pose_align/transform.h"


namespace pose_align {

    Loader::Loader() {}

    bool Loader::loadPoses(std::string &data_path) {
        std::string odom_path = data_path + "/odometry_loc.txt";
        std::ifstream fp(odom_path.c_str());

        bool first_frame = true;
        Transform init_pose_inv;

        Odom odom_temp;

        if (!fp.eof()) {
            std::string line;

            int index = 0;

            while (std::getline(fp, line)) {
                std::vector<std::string> split_str;
                boost::split(split_str, line, boost::is_any_of("  ,\t"));
                // add one judge condition

                Transform::Rotation quat(std::atof(split_str[8].c_str()),
                                            std::atof(split_str[5].c_str()),
                                            std::atof(split_str[6].c_str()),
                                            std::atof(split_str[7].c_str()));
                Transform::Translation trans(std::atof(split_str[2].c_str()),
                                                std::atof(split_str[3].c_str()),
                                                std::atof(split_str[4].c_str()));

                Transform pose = Transform(trans, quat);

                if (first_frame) {
                    init_pose_inv = pose.inverse();
                    first_frame = false;
                }
                pose = init_pose_inv * pose;

                double timestamp = std::atof(split_str[1].c_str());
                odom_temp.addTransformData(timestamp, pose);
            }
        }

        std::string lidar_slam_pose = data_path + "/lidar_slam_pose.txt";
        std::ifstream fin_slam(lidar_slam_pose.c_str());

        if (!fin_slam.eof()) {
            std::string line;

            size_t index = 0;

            while (std::getline(fin_slam, line)) {
                std::vector<std::string> split_str;
                boost::split(split_str, line, boost::is_any_of("  ,\t"));
                // add one judge condition

                Transform::Rotation quat(
                            std::atof(split_str[7].c_str()), std::atof(split_str[4].c_str()),
                            std::atof(split_str[5].c_str()), std::atof(split_str[6].c_str()));
                Transform::Translation trans(std::atof(split_str[1].c_str()),
                                std::atof(split_str[2].c_str()),
                                std::atof(split_str[3].c_str()));

                Transform pose = Transform(trans, quat);

                double timestamp = std::atof(split_str[0].c_str());
                

                Transform pose_odom = odom_temp.getOdomTransform(timestamp, index, &index);

                if(index <= 0 || index >= odom_temp.getOdomTransformSize() - 1){
                    std::cerr <<"the odom pose's timestamp interval is error"<<std::endl;
                }

                odom_source.addTransformData(timestamp, pose);
                odom_target.addTransformData(timestamp, pose_odom);
            }
        }


        std::cout << "odom_size is " << odom_target.getOdomTransformSize()
                  << std::endl;
    }

}  // namespace pose_align

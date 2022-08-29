#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "yaml-cpp/yaml.h"
#include <boost/filesystem.hpp>

std::vector<std::vector<double>> gnss_poses_;
Eigen::Affine3d velodyne_extrinsic_;
FILE *stamp_file_handle_;

struct PointXYZIT
{
    float x;
    float y;
    float z;
    unsigned char intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity,
                                                                          intensity)(double, timestamp,
                                                                                     timestamp))

bool LoadExtrinsic(const std::string &file_path, Eigen::Affine3d *extrinsic)
{
    YAML::Node config = YAML::LoadFile(file_path);
    if (config["transform"])
    {
        if (config["transform"]["translation"])
        {
            extrinsic->translation()(0) =
                config["transform"]["translation"]["x"].as<double>();
            extrinsic->translation()(1) =
                config["transform"]["translation"]["y"].as<double>();
            extrinsic->translation()(2) =
                config["transform"]["translation"]["z"].as<double>();
            if (config["transform"]["rotation"])
            {
                double qx = config["transform"]["rotation"]["x"].as<double>();
                double qy = config["transform"]["rotation"]["y"].as<double>();
                double qz = config["transform"]["rotation"]["z"].as<double>();
                double qw = config["transform"]["rotation"]["w"].as<double>();
                extrinsic->linear() =
                    Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
                return true;
            }
        }
    }
    return false;
}

inline bool EndsWith(std::string text, std::string suffix)
{
    return suffix.empty() ||
           (text.size() >= suffix.size() &&
            memcmp(text.data() + (text.size() - suffix.size()), suffix.data(),
                   suffix.size()) == 0);
}

bool GetFileList(const std::string &path, const std::string &suffix,
                 std::vector<std::string> *files)
{
    if (!boost::filesystem::exists(path))
    {
        std::cerr << path << " not exist." << std::endl;
        return false;
    }

    boost::filesystem::recursive_directory_iterator itr(path);
    while (itr != boost::filesystem::recursive_directory_iterator())
    {
        try
        {
            if (EndsWith(itr->path().string(), suffix))
            {
                files->push_back(itr->path().string());
            }
            ++itr;
        }
        catch (const std::exception &ex)
        {
            std::cerr << "Caught execption: " << ex.what();
            continue;
        }
    }
    return true;
}

inline void GetTimestampInterval(const pcl::PointCloud<PointXYZIT>::Ptr &msg,
                                 double *timestamp_min,
                                 double *timestamp_max)
{
    *timestamp_max = 0;
    *timestamp_min = std::numeric_limits<double>::max();

    for (const auto &point : msg->points)
    {
        double timestamp = point.timestamp;
        if (timestamp < *timestamp_min)
        {
            *timestamp_min = timestamp;
        }

        if (timestamp > *timestamp_max)
        {
            *timestamp_max = timestamp;
        }
    }
    // std::cout << "*timestamp_max" << *timestamp_max << std::endl;
    // std::cout << "*timestamp_min" << *timestamp_min << std::endl;
}

void ComputeInterpolationPose(
    const size_t index, const double ref_timestamp,
    const std::vector<std::vector<double>> &poses, Eigen::Affine3d *pose)
{
    double cur_timestamp = poses[index][1];
    double pre_timestamp = poses[index - 1][1];
    double t = (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);
    Eigen::Quaterniond pre_quatd(poses[index - 1][8], poses[index - 1][5],
                                 poses[index - 1][6], poses[index - 1][7]);
    Eigen::Translation3d pre_transd(poses[index - 1][2], poses[index - 1][3],
                                    poses[index - 1][4]);
    Eigen::Quaterniond cur_quatd(poses[index][8], poses[index][5],
                                 poses[index][6], poses[index][7]);
    Eigen::Translation3d cur_transd(poses[index][2], poses[index][3],
                                    poses[index][4]);

    Eigen::Quaterniond res_quatd = pre_quatd.slerp(1 - t, cur_quatd);

    Eigen::Translation3d re_transd;
    re_transd.x() = pre_transd.x() * t + cur_transd.x() * (1 - t);
    re_transd.y() = pre_transd.y() * t + cur_transd.y() * (1 - t);
    re_transd.z() = pre_transd.z() * t + cur_transd.z() * (1 - t);

    *pose = re_transd * res_quatd * velodyne_extrinsic_;
}

void GetTwoInterpolationPose(const double &timestamp_min,
                             const double &timestamp_max,
                             Eigen::Affine3d *pose_min_time,
                             Eigen::Affine3d *pose_max_time)
{
    double ref_timestamp = timestamp_min;
    auto time_search_cmp = [](const std::vector<double> &poses,
                              const double t) -> bool
    { return poses[1] < t; };
    size_t gnss_index = std::lower_bound(gnss_poses_.begin(), gnss_poses_.end(),
                                         ref_timestamp, time_search_cmp) -
                                         gnss_poses_.begin();
    if (gnss_index == 0)
        ++gnss_index;
    if (gnss_index > gnss_poses_.size() - 1)
        gnss_index = gnss_poses_.size() - 1;

    ComputeInterpolationPose(gnss_index, ref_timestamp, gnss_poses_,
                             pose_min_time);
    ref_timestamp = timestamp_max;
    gnss_index = std::lower_bound(gnss_poses_.begin(), gnss_poses_.end(),
                                  ref_timestamp, time_search_cmp) -
                                  gnss_poses_.begin();
    if (gnss_index == 0)
        ++gnss_index;
    if (gnss_index > gnss_poses_.size() - 1)
        gnss_index = gnss_poses_.size() - 1;
    ComputeInterpolationPose(gnss_index, ref_timestamp, gnss_poses_,
                             pose_max_time);
}

// extract code from modules/drivers/lidar_common/compensator/compensator.cc
void MotionCompensation(const pcl::PointCloud<PointXYZIT>::Ptr &msg,
                        const double timestamp_min,
                        const double timestamp_max,
                        const Eigen::Affine3d &pose_min_time,
                        const Eigen::Affine3d &pose_max_time,
                        pcl::PointCloud<PointXYZIT> *cloud)
{
    using std::abs;
    using std::acos;
    using std::sin;

    Eigen::Vector3d translation =
        pose_min_time.translation() - pose_max_time.translation();
    Eigen::Quaterniond q_max(pose_max_time.linear());
    Eigen::Quaterniond q_min(pose_min_time.linear());
    // std::cout << q_max.coeffs() << std::endl;
    // std::cout << q_min.coeffs() << std::endl;
    // std::cout << "========================" << std::endl;

    Eigen::Quaterniond q1(q_max.conjugate() * q_min);
    Eigen::Quaterniond q0(Eigen::Quaterniond::Identity());
    q1.normalize();
    translation = q_max.conjugate() * translation;

    double d = q0.dot(q1);
    double abs_d = abs(d);
    double f = 1.0 / (timestamp_max - timestamp_min + std::numeric_limits<double>::min());

    if (abs_d < 1.0 - 1.0e-8)
    {
        double theta = acos(abs_d);
        double sin_theta = sin(theta);
        double c1_sign = (d > 0) ? 1 : -1;
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            const auto &point = msg->points[i];
            float x_scalar = point.x;
            if (std::isnan(x_scalar))
            {
                cloud->points[i].x = point.x;
                cloud->points[i].y = point.y;
                cloud->points[i].z = point.z;
                cloud->points[i].intensity = point.intensity;
                continue;
            }
            float y_scalar = point.y;
            float z_scalar = point.z;
            Eigen::Vector3d p(x_scalar, y_scalar, z_scalar);

            double tp = point.timestamp;
            double t = static_cast<double>(timestamp_max - tp) * f;

            Eigen::Translation3d ti(t * translation);

            double c0 = sin((1 - t) * theta) / sin_theta;
            double c1 = sin(t * theta) / sin_theta * c1_sign;
            Eigen::Quaterniond qi(c0 * q0.coeffs() + c1 * q1.coeffs());

            Eigen::Affine3d trans = ti * qi;
            p = trans * p;

            cloud->points[i].x = static_cast<float>(p.x());
            cloud->points[i].y = static_cast<float>(p.y());
            cloud->points[i].z = static_cast<float>(p.z());
            cloud->points[i].intensity = point.intensity;
        }
        return;
    }
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const auto &point = msg->points[i];
        float x_scalar = point.x;
        if (std::isnan(x_scalar))
        {
            cloud->points[i].x = point.x;
            cloud->points[i].y = point.y;
            cloud->points[i].z = point.z;
            cloud->points[i].intensity = point.intensity;
            continue;
        }
        float y_scalar = point.y;
        float z_scalar = point.z;
        Eigen::Vector3d p(x_scalar, y_scalar, z_scalar);

        double tp = point.timestamp;
        double t = static_cast<double>(timestamp_max - tp) * f;
        Eigen::Translation3d ti(t * translation);

        p = ti * p;

        cloud->points[i].x = static_cast<float>(p.x());
        cloud->points[i].y = static_cast<float>(p.y());
        cloud->points[i].z = static_cast<float>(p.z());
        cloud->points[i].intensity = point.intensity;
    }
}

bool WritePcdFile(const unsigned int idx,
                  const std::string &filename,
                  const pcl::PointCloud<PointXYZIT>::Ptr &msg)
{
    pcl::PointCloud<PointXYZIT> cloud;
    cloud.width = msg->width;
    cloud.height = msg->height;
    cloud.is_dense = false;
    cloud.points.resize(msg->points.size());

    if (cloud.width == 1 || cloud.height == 1 || cloud.width == 0 || cloud.height == 0)
    {
        cloud.width = 1;
        cloud.height = msg->points.size();
        cloud.points.resize(msg->points.size());
    }

    Eigen::Affine3d pose_min_time;
    Eigen::Affine3d pose_max_time;

    double timestamp_min = 0;
    double timestamp_max = 0;
    GetTimestampInterval(msg, &timestamp_min, &timestamp_max);
    fprintf(stamp_file_handle_, "%u %lf\n", idx, timestamp_max);

    GetTwoInterpolationPose(timestamp_min, timestamp_max, &pose_min_time,
                            &pose_max_time);
    std::cout << "timestamp_min: " << std::setprecision(16) << timestamp_min << std::endl;
    std::cout << "timestamp_max: " << std::setprecision(16) << timestamp_max << std::endl;
    std::cout << Eigen::Quaterniond(pose_min_time.linear()).coeffs() << std::endl;
    std::cout << Eigen::Quaterniond(pose_max_time.linear()).coeffs() << std::endl;
    std::cout << "------------------------------" << std::endl;

    MotionCompensation(msg, timestamp_min, timestamp_max, pose_min_time,
                       pose_max_time, &cloud);


    // for (size_t i = 0; i < cloud.points.size(); ++i)
    // {
    //     const auto point = msg->points[i];

    //     cloud.points[i].x = static_cast<float>(point.x);
    //     cloud.points[i].y = static_cast<float>(point.y);
    //     cloud.points[i].z = static_cast<float>(point.z);
    //     cloud.points[i].intensity = point.intensity;
    // }

    pcl::io::savePCDFileBinaryCompressed(filename, cloud);
    return true;
}

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        std::cerr << "Usage: ./compensate gnss_loc_file extrinsic_file pcd_folder pcd_save_folder pcd_timestamp_file" << std::endl;
        std::cerr << "For example: ./compensate gnss_loc.txt velodyne64_novatel_extrinsics_example.yaml /rslidar_points /pcd pcd_timestamp.txt" << std::endl;
        return 1;
    }

    std::string gnss_loc_file = argv[1];
    std::string extrinsic_file = argv[2];
    std::string pcd_folder = argv[3];
    std::string pcd_save_folder = argv[4];
    std::string pcd_timestamp_file = argv[5];

    std::ifstream fodom(gnss_loc_file);
    if (fodom.is_open())
    {
        std::string line;
        while (getline(fodom, line))
        {
            std::stringstream ss(line);
            std::vector<double> data(12, -1);
            for (auto &d : data)
                ss >> d;
            if (data[0] == -1)
                break;
            gnss_poses_.emplace_back(data);
            for ( auto i: gnss_poses_.back()){
                std::cout << i << "\t ";
            }
            std::cout << std::endl;
            
        }
    }
    else
    {
        std::cerr << "Failed to open gnss pose file: " << gnss_loc_file
                  << std::endl;
        exit(0);
    }
    fodom.close();
    std::cout << "Read gnss poses done, gnss num: " << gnss_poses_.size() << std::endl;

    bool success = LoadExtrinsic(extrinsic_file, &velodyne_extrinsic_);
    if (!success)
    {
        std::cerr << "Load lidar extrinsic failed." << std::endl;
        return 1;
    }

    std::vector<std::string> pcd_file_names;
    if (!GetFileList(pcd_folder, ".pcd", &pcd_file_names))
    {
        std::cerr << "pcd_folder: " << pcd_folder << " get file list error." << std::endl;
        return false;
    }
    std::sort(pcd_file_names.begin(), pcd_file_names.end(),
              [](const std::string &lhs, const std::string &rhs)
              {
                  if (lhs.length() != rhs.length())
                  {
                      return lhs.length() < rhs.length();
                  }
                  return lhs <= rhs;
              });

    if (!boost::filesystem::exists(pcd_save_folder))
    {
        boost::filesystem::create_directory(pcd_save_folder);
    }

    if ((stamp_file_handle_ = fopen(pcd_timestamp_file.c_str(), "a")) == nullptr)
    {
        std::cerr << "Cannot open stamp file!" << std::endl;
    }

    for (const auto &pcd_file : pcd_file_names)
    {
        std::cout << "pcd_file " << pcd_file << std::endl;
        pcl::PointCloud<PointXYZIT>::Ptr cloud(new pcl::PointCloud<PointXYZIT>());

        if (pcl::io::loadPCDFile<PointXYZIT>(pcd_file, *cloud) == -1)
        {
            PCL_ERROR("couldn't read file");
            return 0;
        }

        static unsigned int index = 1;
        std::string pcd_filename = pcd_save_folder + "/" + std::to_string(index) + ".pcd";
        std::cout << "pcd_filename " << pcd_filename << std::endl;

        WritePcdFile(index, pcd_filename, cloud);
        // std::string str = line.substr(0, line.length() - 4);
        // fprintf(stamp_file_handle_, "%u %s\n", index, str.c_str());

        ++index;
    }

    if (stamp_file_handle_ != nullptr)
    {
        fclose(stamp_file_handle_);
    }

    return 0;
}

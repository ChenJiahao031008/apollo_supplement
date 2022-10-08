#include <memory>
#include <imgui.h>
#include <portable-file-dialogs.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <glk/lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>

#include <guik/gl_canvas.hpp>
#include <guik/progress_modal.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <hdl_graph_slam/version_modal.hpp>

#include <ros/package.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace hdl_graph_slam {

/**
 * @brief Odometry frame
 *
 */
struct OdometryFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<OdometryFrame>;

public:
  OdometryFrame(const std::string& raw_cloud_path, const Eigen::Isometry3d& pose, unsigned long stamp_sec, unsigned long stamp_usec) : raw_cloud_path(raw_cloud_path), cloud_(nullptr), pose(pose), pose_imu(pose), stamp_sec(stamp_sec), stamp_usec(stamp_usec) {}

  OdometryFrame(const std::string& raw_cloud_path, const Eigen::Isometry3d& pose) : raw_cloud_path(raw_cloud_path), cloud_(nullptr), pose(pose), pose_imu(pose), downsample_resolution(0.1f) {
    char underscore;
    std::stringstream sst(boost::filesystem::path(raw_cloud_path).filename().string());
    sst >> stamp_sec >> underscore >> stamp_usec;
  }

public:
  static OdometryFrame::Ptr load(const std::string& cloud_filename, const std::string& pose_filename) {
    std::ifstream ifs(pose_filename);
    if(!ifs) {
      std::cerr << "error : failed to load " << pose_filename << std::endl;
      return nullptr;
    }

    Eigen::Matrix4d mat;
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        ifs >> mat(i, j);
      }
    }

    Eigen::Isometry3d pose(mat);

    return std::make_shared<OdometryFrame>(cloud_filename, pose);
  }

  static OdometryFrame::Ptr load(const std::string& cloud_filename, const Eigen::Isometry3d& pose, unsigned long stamp_sec, unsigned long stamp_usec) {
    return std::make_shared<OdometryFrame>(cloud_filename, pose, stamp_sec, stamp_usec);
  }

  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud() {
    return cloud(downsample_resolution);
  }

  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud(float downsample_resolution) {
    if(cloud_ == nullptr || std::abs(this->downsample_resolution - downsample_resolution) > 0.01f) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud = load_cloud(raw_cloud_path);
      pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

      pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
      voxel_grid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      voxel_grid.setInputCloud(raw_cloud);
      voxel_grid.filter(*downsampled);

      this->downsample_resolution = downsample_resolution;
      cloud_ = downsampled;
      cloud_buffer.reset(new glk::PointCloudBuffer(cloud_));
    }

    return cloud_;
  }

  void draw(glk::GLSLShader& shader, float downsample_resolution = 0.1f) {
    cloud(downsample_resolution);

    shader.set_uniform("color_mode", 0);
    shader.set_uniform("model_matrix", pose.cast<float>().matrix());
    cloud_buffer->draw(shader);

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
    shader.set_uniform("apply_keyframe_scale", true);
    auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    sphere.draw(shader);
    shader.set_uniform("apply_keyframe_scale", false);
  }
private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr load_cloud(const std::string& filename) const {
    std::string extension = boost::filesystem::path(filename).extension().string();
    if(extension == ".pcd") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::io::loadPCDFile(filename, *cloud);
      return cloud;
    } else if (extension == ".txt") {
      std::ifstream ifs(filename);
      if(!ifs) {
        std::cerr << "warning: failed to open " << filename << std::endl;
        return nullptr;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      while(!ifs.eof()) {
        std::string line;
        std::getline(ifs, line);
        if(line.empty()) {
          continue;
        }

        std::stringstream sst(line);

        pcl::PointXYZI pt;
        sst >> pt.x >> pt.y >> pt.z >> pt.intensity;

        cloud->push_back(pt);
      }

      cloud->is_dense = false;
      cloud->width = cloud->size();
      cloud->height = 1;

      return cloud;
    }

    std::cerr << "unknown extension: " << extension << std::endl;
    std::cerr << "input file : " << filename << std::endl;
    abort();
    return nullptr;
  }

public:
  unsigned long stamp_sec;
  unsigned long stamp_usec;
  double timestamp_;
  Eigen::Isometry3d pose;
  Eigen::Isometry3d pose_imu;
  std::string raw_cloud_path;

private:
  float downsample_resolution;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

  std::unique_ptr<glk::PointCloudBuffer> cloud_buffer;
};

/**
 * @brief Odometry frame set
 *
 */
class OdometrySet {
public:
  enum Format {
    ROS = 1,
    YOKOZUKA = 2,
    APOLLO = 3
  };

  OdometrySet(guik::ProgressInterface& progress, const std::string& directory, Format format) {
    switch(format) {
      default:
      case ROS:
        load_ros(progress, directory);
        break;

      case YOKOZUKA:
        load_yokozuka(progress, directory);
        break;

      case APOLLO:
        load_apollo(progress, directory);
        break;
    }
  }

  void select_keyframes(float keyframe_delta_x, float keyframe_delta_angle) {
    if(frames.empty()) {
      return;
    }

    keyframes.clear();
    keyframes.push_back(frames.front());
    for(const auto& frame : frames) {
      const auto& last_keyframe_pose = keyframes.back()->pose;
      const auto& current_frame_pose = frame->pose;

      Eigen::Isometry3d delta = last_keyframe_pose.inverse() * current_frame_pose;
      double delta_x = delta.translation().norm();
      double delta_angle = Eigen::AngleAxisd(delta.linear()).angle();

      if(delta_x > keyframe_delta_x || delta_angle > keyframe_delta_angle) {
        keyframes.push_back(frame);
      }
    }
  }

  Eigen::Isometry3d interpolate_pose(const double ref_timestamp) {
    size_t index = std::lower_bound(ins_timestamps_.begin(), ins_timestamps_.end(), ref_timestamp) - ins_timestamps_.begin();
    index = std::max(1lu, std::min(index, ins_timestamps_.size() - 1));

    double cur_timestamp = ins_timestamps_[index];
    double pre_timestamp = ins_timestamps_[index - 1];

    double t = (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);

    Eigen::Isometry3d pre_pose = ins_poses_[index - 1];
    Eigen::Isometry3d cur_pose = ins_poses_[index];
    Eigen::Quaterniond pre_quatd(pre_pose.linear());
    Eigen::Translation3d pre_transd(pre_pose.translation());
    Eigen::Quaterniond cur_quatd(cur_pose.linear());
    Eigen::Translation3d cur_transd(cur_pose.translation());

    Eigen::Quaterniond res_quatd = pre_quatd.slerp(1 - t, cur_quatd);

    Eigen::Translation3d re_transd;
    re_transd.x() = pre_transd.x() * t + cur_transd.x() * (1 - t);
    re_transd.y() = pre_transd.y() * t + cur_transd.y() * (1 - t);
    re_transd.z() = pre_transd.z() * t + cur_transd.z() * (1 - t);

    Eigen::Isometry3d pose = re_transd * res_quatd;

    return pose;
  }

  void update_dt(const double dt) {
    if(keyframes.empty()) {
      return;
    }

    for(auto &frame : keyframes) {
      frame->pose_imu = interpolate_pose(frame->timestamp_ + dt);
    }
  }

  void update_extrinsics(const Eigen::Isometry3d &Timu_lidar) {
    if(keyframes.empty()) {
      return;
    }

    for(auto &frame : keyframes) {
      frame->pose = frame->pose_imu * Timu_lidar;
    }
  }

  void draw(glk::GLSLShader& shader, float downsample_resolution = 0.1f) {
    for(const auto& keyframe : keyframes) {
      keyframe->draw(shader, downsample_resolution);
    }
  }

  bool save(guik::ProgressInterface& progress, const std::string& dst_directory) {
    if(keyframes.empty()) {
      return false;
    }

    if(!save_graph(progress, dst_directory + "/graph.g2o")) {
      return false;
    }

    if(!save_keyframes(progress, dst_directory)) {
      return false;
    }

    return true;
  }

private:
  void load_ros(guik::ProgressInterface& progress, const std::string& directory) {
    progress.set_text("sweeping the directory");

    boost::filesystem::directory_iterator itr(directory);
    boost::filesystem::directory_iterator end;

    std::vector<std::string> filenames;
    for(itr; itr != end; itr++) {
      if(itr->path().extension() != ".pcd") {
        continue;
      }

      std::string odom_filename = itr->path().parent_path().string() + "/" + itr->path().stem().string() + ".odom";

      if(!boost::filesystem::exists(odom_filename)) {
        continue;
      }

      filenames.push_back(itr->path().stem().string());
    }

    progress.set_text("loading odometry frames");
    progress.set_maximum(filenames.size());
    progress.set_current(0);

    std::sort(filenames.begin(), filenames.end());
    for(const auto& filename : filenames) {
      progress.increment();

      auto frame = OdometryFrame::load(directory + "/" + filename + ".pcd", directory + "/" + filename + ".odom");
      if(frame == nullptr) {
        continue;
      }

      frames.push_back(frame);
    }
  }

  void load_yokozuka(guik::ProgressInterface& progress, const std::string& directory) {
    progress.set_text("loading graph structure");
    progress.increment();

    std::ifstream ifs(directory + "/Scan3Graph.txt");
    if(!ifs) {
      return;
    }

    std::vector<unsigned long> frame_ids;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> frame_poses;

    while(!ifs.eof()) {
      std::string line;
      std::getline(ifs, line);

      if(line.empty()) {
        continue;
      }

      std::stringstream sst(line);

      long id;
      Eigen::Vector3d trans;
      Eigen::Quaterniond quat;

      sst >> id >> trans.x() >> trans.y() >> trans.z() >> quat.x() >> quat.y() >> quat.z() >> quat.w();

      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = trans;
      pose.linear() = quat.toRotationMatrix();

      frame_ids.push_back(id);
      frame_poses.push_back(pose);
    }

    progress.set_text("loading frames");
    progress.set_maximum(frame_ids.size());
    for(int i = 0; i < frame_ids.size(); i++) {
      progress.increment();

      const auto& id = frame_ids[i];
      const auto& pose = frame_poses[i];

      std::string cloud_filename = (boost::format("%s/Scan3Map/submap-%03d.txt") % directory % id).str();
      auto frame = OdometryFrame::load(cloud_filename, pose, id, 0);

      if(frame == nullptr) {
        continue;
      }

      frames.push_back(frame);
    }
  }

  void load_apollo(guik::ProgressInterface& progress, const std::string& directory) {
    progress.set_text("loading graph structure");
    progress.increment();

    std::string pcd_timestamp_file = directory + "/pcd_timestamp.txt";
    std::vector<unsigned int> frame_ids;
    std::vector<double> pcd_times;
    FILE *file_pcd_timestamp = fopen(pcd_timestamp_file.c_str(), "r");
    if (file_pcd_timestamp)
    {
      unsigned int index;
      double timestamp;
      static constexpr int kSize = 2;
      while (fscanf(file_pcd_timestamp, "%u %lf\n", &index, &timestamp) == kSize)
      {
          pcd_times.push_back(timestamp);
          frame_ids.push_back(index);
      }
      fclose(file_pcd_timestamp);
    }
    else
    {
        std::cerr << "Can't open file to read: " << pcd_timestamp_file << std::endl;
    }

    std::string ins_file_path = directory + "/odometry_loc.txt";
    FILE *file_ins = fopen(ins_file_path.c_str(), "r");
    ins_poses_.clear();
    ins_timestamps_.clear();
    if (file_ins)
    {
      unsigned int index;
      double timestamp;
      double x, y, z;
      double qx, qy, qz, qr;
      double std_x, std_y, std_z;
      static constexpr int kSize = 12;
      while (fscanf(file_ins, "%u %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                    &index, &timestamp, &x, &y, &z, &qx, &qy, &qz, &qr, &std_x,
                    &std_y, &std_z) == kSize)
      {
        static Eigen::Vector3d trans_init = Eigen::Vector3d(x, y, z);
        Eigen::Translation3d trans(Eigen::Vector3d(x, y, z) - trans_init);
        Eigen::Quaterniond quat(qr, qx, qy, qz);
        ins_poses_.push_back(trans * quat);
        ins_timestamps_.push_back(timestamp);
      }
      fclose(file_ins);
    }
    else
    {
      std::cerr << "Can't open file to read: " << ins_file_path << std::endl;
    }

    progress.set_text("loading frames");
    progress.set_maximum(frame_ids.size());
    for(int i = 0; i < frame_ids.size(); i++) {
      progress.increment();

      const auto& id = frame_ids[i];
      const auto& pose = interpolate_pose(pcd_times[i]);

      std::string cloud_filename = directory + "/" + std::to_string(id) + ".pcd";
      auto frame = OdometryFrame::load(cloud_filename, pose, id, 0);
      frame->timestamp_ = pcd_times[i];

      if(frame == nullptr) {
        continue;
      }

      frames.push_back(frame);
    }
  }

  bool save_graph(guik::ProgressInterface& progress, const std::string& filename) const {
    progress.set_text("save graph file");
    progress.increment();

    std::ofstream ofs(filename);
    if(!ofs) {
      return false;
    }

    for(int i = 0; i < keyframes.size(); i++) {
      std::unique_ptr<g2o::VertexSE3> v(new g2o::VertexSE3());
      v->setEstimate(keyframes[i]->pose);

      ofs << "VERTEX_SE3:QUAT " << i << " ";
      v->write(ofs);
      ofs << std::endl;
    }
    ofs << "FIX 0" << std::endl;

    for(int i = 0; i < keyframes.size() - 1; i++) {
      const auto& delta_pose = keyframes[i]->pose.inverse() * keyframes[i + 1]->pose;
      std::unique_ptr<g2o::EdgeSE3> e(new g2o::EdgeSE3());
      e->setMeasurement(delta_pose);

      Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
      inf.block<3, 3>(0, 0) *= 10.0;
      inf.block<3, 3>(3, 3) *= 20.0;

      e->setInformation(inf);
      ofs << "EDGE_SE3:QUAT " << i << " " << i + 1 << " ";
      e->write(ofs);
      ofs << std::endl;
    }

    ofs.close();

    return true;
  }

  bool save_keyframes(guik::ProgressInterface& progress, const std::string& directory) const {
    for(int i = 0; i < keyframes.size(); i++) {
      std::string keyframe_directory = (boost::format("%s/%06d") % directory % i).str();
      boost::filesystem::create_directories(keyframe_directory);

      boost::filesystem::copy_file(keyframes[i]->raw_cloud_path, keyframe_directory + "/raw.pcd");
      pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *keyframes[i]->cloud());

      std::ofstream ofs(keyframe_directory + "/data");
      if(!ofs) {
        return false;
      }

      ofs << "stamp " << keyframes[i]->stamp_sec << " " << keyframes[i]->stamp_usec << std::endl;
      ofs << "estimate" << std::endl << keyframes[i]->pose.matrix() << std::endl;
      ofs << "odom " << std::endl << keyframes[i]->pose.matrix() << std::endl;
      ofs << "id " << i << std::endl;
    }

    return true;
  }

private:
  std::vector<OdometryFrame::Ptr> frames;
  std::vector<OdometryFrame::Ptr> keyframes;

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> ins_poses_;
  std::vector<double> ins_timestamps_;
};

/**
 * @brief Application to convert an odometry sequence into the graph description format
 *
 */
class Odometry2GraphApplication : public guik::Application {
public:
  Odometry2GraphApplication() : Application() {}
  ~Odometry2GraphApplication() {}

  /**
   * @brief initialize the application
   *
   * @param size            window size
   * @param glsl_version    glsl version
   * @return if successfully initialized
   */
  bool init(const char* window_name, const Eigen::Vector2i& size, const char* glsl_version = "#version 330") override {
    if(!Application::init(window_name, size, glsl_version)) {
      return false;
    }

    framebuffer_size = size;
    progress.reset(new guik::ProgressModal("progress modal"));

    std::string package_path = ros::package::getPath("interactive_slam");
    std::string data_directory = package_path + "/data";

    main_canvas.reset(new guik::GLCanvas(data_directory, size));
    if(!main_canvas->ready()) {
      close();
    }

    version_modal.reset(new VersionModal());

    delta_updated = false;
    keyframe_delta_x = 0.1f;
    keyframe_delta_angle = 1.0f;
    downsample_resolution = 0.2f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 1.571785f;

    return true;
  }

  /**
   * @brief draw ImGui-based UI
   *
   */
  virtual void draw_ui() override {
    main_canvas->draw_ui();

    {
      ImGui::Begin("keyframe settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::DragFloat("downsample_resolution", &downsample_resolution, 0.01f, 0.01f, 1.0f);

      bool updated = false;
      updated |= ImGui::DragFloat("keyframe_delta_x", &keyframe_delta_x, 0.1f, 0.1f, 100.0f);
      updated |= ImGui::DragFloat("keyframe_delta_angle", &keyframe_delta_angle, 0.01f, 0.01f, 3.15f);
      updated |= ImGui::DragFloat("x", &x, 0.01f, 0.01f, 100.0f);
      updated |= ImGui::DragFloat("y", &y, 0.01f, 0.01f, 100.0f);
      updated |= ImGui::DragFloat("z", &z, 0.01f, 0.01f, 100.0f);
      updated |= ImGui::DragFloat("roll", &roll, 0.001f, 0.001f, 3.15f);
      updated |= ImGui::DragFloat("pitch", &pitch, 0.001f, 0.001f, 3.15f);
      updated |= ImGui::DragFloat("yaw", &yaw, 0.001f, 0.001f, 3.15f);
      updated |= ImGui::DragFloat("dt", &dt, 0.0001f, 0.0001f, 1.0f);

      if(delta_updated && !updated) {
        Eigen::Isometry3d Timu_lidar = Eigen::Isometry3d::Identity();
        Timu_lidar.translation() = Eigen::Vector3d(x, y, z);
        Timu_lidar.linear() =  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY()).toRotationMatrix();
        odometry_set->select_keyframes(keyframe_delta_x, keyframe_delta_angle);
        odometry_set->update_dt(dt);
        odometry_set->update_extrinsics(Timu_lidar);

        std::cout << "Timu_lidar translation\n" << Timu_lidar.translation() << std::endl;
        std::cout << "Timu_lidar rotation\n" << Timu_lidar.linear() << std::endl;
        Eigen::Quaterniond q(Timu_lidar.linear());
        std::cout << "Timu_lidar quaternion qx qy qz qw\n" << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      }
      delta_updated = updated;

      ImGui::End();
    }

    main_menu();
    mouse_control();
  }

  /**
   * @brief draw OpenGL-related things
   *
   */
  virtual void draw_gl() override {
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    main_canvas->bind();
    main_canvas->shader->set_uniform("color_mode", 2);
    main_canvas->shader->set_uniform("model_matrix", (Eigen::UniformScaling<float>(3.0f) * Eigen::Isometry3f::Identity()).matrix());

    const auto& coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
    coord.draw(*main_canvas->shader);

    main_canvas->shader->set_uniform("color_mode", 1);
    main_canvas->shader->set_uniform("model_matrix", (Eigen::Translation3f(Eigen::Vector3f::UnitZ() * -0.02) * Eigen::Isometry3f::Identity()).matrix());
    main_canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
    const auto& grid = glk::Primitives::instance()->primitive(glk::Primitives::GRID);
    grid.draw(*main_canvas->shader);

    main_canvas->shader->set_uniform("point_scale", 1.0f);
    if(odometry_set) {
      odometry_set->draw(*main_canvas->shader, downsample_resolution);
    }

    main_canvas->unbind();
    main_canvas->render_to_screen();

    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  }

  /**
   * @brief frame buffer size change callback
   * @param size  frame buffer size
   */
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size) override {
    main_canvas->set_size(size);
    framebuffer_size = size;
  }

private:
  /**
   * @brief draw main menu
   *
   */
  void main_menu() {
    ImGui::BeginMainMenuBar();

    OdometrySet::Format odometry_format;
    bool open_dialog = false;
    bool save_dialog = false;
    if(ImGui::BeginMenu("File")) {
      if(ImGui::BeginMenu("Open")) {
        if(ImGui::MenuItem("ROS")) {
          odometry_format = OdometrySet::ROS;
          open_dialog = true;
        }
        if(ImGui::MenuItem("Yoko")) {
          odometry_format = OdometrySet::YOKOZUKA;
          open_dialog = true;
        }
        if(ImGui::MenuItem("Apollo")) {
          odometry_format = OdometrySet::APOLLO;
          open_dialog = true;
        }

        ImGui::EndMenu();
      }

      if(ImGui::MenuItem("Save")) {
        save_dialog = true;
      }

      if(ImGui::MenuItem("Quit")) {
        close();
      }

      ImGui::EndMenu();
    }

    open(open_dialog, odometry_format);
    save(save_dialog);

    if(ImGui::BeginMenu("View")) {
      if(ImGui::MenuItem("Reset camera")) {
        main_canvas->reset_camera();
      }
      if(ImGui::MenuItem("Projection setting")) {
        main_canvas->show_projection_setting();
      }
      ImGui::EndMenu();
    }

    bool show_version = false;
    if(ImGui::BeginMenu("Help")) {
      if(ImGui::MenuItem("About")) {
        show_version = true;
      }

      ImGui::EndMenu();
    }

    if(show_version) {
      version_modal->open();
    }
    version_modal->run();

    ImGui::EndMainMenuBar();
  }

private:
  /**
   * @brief open odometry data
   *
   * @param open_dialog
   */
  void open(bool open_dialog, const OdometrySet::Format& format) {
    if(progress->run("open")) {
      auto result = progress->result<std::shared_ptr<OdometrySet>>();
      if(result == nullptr) {
        pfd::message message("Error", "failed to load odometry data", pfd::choice::ok);
        while(!message.ready()) {
          usleep(100);
        }
        return;
      }

      odometry_set = result;
      odometry_set->select_keyframes(keyframe_delta_x, keyframe_delta_angle);
      delta_updated = true;
    }

    if(!open_dialog) {
      return;
    }

    pfd::select_folder dialog("choose odometry directory");
    while(!dialog.ready()) {
      usleep(100);
    }

    if(dialog.result().empty()) {
      return;
    }

    std::string directory = dialog.result();
    auto open_task = [=](guik::ProgressInterface& p) { return std::make_shared<OdometrySet>(p, directory, format); };
    progress->open<std::shared_ptr<OdometrySet>>("open", open_task);
  }

  /**
   * @brief save graph description
   *
   * @param save_dialog
   */
  void save(bool save_dialog) {
    if(progress->run("save")) {
      auto result = progress->result<bool>();
      if(!result) {
        pfd::message message("Error", "failed to save graph data", pfd::choice::ok);
        while(!message.ready()) {
          usleep(100);
        }
        return;
      }
    }

    if(!save_dialog) {
      return;
    }

    pfd::select_folder dialog("choose odometry directory");
    while(!dialog.ready()) {
      usleep(100);
    }

    if(dialog.result().empty()) {
      return;
    }

    std::string directory = dialog.result();
    auto save_task = [this, directory](guik::ProgressInterface& p) { return odometry_set->save(p, directory); };
    progress->open<bool>("save", save_task);
  }

  /**
   * @brief mouse event handler
   *
   */
  void mouse_control() {
    ImGuiIO& io = ImGui::GetIO();
    if(!io.WantCaptureMouse) {
      main_canvas->mouse_control();
    }
  }

private:
  Eigen::Vector2i framebuffer_size;

  std::unique_ptr<guik::GLCanvas> main_canvas;
  std::unique_ptr<guik::ProgressModal> progress;

  std::unique_ptr<VersionModal> version_modal;

  bool delta_updated;
  float keyframe_delta_x;
  float keyframe_delta_angle;
  float downsample_resolution;
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  float dt;
  std::shared_ptr<OdometrySet> odometry_set;
};

}  // namespace hdl_graph_slam

/**
 * @brief main
 */
int main(int argc, char** argv) {
  std::unique_ptr<guik::Application> app(new hdl_graph_slam::Odometry2GraphApplication());

  if(!app->init("Odometry2Graph", Eigen::Vector2i(1920, 1080))) {
    return 1;
  }

  app->run();

  return 0;
}

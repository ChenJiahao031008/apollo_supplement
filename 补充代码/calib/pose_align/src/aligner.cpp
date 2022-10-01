#include "pose_align/aligner.h"
#include "pose_align/eigen_quaternion_parameterization.h"
#include "pose_align/pose_error.h"
#include "pose_align/sensors.h"
#include "pose_align/transform.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



namespace pose_align {

    void Aligner::align(Odom &odom_source, Odom &odom_target) {
        int odom_size = odom_target.getOdomTransformSize();
        std::cout<<"odom size is "<<odom_size<<std::endl;

        std::vector<Transform> odom_source_delta;
        std::vector<Transform> odom_target_delta;

        ceres::Problem problem;
        ceres::LossFunction *loss_func_ptr = new ceres::CauchyLoss(0.2);
        ceres::Solver::Summary summary;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 500;
        options.num_threads = 8;

        double translation[3] = {0, 0, 0};
        double quaternion[4] = {0, 0, 0., 1};

        int last_source_id = 0;

        for (int i = 0; i < odom_size; i++) {

            Transform last_src_pos = odom_target.getOdomTransformOfidx(last_source_id);
            Transform cur_src_pos = odom_target.getOdomTransformOfidx(i);
            Transform delta_src_pos = cur_src_pos.inverse() * last_src_pos;

            if(delta_src_pos.translation_.norm() < 1)
            {
                continue;
            }
            last_source_id = i;
            int last_id = i;
            for (int j = i + 1; j < odom_size; j++) {
                Transform src_pos = odom_target.getOdomTransformOfidx(last_id);
                Transform base_pos = odom_target.getOdomTransformOfidx(j);
                Transform delta_pos = base_pos.inverse() * src_pos;

                Transform delta_src_pos = base_pos.inverse() * cur_src_pos;

                Eigen::Vector3f euler_angle = delta_pos.rotation_.matrix().eulerAngles(2, 1, 0);
                double yaw = std::abs(euler_angle[0] * 180 / 3.1415926);
                double dist = delta_pos.translation_.norm();

                if (dist > 2 && delta_src_pos.translation_.norm() < 5) {
                    odom_source_delta.push_back(odom_source.getOdomTransformOfidx(i).inverse() * odom_source.getOdomTransformOfidx(j));
                    odom_target_delta.push_back(odom_target.getOdomTransformOfidx(i).inverse() * odom_target.getOdomTransformOfidx(j));
                    last_id = j;
                }
                
            }
        }

        std::cout <<" delta data size is "<<odom_target_delta.size()<<std::endl;

        for (size_t i = 0; i < odom_target_delta.size(); ++i) {
            Eigen::Affine3d source_transform;
            Eigen::Affine3d target_transform;

            source_transform.translation() = odom_source_delta[i].translation_.cast<double>();
            source_transform.linear() = odom_source_delta[i].rotation_.toRotationMatrix().cast<double>();

            target_transform.translation() = odom_target_delta[i].translation_.cast<double>();
            target_transform.linear() = odom_target_delta[i].rotation_.toRotationMatrix().cast<double>();

            ceres::CostFunction *cost_func =
                    new ceres::AutoDiffCostFunction<HandEyeCostFunction, 1, 1, 1, 1, 4>
                    (new HandEyeCostFunction(target_transform, source_transform));
            problem.AddResidualBlock(cost_func, loss_func_ptr, translation,
                                        translation + 1, translation + 2, quaternion);
        }

        ceres::LocalParameterization *local_para_ptr = new EigenQuaternionParameterization;
        problem.SetParameterization(quaternion, local_para_ptr);

        if (fix_extrinsic_z_) {
            problem.SetParameterBlockConstant(translation + 2);
        }

        ceres::Solve(options, &problem, &summary);
        Eigen::Quaternionf quat_result(quaternion[3], quaternion[0], quaternion[1],
                                        quaternion[2]);
        Eigen::Vector3f tras_result = Eigen::Vector3f(translation[0], translation[1], translation[2]);
        Transform pose_result;
        pose_result.rotation_ = quat_result;
        pose_result.translation_ = tras_result;

        pose_result = pose_result.inverse();

        std::cout<<"the extrinsic is "<<std::endl;
        std::cout<<"pose_result quaternion is "<<pose_result.rotation_.coeffs().transpose()<<std::endl;
        std::cout<<"pose_result eular is "<<pose_result.rotation_.matrix().eulerAngles(2, 0, 1).transpose() * 180 / 3.1415926<<std::endl;
        std::cout<<"pose_result translation is "<<pose_result.translation_.transpose()<<std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_pts(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_pts(new pcl::PointCloud<pcl::PointXYZ>);

        for(int i = 0; i < odom_size; i++){
            Transform pose_tgt = odom_target.getOdomTransformOfidx(i);
            pose_tgt = pose_result.inverse() * pose_tgt * pose_result;
            Eigen::Vector3f translation_tgt = pose_tgt.translation_;
            target_pts->points.push_back(pcl::PointXYZ(translation_tgt.x(), translation_tgt.y(), translation_tgt.z()));

            Transform pose_sor = odom_source.getOdomTransformOfidx(i);
            // pose_sor = pose_result.inverse() * pose_sor * pose_result;
            Eigen::Vector3f translation_sor = pose_sor.translation_;
            source_pts->points.push_back(pcl::PointXYZ(translation_sor.x(), translation_sor.y(), translation_sor.z()));

        }

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("trans_viewer"));
        viewer->setBackgroundColor(255, 255, 255);
        
        // viewer->

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color_handle(target_pts, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color_handle(source_pts, 0, 255, 0);

        viewer->addPointCloud(target_pts, target_color_handle, "target");
        viewer->addPointCloud(source_pts, source_color_handle, "source");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");

        viewer->spin();
        viewer->close();
    }

}  // namespace pose_align

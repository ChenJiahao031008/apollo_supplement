# Apollo&SVL联合仿真 （四）融合感知

## 1 简介

Apollo支持三种感知方式：基于激光点云的感知模型、基于相机的感知模型、基于融合感知的模型。支持多相机、多激光、毫米波雷达感知等多种传感设备。

## 2 视觉感知

- 确保存在定位模块/数据集有下列信息正确输出：

  `/apollo/sensor/camera/front_6mm/image`

  `/apollo/localization/pose`

  `/tf`及`/tf_static`

修改配置文件，确保检测到的障碍物信息向指定channel输出：

- 修改文件位置为：`modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`

- 修改文件内容为：

  ```
  output_final_obstacles : true
  output_obstacles_channel_name : "/apollo/perception/obstacles"
  ```


启动图像模块，等待待显存稳定（一般在2-3分钟左右）。

```bash
 cyber_launch start /apollo/modules/perception/production/launch/dev_kit_perception_camera.launch
```

![](Apollo&SVL%E8%81%94%E5%90%88%E4%BB%BF%E7%9C%9F%EF%BC%88%E5%9B%9B%EF%BC%89%E6%84%9F%E7%9F%A5%E8%9E%8D%E5%90%88.assets/2022-04-07%2013-51-50%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)



查看输出结果：（camera检测输出为三维目标框）

```
cyber_monitor
```

观察到`/apollo/perception/obstacles`中有数据输出。

![](Apollo&SVL%E8%81%94%E5%90%88%E4%BB%BF%E7%9C%9F%EF%BC%88%E5%9B%9B%EF%BC%89%E6%84%9F%E7%9F%A5%E8%9E%8D%E5%90%88.assets/2022-04-07%2013-52-47%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

**扩展：感知可视化**

- 参数配置

  打开``modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt``配置文件，更改如下参数选项

  ```c++
  frame_capacity : 15
  timestamp_offset : 0.2
  enable_visualization : true
  output_final_obstacles : true
  output_obstacles_channel_name : "/apollo/perception/obstacles"
  ```

- 程序更改

  打开`/home/t/apollo/modules/perception/camera/tools/offline/visualizer.cc`, 跳转到`void Visualizer::ShowResult_all_info_single_camera(...)函数体（约1365行）`，做如下更改

  ![image-20221002211352225](Apollo&SVL%E8%81%94%E5%90%88%E4%BB%BF%E7%9C%9F%EF%BC%88%E5%9B%9B%EF%BC%89%E6%84%9F%E7%9F%A5%E8%9E%8D%E5%90%88.assets/image-20221002211352225.png)

  通过opecv将感知结果在图像上进行显示，感知结果包括障碍物以及车道线。

- 程序编译后运行camera的感知单元

  ```bash
  bash apollo.sh build_opt
  cyber_launch start modules/perception/production/launch/dev_kit_perception_camera.launch
  ```

- 运行效果如下

![image-20221002205013721](Apollo&SVL%E8%81%94%E5%90%88%E4%BB%BF%E7%9C%9F%EF%BC%88%E5%9B%9B%EF%BC%89%E6%84%9F%E7%9F%A5%E8%9E%8D%E5%90%88.assets/image-20221002205013721.png)



## 3 激光感知

### 3.1 CNNSegmentation 感知

CNNSegmentation算法（以下简称cnnseg）是由百度研发、尚未开源的语义分割算法（而非传统上基于bbox的方法）。它分为`center offset`，`objectness`，`positiveness`，`object height`，`class probability`五个层级，由于鲁棒性和检测效果较好，后续如不额外声明，均以该方法作为基础。

cnnseg的模型权重文件位于：`modules/perception/production/data/perception/lidar/models/cnnseg/velodyne16/deploy.caffemodel`

1. 确保存在定位模块/数据集有下列信息正确输出：

   + `/apollo/sensor/lidar128/compensator/PointCloud2`
   + `/apollo/localization/pose`
   + `/tf`及`/tf_static`

2. 调整 `/apollo/modules/perception/production/dag/`文件夹下的 `dag_streaming_perception_dev_kit_lidar.dag`  ：

   ```yaml
    components {
       class_name : "SegmentationComponent"
       config {
         name: "Velodyne128Segmentation"
         config_file_path: "/apollo/modules/perception/production/conf/perception/lidar/velodyne128_segmentation_conf.pb.txt"
         flag_file_path: "/apollo/modules/perception/production/conf/perception/perception_common.flag"
         readers {
             channel: "/apollo/sensor/lidar128/compensator/PointCloud2"
           }
       }
     }
   ```

3. 创建/修改参数配置文件：

   + `modules/perception/production/conf/perception/lidar/velodyne16_segmentation_conf.pb.txt`：

        ```yaml
        sensor_name: "velodyne128"
        enable_hdmap: true
        lidar_query_tf_offset: 0
        lidar2novatel_tf2_child_frame_id: "lidar128"
        output_channel_name: "/perception/inner/SegmentationObjects"
        ```

   + `modules/perception/production/conf/perception/lidar/recognition_conf.pb.txt`：

        ```yaml
        main_sensor_name: "velodyne128"
        output_channel_name: "/perception/inner/PrefusedObjects"
        ```

   + `modules/perception/production/data/perception/lidar/models/multi_lidar_fusion/mlf_engine.conf`：

        ```yaml
        main_sensor: "velodyne128"
        use_histogram_for_match: true
        histogram_bin_size: 10
        output_predict_objects: false
        reserved_invisible_time: 0.3
        use_frame_timestamp: true
        ```

   + `modules/perception/production/conf/perception/fusion/fusion_component_conf.pb.txt`:

        ```yaml
        fusion_method: "ProbabilisticFusion"
        fusion_main_sensors: "velodyne128"
        object_in_roi_check: true
        radius_for_roi_object_check: 120
        output_obstacles_channel_name: "/apollo/perception/obstacles"
        output_viz_fused_content_channel_name: "/perception/inner/visualization/FusedObjects"
        ```

4. 启动感知模块，等待待显存稳定（一般在2分钟左右）

   ```shell
   cyber_launch start /apollo/modules/perception/production/launch/dev_kit_perception_lidar.launch
   ```

   ![](Apollo&SVL%E8%81%94%E5%90%88%E4%BB%BF%E7%9C%9F%EF%BC%88%E5%9B%9B%EF%BC%89%E6%84%9F%E7%9F%A5%E8%9E%8D%E5%90%88.assets/2022-04-07%2014-38-22%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

 5. 查看`/apollo/perception/obstacles`中是否由数据输出（激光检测输出为多边形）

    ```bash
    cyber_monitor
    ```

### 3.2 PointPillar感知

PointPillar是基于激光点云进行目标检测的经典开源算法，在apollo和autoware上均有其实现。训练模型权重详见文件：`modules/perception/production/data/perception/lidar/models/detection/point_pillars`。

1. 在3.1`CNNSegmentation`配置文件的基础上，仅修改 `/apollo/modules/perception/production/dag/` 文件夹下的 `dag_streaming_perception_dev_kit_lidar.dag` 文件，将`SegmentationComponent`更换为`DetectionComponent`。

     ```shell
     module_config {
       module_library : "/apollo/bazel-bin/modules/perception/onboard/component/libperception_component_lidar.so"
     
       components {
         class_name : "DetectionComponent"
         config {
           name: "Velodyne128Detection"
           config_file_path: "/apollo/modules/perception/production/conf/perception/lidar/velodyne16_detection_conf.pb.txt"
           flag_file_path: "/apollo/modules/perception/production/conf/perception/perception_common.flag"
           readers {
             channel: "/apollo/sensor/lidar128/compensator/PointCloud2"
           }
         }
       }
     
       components {
         class_name : "RecognitionComponent"
         config {
           name: "RecognitionComponent"
           config_file_path: "/apollo/modules/perception/production/conf/perception/lidar/recognition_conf.pb.txt"
           readers {
             channel: "/perception/inner/DetectionObjects"
           }
         }
       }
     
       components {
         class_name: "FusionComponent"
         config {
           name: "SensorFusion"
           config_file_path: "/apollo/modules/perception/production/conf/perception/fusion/fusion_component_conf.pb.txt"
           readers {
             channel: "/perception/inner/PrefusedObjects"
           }
         }
       }
     }
     ```

2. 启动感知模块，等待待显存稳定（一般在2分钟左右）

     ```shell
     cyber_launch start /apollo/modules/perception/production/launch/dev_kit_perception_lidar.launch
     ```

 启动感知模块，等待待显存稳定（一般在2分钟左右）

## 4 融合感知

1. 确保存在定位模块/数据集有下列信息正确输出：

   + `/apollo/sensor/lidar128/compensator/PointCloud2`
   + `/apollo/localization/pose`
   + `/tf`及`/tf_static`

2. 修改相关配置：

   与单传感器感知不同，融合感知需要将点云、相机的感知结果进行后融合。为了保证输出通道不被占用，首先需要将相机感知的通道做一定调整，将结果传给位于点云感知的融合模块：

   + 修改文件：`modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`

     ```yaml
     output_final_obstacles : true
     output_obstacles_channel_name : "/perception/obstacles"
     ```

   选择需要融合的主传感器，并设置输出通道：

   + 修改文件：`modules/perception/production/conf/perception/fusion/fusion_component_conf.pb.txt`

     ```yaml
     fusion_method: "ProbabilisticFusion"
     fusion_main_sensors: "velodyne128"
     fusion_main_sensors: "front_6mm"
     object_in_roi_check: true
     radius_for_roi_object_check: 120
     output_obstacles_channel_name: "/apollo/perception/obstacles"
     output_viz_fused_content_channel_name: "/perception/inner/visualization/FusedObjects"
     ```

3. 设置启动文件：修改`modules/perception/production/dag/dag_streaming_perception.dag`，内容如下：

   ```yaml
   module_config {
     module_library : "/apollo/bazel-bin/modules/perception/onboard/component/libperception_component_camera.so"
     components {
       class_name : "FusionCameraDetectionComponent"
       config {
         name: "FusionCameraComponent"
         config_file_path: "/apollo/modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt"
         flag_file_path: "/apollo/modules/perception/production/conf/perception/perception_common.flag"
       }
     }
   }
   module_config {
     module_library : "/apollo/bazel-bin/modules/perception/onboard/component/libperception_component_lidar.so"
    components {
       class_name : "SegmentationComponent"
       config {
         name: "Velodyne128Segmentation"
         config_file_path: "/apollo/modules/perception/production/conf/perception/lidar/velodyne128_segmentation_conf.pb.txt"
         flag_file_path: "/apollo/modules/perception/production/conf/perception/perception_common.flag"
         readers {
             channel: "/apollo/sensor/lidar128/compensator/PointCloud2"
           }
       }
     }
   
     components {
       class_name : "RecognitionComponent"
       config {
         name: "RecognitionComponent"
         config_file_path: "/apollo/modules/perception/production/conf/perception/lidar/recognition_conf.pb.txt"
         readers {
             channel: "/perception/inner/SegmentationObjects"
           }
       }
     }
     components {
       class_name: "FusionComponent"
       config {
         name: "SensorFusion"
         config_file_path: "/apollo/modules/perception/production/conf/perception/fusion/fusion_component_conf.pb.txt"
         readers {
             channel: "/perception/inner/PrefusedObjects"
           }
       }
     }
   }
   ```
   
4. 启动融合感知模块，等待待显存稳定（一般在5分钟左右）。

5. 查看输出结果：（三维目标框和多边形框）

   ```bash
   cyber_monitor
   ```

   观察到`/apollo/perception/obstacles`中有数据输出。

 6. 在`dreamviewer`最终效果如下：可以看出，**近处时为点云检测为主的多边形目标，远处时为相机检测到的矩形框**，兼具了两者的优势。

## 5 感知动态障碍物剔除

1. 录制`Random Traffic`场景下的数据集，包含行人，汽车等动态障碍物

2. 数据包解析，将Lidar点云以pcd文件格式保存，odometery，localization_pose话题消息保存成txt文件形式，并通过现行插值获取Lidar时间戳下的位姿信息。

   `"Usage: lidar_parse.sh [records folder][output folder]  [extrinsic_file] [lidar_type]"`

   ```bash
   bash scripts/lidar_parse.sh /apollo/data/bag/calibration /apollo/data/bag/calibration /apollo/modules/calibration/data/Lincoln2017MKZ/velodyne_params/velodyne128_novatel_extrinsics_example.yaml lidar128
   ```

3. 将上一步的解析文件复制到新建文件夹 `/apollo/data/bag/calibration/obstacle_filter`下后，启动感知模块下的动态障碍物剔除程序，将lidar点云中的动态障碍物进行剔除

   `Usage:/lidar_per.sh [pcd folder] [corrected_file] `

   ```bash
   bash scripts/lidar_per.sh /apollo/data/bag/calibration/parsed_data/00000/pcd  /apollo/data/bag/calibration/parsed_data/00000/pcd/corrected_poses.txt
   ```

4. 利用标定工具箱下的Lidar-Ins标定工具，对比动态障碍物剔除前后点云变化

   ```bash
   rosrun interactive_slam odometry2graph
   ```

    ![](Apollo&SVL%E8%81%94%E5%90%88%E4%BB%BF%E7%9C%9F%EF%BC%88%E5%9B%9B%EF%BC%89%E6%84%9F%E7%9F%A5%E8%9E%8D%E5%90%88.assets/2022-04-08%2019-52-14%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

   <img src="Apollo&SVL%E8%81%94%E5%90%88%E4%BB%BF%E7%9C%9F%EF%BC%88%E5%9B%9B%EF%BC%89%E6%84%9F%E7%9F%A5%E8%9E%8D%E5%90%88.assets/image-20220409135251432.png" alt="image-20220409135251432" style="zoom: 50%;" />

   右图为启动感知模块的point_pillars来对动态障碍物进行感知剔除后的效果，由于无法对每一帧的点云中的障碍物进行识别，导致有部分点云留至地图中，但明显可见，动态障碍物的点云明显较左图变得稀疏。

5. 利用建图工具`msf_simple_map_creator.sh`进行建图，对比剔除动态障碍物前后地图的变化。

   复制`/apollo/scripts/msf_simple_map_creator.sh`文件,并命名为`msf_simple_map_filter_creator.sh`,在脚本文件中添加动态障碍物剔除函数

   ```bash
   function map_obstacle_filter()
   {
     /apollo/bazel-bin/modules/perception/lidar/tools/offline_map_obstacle_filter \
       --pcd_folders $1 \
       --pose_files $2 \
       --work_root=/apollo/modules/perception/production \
       --use_hdmap=false \
       --enable_tracking=false \
       --use_tracking_info=true \
       --min_life_time=-0.1
   }
   
   ## 在  create_lossless_map "${DIR_NAME}/pcd" "${DIR_NAME}/pcd/corrected_poses.txt"上方插入
     map_obstacle_filter "${DIR_NAME}/pcd" "${DIR_NAME}/pcd/corrected_poses.txt"
   ```

   运行建图程序

   ```
   bash scripts/msf_simple_map_creator.sh /apollo/data/bag/calibration /apollo/modules/calibration/data/Lincoln2017MKZ/velodyne_params/velodyne128_novatel_extrinsics_example.yaml 10 /apollo/modules/map/data/svl_Borrage_Map lidar128
   ```

   运行剔除动态障碍物下的msf建图程序

   ```bash
   bash scripts/msf_simple_map_filter_creator.sh /apollo/data/bag/calibration /apollo/modules/calibration/data/Lincoln2017MKZ/velodyne_params/velodyne128_novatel_extrinsics_example.yaml 10 /apollo/modules/map/data/svl_Borrage_Map_filter lidar128
   ```

   建图效果对比如下：

   ![image-20220409015901932](Apollo&SVL%E8%81%94%E5%90%88%E4%BB%BF%E7%9C%9F%EF%BC%88%E5%9B%9B%EF%BC%89%E6%84%9F%E7%9F%A5%E8%9E%8D%E5%90%88.assets/image-20220409015901932.png)



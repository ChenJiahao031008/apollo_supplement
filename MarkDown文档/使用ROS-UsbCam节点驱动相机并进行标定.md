# 使用ROS-UsbCam节点驱动相机并进行标定

## 1.1 使用ROS usb_cam驱动相机

```
sudo apt-get install ros-melodic-usb-cam
```

## 1.2 修改launch文件

进入目录：

```
roscd usb_cam
cd launch
sudo gedit usb_cam-test.launch 
```

目前主要修改device和width两个参数，可以使用`ls /dev/video*`查看系统视频设备。

```
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <!-- modify the video device to your device -->
    <param name="video_device" value="/dev/video0" />
    <!-- modify the size of your device -->
    <param name="image_width" value="1920" />
    <param name="image_height" value="1080" />
    <param name="pixel_format" value="yuyv" />
    <param name="camera_frame_id" value="usb_cam" />
    <param name="io_method" value="mmap"/>
  </node>
  <node name="image_view" pkg="image_view" type="image_view" respawn="false" ou$
    <remap from="image" to="/usb_cam/image_raw"/>
    <param name="autosize" value="true" />
  </node>
</launch>
```

### 1.3 启动相机

## 2 使用ROS进行相机标定

### 2.1 运行标定程序

```
roslaunch usb_cam usb_cam-test.launch
```

运行前需要根据你的棋盘格修改参数

- 一个是size参数为棋盘格角点数量比如8x9=72个格子的棋盘格，角点个数为7x8=63个，size参数就要写7x8
- 另外一个参数为square，传入的参数为棋盘格一个小格子的宽度（注意单位为m）
- `image`,图像话题的原始数据默认为`camera:=/usb_cam`

```
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.045 image:=/usb_cam/image_raw camera:=/usb_cam
```

### 2.2 生成标定文件

标定完成后点击Calculate会稍微有点卡顿，不要担心后台正在进行标定，完成后下面的SAVE和COMMIT按钮变为可用状态，点击SAVE即可保存标定完成后的文件。

## 3 更改apollo中的配置文件

将标定获得的内参数据复制进入对应的行中

```
modules/calibration/data/mkz_example/camera_params/front_camera_intrinsics.yaml
header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: white_mkz_onsemi_obstacle
height: 1080
width: 1920
distortion_model: plumb_bob
D: [-0.54336, 0.26653, -0.00099, -0.00170, 0.00000]
K: [2033.39968, 0.0, 929.01881, 0.0, 2046.55356, 572.81049, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [1729.4515380859375, 0.0, 997.0791139046996, 0.0, 0.0, 1926.0577392578125, 571.4609883012963, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```


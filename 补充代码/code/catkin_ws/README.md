# 手动lidar和imu外参可视化调节工具
用来标定imu和激光时空外参使用，该工具读入运动补偿后的点云与组合惯导的轨迹，拼成一块大点云。要求的格式：读取apollo解包后的pcd/文件，里面为apollo格式的去完畸变的点云，和惯导轨迹文件odometry_loc.txt与激光时间戳文件pcd_timestamp.txt；
## 使用：
安装所需依赖库：

for ROS melodic

sudo apt-get install libglm-dev libglfw3-dev

sudo apt-get install ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-libg2o

catkin_make之后使用rosrun interactive_slam odometry2graph启动

启动之后点左上角File->Open->Apollo，选择同时存在.pcd和odometry_loc.txt与pcd_timestamp.txt的文件夹

之后等待读取点云

通过调节激光和imu外参x,y,z,roll,pitch,yaw，dt的值，各帧点云的位姿将会发生变化，大点云在各处的厚度也会发生变化。通过观察地面和墙的厚度，可以手调出一个相对好的外参。

墙厚主要可通过调节yaw修复；

地面厚主要可通过调节roll，pitch修复

如果组合惯导和激光有相对时延，需要调节dt

最后得到的外参打印在terminal上

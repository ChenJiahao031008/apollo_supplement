cmake_minimum_required(VERSION 3.0.2)
project(cam_collect)

## Compile as C++11, supported in ROS Kinetic and newer
set( CMAKE_BUILD_TYPE "Release" )
## c++11标准
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## 找到依赖安装
find_package(catkin REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  "/usr/include/eigen3/"
)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  geometry_msgs
)

catkin_package(
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} SHARED
  src/get_char_input.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 可执行节点
add_executable(${PROJECT_NAME}_node
  src/cam_mona.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

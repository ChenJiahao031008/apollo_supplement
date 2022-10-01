cmake_minimum_required(VERSION 2.8.3)
project(lidar2camera)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
link_directories(${Pangolin_LIBRARY_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED)

find_package(YamlCpp REQUIRED)
include_directories(${YAMLCPP_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN_ROOT})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(run_lidar2camera src/run_lidar2camera.cpp)
target_link_libraries(run_lidar2camera  ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY}
                     ${YAMLCPP_LIBRARY} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})


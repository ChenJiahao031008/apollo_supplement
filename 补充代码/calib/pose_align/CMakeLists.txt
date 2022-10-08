cmake_minimum_required(VERSION 3.0)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++14 -fext-numeric-literals")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wno-deprecated-register -O0 -Wall -g2 -ggdb")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# if you feel the program is slow when runnin, you can change the "Debug" as "Release"
set(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})

find_package(CERES REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(Boost 1.65 COMPONENTS filesystem program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
message(${Boost_LIBRARY_DIRS})
message(${Boost_LIBRARIES})

include_directories(include)

add_executable(pose_align 
  src/sensors.cpp
  src/aligner.cpp
  src/loader.cpp
  src/eigen_quaternion_parameterization.cpp
  src/pose_align_node.cpp
)
target_link_libraries(pose_align ${PCL_LIBRARIES} ${CERES_LIBS} ${Boost_LIBRARIES})

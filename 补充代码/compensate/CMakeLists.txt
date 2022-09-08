cmake_minimum_required(VERSION 2.8)
project(compensate)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

include_directories( "/usr/include/eigen3"
                     "/usr/local/include/yaml-cpp")

add_executable(compensate compensate.cpp)
target_link_libraries(compensate ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} /usr/local/lib/libyaml-cpp.a)


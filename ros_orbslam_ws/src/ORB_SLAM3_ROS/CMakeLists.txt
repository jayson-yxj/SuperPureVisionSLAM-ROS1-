cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_ros)
set(CMAKE_CXX_STANDARD 14)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenCV 4.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.4 not found.")
   endif()

message(STATUS "OPENCV VERSION: ${OpenCV_VERSION}")

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

message(STATUS "Pangolin_LIBRARIES: ${Pangolin_LIBRARIES}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  ImagePose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs      # 必须添加
  geometry_msgs    # 必须添加
)

include_directories(
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS} 
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
-lboost_system
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs    # 必须添加
    geometry_msgs  # 必须添加
    message_runtime
)

add_executable(orb_mono src/ros_mono.cc)

add_dependencies(orb_mono ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(orb_mono
${LIBS}
${catkin_LIBRARIES}
)
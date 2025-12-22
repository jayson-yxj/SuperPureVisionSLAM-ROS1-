# Install script for directory: /home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/src/ORB_SLAM3_ROS

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orb_slam3_ros/msg" TYPE FILE FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/src/ORB_SLAM3_ROS/msg/ImagePose.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orb_slam3_ros/cmake" TYPE FILE FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/build/ORB_SLAM3_ROS/catkin_generated/installspace/orb_slam3_ros-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/devel/include/orb_slam3_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/devel/share/roseus/ros/orb_slam3_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/devel/share/common-lisp/ros/orb_slam3_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/devel/share/gennodejs/ros/orb_slam3_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/devel/lib/python3/dist-packages/orb_slam3_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/devel/lib/python3/dist-packages/orb_slam3_ros")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/build/ORB_SLAM3_ROS/catkin_generated/installspace/orb_slam3_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orb_slam3_ros/cmake" TYPE FILE FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/build/ORB_SLAM3_ROS/catkin_generated/installspace/orb_slam3_ros-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orb_slam3_ros/cmake" TYPE FILE FILES
    "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/build/ORB_SLAM3_ROS/catkin_generated/installspace/orb_slam3_rosConfig.cmake"
    "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/build/ORB_SLAM3_ROS/catkin_generated/installspace/orb_slam3_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orb_slam3_ros" TYPE FILE FILES "/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/src/ORB_SLAM3_ROS/package.xml")
endif()


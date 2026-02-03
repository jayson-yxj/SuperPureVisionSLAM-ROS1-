# Install script for directory: /home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/pub_video

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/build/pub_video/catkin_generated/installspace/pub_video.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pub_video/cmake" TYPE FILE FILES
    "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/build/pub_video/catkin_generated/installspace/pub_videoConfig.cmake"
    "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/build/pub_video/catkin_generated/installspace/pub_videoConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pub_video" TYPE FILE FILES "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/pub_video/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pub_video" TYPE PROGRAM FILES "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/build/pub_video/catkin_generated/installspace/pub_video_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pub_video" TYPE PROGRAM FILES "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/build/pub_video/catkin_generated/installspace/pub_video_node_stereo.py")
endif()


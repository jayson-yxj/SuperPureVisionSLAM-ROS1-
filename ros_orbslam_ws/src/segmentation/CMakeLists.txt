cmake_minimum_required(VERSION 3.0.2)
project(segmentation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dynamic_segmentor_node.py
  scripts/demo.py
  scripts/export_onnx.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY checkpoints/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/checkpoints
)
# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "depth_maping: 1 messages, 0 services")

set(MSG_I_FLAGS "-Idepth_maping:/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(depth_maping_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg" NAME_WE)
add_custom_target(_depth_maping_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "depth_maping" "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg" "geometry_msgs/Point:sensor_msgs/Image:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(depth_maping
  "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depth_maping
)

### Generating Services

### Generating Module File
_generate_module_cpp(depth_maping
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depth_maping
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(depth_maping_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(depth_maping_generate_messages depth_maping_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg" NAME_WE)
add_dependencies(depth_maping_generate_messages_cpp _depth_maping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depth_maping_gencpp)
add_dependencies(depth_maping_gencpp depth_maping_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depth_maping_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(depth_maping
  "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depth_maping
)

### Generating Services

### Generating Module File
_generate_module_eus(depth_maping
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depth_maping
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(depth_maping_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(depth_maping_generate_messages depth_maping_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg" NAME_WE)
add_dependencies(depth_maping_generate_messages_eus _depth_maping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depth_maping_geneus)
add_dependencies(depth_maping_geneus depth_maping_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depth_maping_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(depth_maping
  "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depth_maping
)

### Generating Services

### Generating Module File
_generate_module_lisp(depth_maping
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depth_maping
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(depth_maping_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(depth_maping_generate_messages depth_maping_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg" NAME_WE)
add_dependencies(depth_maping_generate_messages_lisp _depth_maping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depth_maping_genlisp)
add_dependencies(depth_maping_genlisp depth_maping_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depth_maping_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(depth_maping
  "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depth_maping
)

### Generating Services

### Generating Module File
_generate_module_nodejs(depth_maping
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depth_maping
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(depth_maping_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(depth_maping_generate_messages depth_maping_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg" NAME_WE)
add_dependencies(depth_maping_generate_messages_nodejs _depth_maping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depth_maping_gennodejs)
add_dependencies(depth_maping_gennodejs depth_maping_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depth_maping_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(depth_maping
  "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depth_maping
)

### Generating Services

### Generating Module File
_generate_module_py(depth_maping
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depth_maping
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(depth_maping_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(depth_maping_generate_messages depth_maping_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/depth_maping/msg/ImagePose.msg" NAME_WE)
add_dependencies(depth_maping_generate_messages_py _depth_maping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depth_maping_genpy)
add_dependencies(depth_maping_genpy depth_maping_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depth_maping_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depth_maping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depth_maping
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(depth_maping_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(depth_maping_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(depth_maping_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depth_maping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depth_maping
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(depth_maping_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(depth_maping_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(depth_maping_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depth_maping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depth_maping
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(depth_maping_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(depth_maping_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(depth_maping_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depth_maping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depth_maping
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(depth_maping_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(depth_maping_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(depth_maping_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depth_maping)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depth_maping\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depth_maping
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(depth_maping_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(depth_maping_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(depth_maping_generate_messages_py geometry_msgs_generate_messages_py)
endif()

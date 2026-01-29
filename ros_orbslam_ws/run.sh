source devel/setup.bash 
gnome-terminal -- bash -c "source devel/setup.bash; rosrun depth_maping depth_maping_node.py ; exec bash"
gnome-terminal -- bash -c "source devel/setup.bash; roslaunch octomap_mapping octomap_mapping.launch ; exec bash"

rosrun orb_slam3_ros orb_mono ../Vocabulary/ORBvoc.txt ../MonoConfig/Fisheye.yaml

rosrun orb_slam3_ros orb_mono ../Vocabulary/ORBvoc.txt ../MonoConfig/turtlebot3.yaml

rosrun orb_slam3_ros orb_mono ../Vocabulary/ORBvoc.txt ../MonoConfig/USBCam.yaml

# stereo
rosrun orb_slam3_ros Stereo ../Vocabulary/ORBvoc.txt ../MonoConfig/ZED_Mini.yaml false
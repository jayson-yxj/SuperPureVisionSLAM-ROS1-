import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_publisher():
    rospy.init_node('fisheye_sub', anonymous=True)
    rospy.loginfo("fisheye pub中 ...")
    pub = rospy.Publisher('/fisheye/raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(30)
    cap = cv2.VideoCapture('/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/src/video/ORBSLAM3.mp4')

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(ros_image)
        rate.sleep() 

    cap.release() 

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
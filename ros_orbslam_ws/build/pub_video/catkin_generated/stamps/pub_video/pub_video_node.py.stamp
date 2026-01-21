import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

VIDEO_PATH = '/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/video/ORBSLAM3_low_pose.mp4'
VIDEO_PATH2 = '/home/sunteng/Desktop/HighTorque_vision/video/hightorque_room.mp4'
VIDEO_PATH3 = '/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/video/ORBSLAM3.mp4'

def image_publisher():
    rospy.init_node('fisheye_sub', anonymous=True)
    rospy.loginfo("fisheye pub中 ...")
    pub = rospy.Publisher('/fisheye/raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(30)
    cap = cv2.VideoCapture(VIDEO_PATH2)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # 🔥 修复：设置当前时间戳（视频回放时必须使用当前时间）
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = "camera"
            pub.publish(ros_image)
            cv2.imshow("Fisheye Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            rospy.logwarn("Failed to capture image")
            break
        rate.sleep() 

    cap.release() 

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
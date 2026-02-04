#!/usr/bin/env python3
"""
双目视频发布节点
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

LEFT_VIDEO_PATH = '/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/video/left.mp4'
RIGHT_VIDEO_PATH = '/home/sunteng/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws/src/video/right.mp4'

def image_publisher():
    rospy.init_node('stereo_video_publisher', anonymous=True)
    rospy.loginfo("双目视频发布节点启动...")
    
    pub_left = rospy.Publisher('/stereo/raw_left', Image, queue_size=10)
    pub_right = rospy.Publisher('/stereo/raw_right', Image, queue_size=10)

    bridge = CvBridge()
    rate = rospy.Rate(30)  # 30 FPS
    
    cap_left = cv2.VideoCapture(LEFT_VIDEO_PATH)
    cap_right = cv2.VideoCapture(RIGHT_VIDEO_PATH)
    
    # 检查视频是否成功打开
    if not cap_left.isOpened() or not cap_right.isOpened():
        rospy.logerr("无法打开视频文件！")
        return
    
    # 获取视频信息
    fps_left = cap_left.get(cv2.CAP_PROP_FPS)
    fps_right = cap_right.get(cv2.CAP_PROP_FPS)
    total_frames_left = int(cap_left.get(cv2.CAP_PROP_FRAME_COUNT))
    total_frames_right = int(cap_right.get(cv2.CAP_PROP_FRAME_COUNT))
    
    rospy.loginfo(f"左视频: {fps_left} FPS, {total_frames_left} 帧")
    rospy.loginfo(f"右视频: {fps_right} FPS, {total_frames_right} 帧")
    
    # 检查帧数是否匹配
    if total_frames_left != total_frames_right:
        rospy.logwarn(f"⚠️  左右视频帧数不匹配: {total_frames_left} vs {total_frames_right}")

    # 畸变校正参数
    fx = 365.485  # 焦距 X (像素) - 对于 640x360 图像
    fy = 365.485  # 焦距 Y (像素)
    cx = 314.486  # 光心 X (像素) - 图像宽度的一半
    cy = 179.739  # 光心 Y (像素) - 图像高度的一半

    k1=-0.0324796
    k2=0.013859
    k3=0.000264742
    k4=0.000606081

    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]])
    D = np.array([k1, k2, k3, k4])

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, (640, 360), cv2.CV_16SC2)

    frame_count = 0
    
    while not rospy.is_shutdown():
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        # 视频结束时循环播放
        if not ret_left or not ret_right:
            rospy.loginfo("视频播放完毕，重新开始...")
            cap_left.set(cv2.CAP_PROP_POS_FRAMES, 0)
            cap_right.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frame_count = 0
            continue

        # 畸变校正
        frame_left = cv2.remap(frame_left, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        frame_right = cv2.remap(frame_right, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # 关键修复：使用相同的时间戳
        current_time = rospy.Time.now()
        
        # 转换为 ROS 消息
        ros_image_left = bridge.cv2_to_imgmsg(frame_left, encoding="bgr8")
        ros_image_right = bridge.cv2_to_imgmsg(frame_right, encoding="bgr8")

        # 设置相同的时间戳和坐标系
        ros_image_left.header.stamp = current_time
        ros_image_left.header.frame_id = "camera_left"
        
        ros_image_right.header.stamp = current_time  # 相同时间戳！
        ros_image_right.header.frame_id = "camera_right"

        # 发布图像
        pub_left.publish(ros_image_left)
        pub_right.publish(ros_image_right)

        frame_count += 1
        if frame_count % 100 == 0:
            rospy.loginfo(f"已发布 {frame_count} 帧")

        # 可视化（可选）
        stereo = np.hstack((frame_left, frame_right))
        cv2.imshow("Stereo Camera", stereo)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.loginfo("用户退出")
            break
            
        rate.sleep()

    cap_left.release() 
    cap_right.release() 

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
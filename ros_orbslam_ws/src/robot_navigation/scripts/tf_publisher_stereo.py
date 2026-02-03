#!/usr/bin/env python3
"""
TF 发布节点（双目版本）
功能：将 ORB-SLAM3 的位姿转换为 TF 变换
发布 TF 树: map -> odom -> base_link
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from monster.msg import ImagePoseStereo
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import os

class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher', anonymous=False)
        
        # TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # 坐标系名称
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera')
        
        # 加载尺度因子（从配置文件）
        self.translation_scale = self._load_translation_scale()
        
        # 位姿缓存
        self.current_pose = None
        self.last_pose_time = None
        
        # 订阅 ORB-SLAM3 位姿
        self.pose_sub = rospy.Subscriber(
            '/orb_slam3/image_pose_stereo', 
            ImagePoseStereo, 
            self.pose_callback, 
            queue_size=10
        )
        
        # 发布里程计（可选，用于可视化）
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # 注释掉静态 TF 发布：URDF 已经定义了 base_link -> camera 的变换
        # 如果这里再发布会与 URDF 冲突，导致相机"分家"
        # self.publish_static_tf()
        
        # 定时发布 TF（即使没有新位姿也保持发布）
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)  # 20Hz
        
        rospy.loginfo("✓ TF 发布节点已启动")
        rospy.loginfo(f"  TF 树: {self.map_frame} -> {self.odom_frame} -> {self.base_link_frame}")
        rospy.loginfo(f"  注意: base_link -> camera 的 TF 由 URDF 提供")
        rospy.loginfo(f"  平移尺度因子: {self.translation_scale}")
    
    def _load_translation_scale(self):
        """从配置文件加载平移尺度因子"""
        try:
            # 配置文件路径（相对于当前脚本）
            config_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '../../monster/config/monster_config.yaml'
            )
            
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                scale = config.get('scale_factor', 1.0)
                rospy.loginfo(f"✓ 从配置文件加载尺度因子: {scale}")
                return scale
            else:
                rospy.logwarn(f"⚠️  配置文件不存在: {config_path}，使用默认值1.0")
                return 1.0
        except Exception as e:
            rospy.logwarn(f"⚠️  加载配置文件失败: {e}，使用默认值1.0")
            return 1.0
    
    def publish_static_tf(self):
        """
        发布静态 TF: base_link -> camera
        假设相机安装在机器人中心，朝向前方
        """
        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = self.base_link_frame
        static_tf.child_frame_id = self.camera_frame
        
        # 相机相对于 base_link 的位置（根据实际安装调整）
        static_tf.transform.translation.x = 0.0  # 前方偏移（米）
        static_tf.transform.translation.y = 0.0  # 左右偏移（米）
        static_tf.transform.translation.z = 3.0  # 高度偏移（米）
        
        # 相机朝向（单位四元数，默认朝前）
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(static_tf)
        rospy.loginfo("✓ 已发布静态 TF: base_link -> camera")
    
    def pose_callback(self, msg):
        """
        接收 ORB-SLAM3 位姿并缓存
        关键修复：ORB-SLAM3输出的是Twc（World->Camera），需要取逆得到Tcw（Camera->World）
        """
        # 提取 ORB-SLAM3 位姿（Twc: World到Camera的变换）
        pos_cv = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat_cv = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        # 步骤1：构建Twc变换矩阵（4x4）
        rot_cv = R.from_quat(quat_cv)
        T_wc = np.eye(4)
        T_wc[:3, :3] = rot_cv.as_matrix()
        T_wc[:3, 3] = pos_cv * self.translation_scale  # 应用尺度因子（从配置文件加载）
        
        # 步骤2：取逆得到Tcw（Camera在World中的位姿）
        T_cw = np.linalg.inv(T_wc)
        pos_cw = T_cw[:3, 3]
        rot_cw = R.from_matrix(T_cw[:3, :3])
        
        # 步骤3：坐标系转换 CV -> ROS（修正前进方向）
        # 原始映射会导致前进方向偏移90°，需要调整轴映射
        T_cv_to_ros = np.array([
            [1, 0,  0],   # ROS X = -CV X（修正前进方向）
            [0,  0,  1],   # ROS Y = CV Z
            [0, -1,  0]    # ROS Z = -CV Y
        ])
        
        # 位置转换
        pos_ros = T_cv_to_ros @ pos_cw
        
        # 姿态转换：使用共轭变换
        rot_cv_to_ros = R.from_matrix(T_cv_to_ros)
        rot_ros = rot_cv_to_ros * rot_cw * rot_cv_to_ros.inv()
        quat_ros = rot_ros.as_quat()  # 返回 [x, y, z, w]
        
        # 缓存
        self.current_pose = {
            'position': pos_ros.tolist(),
            'orientation': quat_ros.tolist(),
            'timestamp': msg.header.stamp
        }
        self.last_pose_time = rospy.Time.now()

    
    def timer_callback(self, event):
        """
        定时发布 TF（保持 TF 树连续）
        """
        if self.current_pose is None:
            rospy.logwarn_throttle(5, "⚠️  等待 ORB-SLAM3 位姿...")
            return
        
        # 检查位姿是否过期（超过1秒）
        if self.last_pose_time is not None:
            age = (rospy.Time.now() - self.last_pose_time).to_sec()
            if age > 1.0:
                rospy.logwarn_throttle(2, f"⚠️  位姿数据过期 ({age:.2f}s)")
        
        # 使用当前时间戳（避免外推错误）
        current_time = rospy.Time.now()
        
        # 发布 TF: map -> odom (假设 odom 与 map 重合，因为使用视觉 SLAM)
        self.publish_map_to_odom(current_time)
        
        # 发布 TF: odom -> base_link (使用 ORB-SLAM3 位姿)
        self.publish_odom_to_base_link(current_time)
        
        # 发布里程计消息
        self.publish_odometry(current_time)
    
    def publish_map_to_odom(self, timestamp):
        """
        发布 TF: map -> odom
        对于视觉 SLAM，通常 map 和 odom 重合（无漂移）
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        
        # 单位变换（无偏移）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odom_to_base_link(self, timestamp):
        """
        发布 TF: odom -> base_link
        使用 ORB-SLAM3 的位姿（Twc: Camera -> World）
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        
        # ORB-SLAM3 提供的是相机位姿，需要转换为 base_link
        # 假设 base_link 与 camera 重合（通过静态 TF 调整）
        t.transform.translation.x = self.current_pose['position'][0]
        t.transform.translation.y = self.current_pose['position'][1]
        t.transform.translation.z = self.current_pose['position'][2]

        # 测试固定位置
        # t.transform.translation.x = 0.
        # t.transform.translation.y = 10.
        # t.transform.translation.z = 0.
        
        t.transform.rotation.x = self.current_pose['orientation'][0]
        t.transform.rotation.y = self.current_pose['orientation'][1]
        t.transform.rotation.z = self.current_pose['orientation'][2]
        t.transform.rotation.w = self.current_pose['orientation'][3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odometry(self, timestamp):
        """
        发布里程计消息（用于可视化和调试）
        """
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame
        
        # 位置
        odom.pose.pose.position.x = self.current_pose['position'][0]
        odom.pose.pose.position.y = self.current_pose['position'][1]
        odom.pose.pose.position.z = self.current_pose['position'][2]

        # 测试固定位置
        # odom.pose.pose.position.x = 0.
        # odom.pose.pose.position.y = 10.
        # odom.pose.pose.position.z = 0.
        
        # 姿态
        odom.pose.pose.orientation.x = self.current_pose['orientation'][0]
        odom.pose.pose.orientation.y = self.current_pose['orientation'][1]
        odom.pose.pose.orientation.z = self.current_pose['orientation'][2]
        odom.pose.pose.orientation.w = self.current_pose['orientation'][3]
        
        # 6×6协方差矩阵展平，顺序：x, y, z, roll, pitch, yaw
        odom.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,   0.0,   0.0,    # x轴位置方差（0.01m²，误差±0.1m）
            0.0,  0.01, 0.0,  0.0,   0.0,   0.0,    # y轴位置方差（同x）
            0.0,  0.0,  0.5,   0.0,   0.0,   0.0,    # z轴位置方差（0.5m²，误差±0.7m，单目z误差大）
            0.0,  0.0,  0.0,   0.001, 0.0,   0.0,    # roll方差（0.001rad²，≈0.057度）
            0.0,  0.0,  0.0,   0.0,   0.001, 0.0,    # pitch方差（同roll）
            0.0,  0.0,  0.0,   0.0,   0.0,   0.001   # yaw方差（同roll）
        ]
        
        # 速度（暂时设为0，可以通过位姿差分计算）
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        # 速度协方差（适配位姿差分计算的速度）
        odom.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,    0.0,    0.0,     # 线速度vx方差
            0.0,  0.01, 0.0,  0.0,    0.0,    0.0,     # 线速度vy方差
            0.0,  0.0,  0.1,   0.0,    0.0,    0.0,     # 线速度vz方差
            0.0,  0.0,  0.0,   0.001,  0.0,    0.0,     # 角速度vroll方差
            0.0,  0.0,  0.0,   0.0,    0.001,  0.0,     # 角速度vpitch方差
            0.0,  0.0,  0.0,   0.0,    0.0,    0.001   # 角速度vyaw方差
        ]
        
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        tf_pub = TFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF 发布节点已关闭")

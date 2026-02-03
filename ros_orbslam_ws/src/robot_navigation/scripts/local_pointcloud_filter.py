#!/usr/bin/env python3
"""
局部点云过滤节点
功能：保留以机器人为中心的正方形范围内的点云，用于360度感知
"""

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
import struct

class LocalPointCloudFilter:
    def __init__(self):
        rospy.init_node('local_pointcloud_filter', anonymous=False)
        
        # 参数配置
        self.box_size = rospy.get_param('~box_size', 10.0)  # 正方形边长（米）
        self.input_topic = rospy.get_param('~input_topic', '/o3d_pointCloud')
        self.output_topic = rospy.get_param('~output_topic', '/local_pointcloud')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.update_rate = rospy.get_param('~update_rate', 5.0)  # Hz
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布器和订阅器
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.pointcloud_callback, queue_size=1)
        
        # 缓存最新的点云
        self.latest_cloud = None
        self.last_publish_time = rospy.Time.now()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("局部点云过滤节点已启动")
        rospy.loginfo(f"  输入话题: {self.input_topic}")
        rospy.loginfo(f"  输出话题: {self.output_topic}")
        rospy.loginfo(f"  机器人坐标系: {self.robot_frame}")
        rospy.loginfo(f"  过滤范围: {self.box_size}m × {self.box_size}m 正方形")
        rospy.loginfo(f"  更新频率: {self.update_rate} Hz")
        rospy.loginfo("=" * 60)
    
    def pointcloud_callback(self, msg):
        """接收点云并缓存"""
        self.latest_cloud = msg
        
        # 按照设定频率发布
        current_time = rospy.Time.now()
        if (current_time - self.last_publish_time).to_sec() >= 1.0 / self.update_rate:
            self.process_and_publish()
            self.last_publish_time = current_time
    
    def process_and_publish(self):
        """处理并发布局部点云"""
        if self.latest_cloud is None:
            return
        
        try:
            # 获取机器人在点云坐标系中的位置
            transform = self.tf_buffer.lookup_transform(
                self.latest_cloud.header.frame_id,  # 目标坐标系（点云坐标系）
                self.robot_frame,                    # 源坐标系（机器人坐标系）
                rospy.Time(0),                       # 最新的变换
                rospy.Duration(0.1)                  # 超时时间
            )
            
            # 提取机器人位置
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_z = transform.transform.translation.z
            
            # 定义过滤范围（以机器人为中心的正方形）
            half_size = self.box_size / 2.0
            x_min = robot_x - half_size
            x_max = robot_x + half_size
            y_min = robot_y - half_size
            y_max = robot_y + half_size
            
            # 读取点云数据
            points_list = []
            for point in pc2.read_points(self.latest_cloud, skip_nans=True):
                x, y, z = point[:3]
                
                # 检查点是否在正方形范围内
                if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                    points_list.append(point)
            
            if len(points_list) == 0:
                rospy.logwarn_throttle(5, "⚠️  过滤后点云为空")
                return
            
            # 创建新的点云消息
            filtered_cloud = self.create_pointcloud2(
                points_list,
                self.latest_cloud.header,
                self.latest_cloud.fields
            )
            
            # 发布
            self.pub.publish(filtered_cloud)
            
            # 日志
            original_count = self.latest_cloud.width * self.latest_cloud.height
            filtered_count = len(points_list)
            rospy.loginfo_throttle(
                2,
                f"✓ 点云过滤: {original_count} → {filtered_count} 点 "
                f"(保留 {filtered_count/original_count*100:.1f}%) "
                f"| 机器人位置: ({robot_x:.2f}, {robot_y:.2f}, {robot_z:.2f})"
            )
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"⚠️  TF查询失败: {e}")
    
    def create_pointcloud2(self, points_list, header, fields):
        """创建PointCloud2消息"""
        # 创建点云数据
        cloud_data = []
        
        for point in points_list:
            # 根据字段数量打包数据
            if len(fields) == 3:  # XYZ
                cloud_data.append(struct.pack('fff', point[0], point[1], point[2]))
            elif len(fields) == 4:  # XYZRGB 或 XYZI
                cloud_data.append(struct.pack('ffff', point[0], point[1], point[2], point[3]))
            elif len(fields) == 6:  # XYZRGB (分开的RGB)
                cloud_data.append(struct.pack('ffffff', *point[:6]))
            else:
                # 默认处理：打包所有字段
                fmt = 'f' * len(point)
                cloud_data.append(struct.pack(fmt, *point))
        
        # 创建PointCloud2消息
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points_list)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = sum([f.count * 4 for f in fields])  # 假设所有字段都是float32
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = b''.join(cloud_data)
        
        return cloud_msg

if __name__ == '__main__':
    try:
        filter_node = LocalPointCloudFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("局部点云过滤节点已关闭")
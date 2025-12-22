#!/usr/bin/env python3
import rospy
import numpy as np
import open3d as o3d
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import heapq
import tf
import tf2_ros
import tf2_geometry_msgs
from threading import Lock

class InteractivePathPlanner:
    def __init__(self):
        
        # 路径规划参数
        self.voxel_size = 0.05
        self.planning_lock = Lock()
        
        # 存储点云和地图数据
        self.voxel_grid = None
        self.octomap = None
        
        # 起点和终点
        self.start_point = None
        self.goal_point = None
        self.current_path = None
        
        # RViz交互订阅
        self.click_sub = rospy.Subscriber('/clicked_point', PointStamped, self.click_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # 发布器
        self.marker_pub = rospy.Publisher('/visualization_markers', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('/global_plan', Marker, queue_size=10)
        self.start_pub = rospy.Publisher('/rviz_start', Marker, queue_size=10)
        self.goal_pub = rospy.Publisher('/rviz_goal', Marker, queue_size=10)
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("🚀 交互式路径规划器已启动")
        rospy.loginfo("💡 在RViz中:")
        rospy.loginfo("   - 使用Publish Point工具设置起点")
        rospy.loginfo("   - 使用2D Nav Goal工具设置目标点")
        rospy.loginfo("   - 路径将自动规划并显示")
        
    def update_voxel_grid(self, new_voxel_grid):
        """更新体素网格地图"""
        with self.planning_lock:
            self.voxel_grid = new_voxel_grid
            # 如果有起点和终点，重新规划路径
            if self.start_point and self.goal_point:
                self.plan_path()

    def click_callback(self, msg):
        """处理RViz中的点击事件（设置起点）"""
        try:
            # 转换到地图坐标系
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, 
                                                     rospy.Time(0), rospy.Duration(1.0))
            point_transformed = tf2_geometry_msgs.do_transform_point(msg, transform)
            
            self.start_point = [
                point_transformed.point.x,
                point_transformed.point.y, 
                point_transformed.point.z
            ]
            
            rospy.loginfo(f"🎯 设置起点: {self.start_point}")
            self.publish_start_marker()
            
            # 如果有目标点，立即规划路径
            if self.goal_point:
                self.plan_path()
                
        except Exception as e:
            rospy.logwarn(f"坐标转换失败: {e}")

    def goal_callback(self, msg):
        """处理RViz中的目标点设置"""
        try:
            # 转换到地图坐标系
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, 
                                                     rospy.Time(0), rospy.Duration(1.0))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
            
            self.goal_point = [
                pose_transformed.pose.position.x,
                pose_transformed.pose.position.y,
                pose_transformed.pose.position.z
            ]
            
            rospy.loginfo(f"🎯 设置目标点: {self.goal_point}")
            self.publish_goal_marker()
            
            # 如果有起点，立即规划路径
            if self.start_point:
                self.plan_path()
                
        except Exception as e:
            rospy.logwarn(f"坐标转换失败: {e}")

    def plan_path(self):
        """执行路径规划"""
        if not self.voxel_grid:
            rospy.logwarn("尚无地图数据，无法规划路径")
            return
            
        with self.planning_lock:
            try:
                rospy.loginfo("🔄 开始路径规划...")
                
                # 执行A*路径规划
                path = self.a_star_3d_planning(self.start_point, self.goal_point)
                
                if path and len(path) > 1:
                    self.current_path = path
                    self.publish_path_marker(path)
                    rospy.loginfo(f"✅ 路径规划成功! 路径点: {len(path)}")
                else:
                    rospy.logwarn("❌ 未找到可行路径")
                    self.clear_path_display()
                    
            except Exception as e:
                rospy.logerr(f"路径规划错误: {e}")

    def a_star_3d_planning(self, start, goal):
        """3D A*路径规划算法"""
        if not self.voxel_grid:
            return None
            
        # 将起点和终点转换为体素坐标
        start_voxel = self.world_to_voxel(start)
        goal_voxel = self.world_to_voxel(goal)
        
        # 检查起点和终点是否在障碍物中
        if self.is_occupied(start_voxel) or self.is_occupied(goal_voxel):
            rospy.logwarn("起点或终点位于障碍物中!")
            return None
        
        # A*算法实现
        open_set = []
        heapq.heappush(open_set, (0, start_voxel))
        
        came_from = {}
        g_score = {start_voxel: 0}
        f_score = {start_voxel: self.heuristic(start_voxel, goal_voxel)}
        
        # 26连通方向
        directions = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    directions.append((dx, dy, dz))
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current == goal_voxel:
                return self.reconstruct_path(came_from, current, start_voxel)
            
            for direction in directions:
                neighbor = (
                    current[0] + direction[0],
                    current[1] + direction[1], 
                    current[2] + direction[2]
                )
                
                # 跳过障碍物和越界点
                if self.is_occupied(neighbor):
                    continue
                
                # 计算移动代价（对角线移动代价更高）
                move_cost = 1.0 if sum(abs(d) for d in direction) == 1 else 1.414
                tentative_g = g_score[current] + move_cost
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_voxel)
                    
                    if not any(neighbor == item[1] for item in open_set):
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None

    def is_occupied(self, voxel_coord):
        """检查体素是否被占用"""
        try:
            # 检查体素网格中是否存在该体素
            voxel_center = self.voxel_to_world(voxel_coord)
            # 这里需要根据您的体素网格实现进行检查
            # 简化实现：假设所有存在的体素都是障碍物
            return False  # 临时实现
        except:
            return False

    def world_to_voxel(self, world_point):
        """世界坐标转换为体素坐标"""
        x, y, z = world_point
        return (
            int(round(x / self.voxel_size)),
            int(round(y / self.voxel_size)), 
            int(round(z / self.voxel_size))
        )

    def voxel_to_world(self, voxel_coord):
        """体素坐标转换为世界坐标"""
        x, y, z = voxel_coord
        return (
            x * self.voxel_size,
            y * self.voxel_size,
            z * self.voxel_size
        )

    def heuristic(self, a, b):
        """启发式函数（欧几里得距离）"""
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def reconstruct_path(self, came_from, current, start):
        """重建路径"""
        path = [self.voxel_to_world(current)]
        while current in came_from:
            current = came_from[current]
            path.append(self.voxel_to_world(current))
        path.reverse()
        return path

    def publish_start_marker(self):
        """发布起点标记"""
        if not self.start_point:
            return
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.start_point[0]
        marker.pose.position.y = self.start_point[1]
        marker.pose.position.z = self.start_point[2]
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.start_pub.publish(marker)

    def publish_goal_marker(self):
        """发布目标点标记"""
        if not self.goal_point:
            return
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning" 
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_point[0]
        marker.pose.position.y = self.goal_point[1]
        marker.pose.position.z = self.goal_point[2]
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.goal_pub.publish(marker)

    def publish_path_marker(self, path):
        """发布路径可视化"""
        if not path or len(path) < 2:
            return
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "global_path"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 设置路径线条属性
        marker.scale.x = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # 添加路径点
        for point in path:
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)
        
        self.path_pub.publish(marker)

    def clear_path_display(self):
        """清除路径显示"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "global_path"
        marker.id = 2
        marker.action = Marker.DELETE
        self.path_pub.publish(marker)

    def run(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 定期更新可视化
            if self.current_path:
                self.publish_path_marker(self.current_path)
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = InteractivePathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
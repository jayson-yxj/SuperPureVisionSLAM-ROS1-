#!/usr/bin/env python3
"""
Depth Mapping ROS Node - 重构版本

使用模块化架构，保留所有原有功能
"""

import os
import sys
import cv2
import numpy as np
import torch
import pypose as pp
import rospy
import yaml
import json
import time
import shutil
from typing import Dict, Any, Optional

# ROS消息类型
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2

# 自定义消息
from depth_maping.msg import ImagePose

# 添加当前脚本目录到 Python 路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# 导入重构后的模块
from pipeline_manager import PipelineManager
from parallel_pipeline_manager import ParallelPipelineManager


class DepthMappingNode:
    """深度建图ROS节点（重构版）"""
    
    def __init__(self):
        """初始化节点"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("初始化 Depth Mapping Node (重构版)")
        rospy.loginfo("=" * 60)
        
        # 当前文件路径
        self.current_file_path = os.path.abspath(__file__)
        self.current_dir = os.path.dirname(self.current_file_path)
        
        # 加载配置
        self.config = self._load_config()
        
        # 初始化Pipeline Manager（根据配置选择串行或并行模式）
        parallel_config = self.config.get('parallel_processing', {})
        enable_parallel = parallel_config.get('enabled', False)
        
        if enable_parallel:
            self.pipeline = ParallelPipelineManager(config_dict=self.config)
            self.use_parallel = True
            rospy.loginfo("✓ 使用并行处理模式")
        else:
            self.pipeline = PipelineManager(config_dict=self.config)
            self.use_parallel = False
            rospy.loginfo("✓ 使用串行处理模式")
        
        # 相机参数
        camera_config = self.config['camera']
        self.fx = camera_config['fx']
        self.fy = camera_config['fy']
        self.cx = camera_config['cx']
        self.cy = camera_config['cy']
        
        # 畸变参数
        distortion = camera_config['distortion']
        self.K = np.array([[self.fx, 0.0, self.cx],
                          [0.0, self.fy, self.cy],
                          [0.0, 0.0, 1.0]], dtype=np.float64)
        self.D = np.array([distortion['k1'], distortion['k2'], 
                          distortion['k3'], distortion['k4']], dtype=np.float64)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), self.K, (640, 480), cv2.CV_16SC2
        )
        
        # 位姿处理
        self.translation_scale = self.config['pose']['translation_scale']
        
        # ROS相关
        self.bridge = CvBridge()
        self.frame_counter = 0
        self.last_msg_time = None
        self.is_shutdown = False
        self.is_first_frame = True
        
        # 重力对齐
        gravity_config = self.config.get('gravity_alignment', {})
        self.enable_gravity_alignment = gravity_config.get('enabled', True)
        self.last_gravity_estimate_time = 0
        self.gravity_estimate_interval = gravity_config.get('save_interval', 1.0)
        self.R_align = None
        self.last_R_align_load_time = 0
        self.R_align_check_interval = gravity_config.get('check_interval', 0.5)
        
        # ROS话题配置
        ros_config = self.config['ros']
        topics = ros_config['topics']
        publish_rate = ros_config['publish_rate']
        
        # 订阅器
        self.image_pose_sub = rospy.Subscriber(
            topics['input_image_pose'],
            ImagePose,
            self.image_pose_callback,
            queue_size=100
        )
        
        # 发布器
        self.pcl_pub = rospy.Publisher(
            topics['output_point_cloud'],
            PointCloud2,
            queue_size=10
        )
        
        self.map_pub = rospy.Publisher(
            topics['output_map'],
            OccupancyGrid,
            queue_size=1,
            latch=True
        )
        
        # 发布频率
        self.point_cloud_publish_rate = publish_rate['point_cloud']
        self.map_publish_rate = publish_rate['map']
        
        # 可视化（可选）
        vis_config = self.config.get('visualization', {})
        self.enable_visualization = vis_config.get('enabled', False)
        if self.enable_visualization:
            try:
                import open3d as o3d
                self.vis = o3d.visualization.VisualizerWithKeyCallback()
                window_size = vis_config.get('window_size', [1280, 960])
                self.vis.create_window(
                    window_name="Point Cloud",
                    width=window_size[0],
                    height=window_size[1],
                    visible=True
                )
                rospy.loginfo("✓ Open3D 可视化已启用")
            except Exception as e:
                rospy.logwarn(f"无法创建可视化窗口: {e}")
                self.enable_visualization = False
        
        # 性能监控
        profiling_config = self.config.get('profiling', {})
        self.enable_profiling = profiling_config.get('enabled', True)
        self.profiling_log_interval = profiling_config.get('log_interval', 5)
        self.last_profiling_log_time = 0
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("✓ Depth Mapping Node 初始化完成")
        rospy.loginfo("=" * 60)
    
    def image_pose_callback(self, data: ImagePose):
        """图像位姿回调函数"""
        if self.is_shutdown or rospy.is_shutdown():
            return
        
        callback_start_time = time.time()
        
        # 计算消息延迟
        msg_timestamp = data.header.stamp.to_sec()
        current_time = rospy.Time.now().to_sec()
        msg_delay = current_time - msg_timestamp
        
        # 跳过延迟过大的消息
        if msg_delay > 0.1:
            rospy.logwarn_throttle(1, f"⏭️  跳过旧消息（延迟 {msg_delay:.3f}s）")
            return
        
        # 计算消息间隔
        if self.last_msg_time is not None:
            msg_interval = msg_timestamp - self.last_msg_time
            rospy.loginfo_throttle(
                2,
                f"📊 消息延迟: {msg_delay:.3f}s | 间隔: {msg_interval:.3f}s ({1/msg_interval:.1f} Hz)"
            )
        self.last_msg_time = msg_timestamp
        
        self.frame_counter += 1
        
        # 转换图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data.image, "bgr8")
        except Exception as e:
            rospy.logwarn(f"图像转换失败: {e}")
            return
        
        # 提取位姿
        translation = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        quaternion = [data.pose.orientation.x, data.pose.orientation.y,
                     data.pose.orientation.z, data.pose.orientation.w]
        
        # 位姿变换（ORB-SLAM3: Tcw -> Twc）
        T_pp = pp.SE3(torch.tensor(translation + quaternion))  # Tcw
        T_pp_inv = pp.Inv(T_pp)  # Twc
        
        # 缩放平移向量
        original_translation = T_pp_inv.translation()
        original_rotation = T_pp_inv.rotation()
        new_translation = original_translation * self.translation_scale
        T_pp_inv_scaled = pp.SE3(torch.cat([new_translation, original_rotation]))
        
        # 去畸变
        raw_image = cv2.cvtColor(cv_image.copy(), cv2.COLOR_BGR2RGB)
        undistorted_frame = cv2.remap(raw_image, self.map1, self.map2, cv2.INTER_LINEAR)
        
        # 显示去畸变图像
        cv2.imshow("Undistorted Frame", undistorted_frame)
        cv2.waitKey(1)
        
        # 初始化重力估计目录
        if self.is_first_frame:
            self.is_first_frame = False
            if self.enable_gravity_alignment:
                ge_info_dir = f"{self.current_dir}/GE_information"
                if not os.path.exists(ge_info_dir):
                    os.makedirs(ge_info_dir)
                    rospy.loginfo(f"✓ 创建重力估计目录: {ge_info_dir}")
                else:
                    rospy.loginfo(f"✓ 清空重力估计目录: {ge_info_dir}")
                    self._clear_folder(ge_info_dir)
        
        # 定期保存图像和位姿用于重力估计
        if self.enable_gravity_alignment:
            current_time = time.time()
            if current_time - self.last_gravity_estimate_time >= self.gravity_estimate_interval:
                self._save_image_and_pose(undistorted_frame, T_pp, msg_timestamp, self.frame_counter)
                self.last_gravity_estimate_time = current_time
            
            # 定期检查并加载R_align
            if current_time - self.last_R_align_load_time >= self.R_align_check_interval:
                self._load_R_align()
                self.last_R_align_load_time = current_time
        
        # 使用Pipeline处理帧
        camera_params = {
            'fx': self.fx,
            'fy': self.fy,
            'cx': self.cx,
            'cy': self.cy
        }
        
        if self.use_parallel:
            # 并行模式：异步提交任务
            success = self.pipeline.process_frame_async(
                undistorted_frame,
                T_pp_inv_scaled,
                camera_params
            )
            if not success:
                rospy.logwarn_throttle(2, "⚠️  处理队列已满，跳过当前帧")
            # 并行模式下不返回result，直接发布当前地图状态
            result = None
        else:
            # 串行模式：同步处理
            result = self.pipeline.process_frame(
                undistorted_frame,
                T_pp_inv_scaled,
                camera_params
            )
        
        # 发布点云
        if self.frame_counter % self.point_cloud_publish_rate == 0:
            try:
                pcd = self.pipeline.get_open3d_pointcloud()
                pcl_msg = self._o3d_to_ros_pointcloud2(pcd, "map")
                self.pcl_pub.publish(pcl_msg)
            except Exception as e:
                rospy.logwarn_throttle(5, f"发布点云失败: {e}")
        
        # 发布地图
        if self.frame_counter % self.map_publish_rate == 0:
            map_dict = self.pipeline.get_2d_map()
            if map_dict is not None:
                occ_msg = self._dict_to_occupancy_grid(map_dict)
                self.map_pub.publish(occ_msg)
                rospy.loginfo_throttle(
                    5,
                    f"已发布2D地图 ({occ_msg.info.width}x{occ_msg.info.height}, "
                    f"res={occ_msg.info.resolution}m)"
                )
        
        # 可视化更新
        if self.enable_visualization and self.frame_counter % 3 == 0:
            try:
                self.vis.poll_events()
                self.vis.update_renderer()
            except Exception as e:
                rospy.logwarn_throttle(10, f"渲染失败: {e}")
        
        # 性能日志
        if self.enable_profiling and not self.use_parallel:
            # 串行模式：使用result中的profiling数据
            if result:
                profiling = result.get('profiling')
                if profiling:
                    current_time = time.time()
                    if current_time - self.last_profiling_log_time >= self.profiling_log_interval:
                        total_time = profiling['total']
                        rospy.loginfo(
                            f"⏱️  性能: 深度={profiling['depth_estimation']*1000:.1f}ms, "
                            f"点云={profiling['point_cloud_generation']*1000:.1f}ms, "
                            f"总计={total_time*1000:.1f}ms ({1/total_time:.1f} FPS)"
                        )
                        self.last_profiling_log_time = current_time
        elif self.enable_profiling and self.use_parallel:
            # 并行模式：定期打印性能摘要
            current_time = time.time()
            if current_time - self.last_profiling_log_time >= self.profiling_log_interval:
                summary = self.pipeline.get_profiling_summary()
                if summary and 'total' in summary:
                    avg_fps = 1.0 / summary['total']['mean']
                    rospy.loginfo(
                        f"⏱️  并行处理性能: "
                        f"深度={summary['depth_estimation']['mean']*1000:.1f}ms, "
                        f"点云={summary['point_cloud_generation']['mean']*1000:.1f}ms, "
                        f"平均FPS={avg_fps:.1f}"
                    )
                self.last_profiling_log_time = current_time
    
    def _o3d_to_ros_pointcloud2(self, o3d_pcd, frame_id="map") -> PointCloud2:
        """将Open3D点云转换为ROS PointCloud2消息"""
        import open3d as o3d
        
        points = np.asarray(o3d_pcd.points)
        
        # 应用重力对齐矩阵
        if self.R_align is not None:
            points = points @ self.R_align.T
        
        # 判断是否有颜色
        if o3d_pcd.has_colors():
            colors = np.asarray(o3d_pcd.colors) * 255
            colors = colors.astype(np.uint8)
            
            # 创建结构化数组
            points_with_color = np.zeros(len(points), dtype=[
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('r', np.uint8), ('g', np.uint8), ('b', np.uint8)
            ])
            points_with_color['x'] = points[:, 0]
            points_with_color['y'] = points[:, 1]
            points_with_color['z'] = points[:, 2]
            points_with_color['r'] = colors[:, 0]
            points_with_color['g'] = colors[:, 1]
            points_with_color['b'] = colors[:, 2]
            
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('r', 12, PointField.UINT8, 1),
                PointField('g', 13, PointField.UINT8, 1),
                PointField('b', 14, PointField.UINT8, 1)
            ]
            data = points_with_color
        else:
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            data = points
        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        
        return pc2.create_cloud(header, fields, data)
    
    def _dict_to_occupancy_grid(self, map_dict: Dict[str, Any]) -> OccupancyGrid:
        """将地图字典转换为ROS OccupancyGrid消息"""
        occ_msg = OccupancyGrid()
        occ_msg.header.stamp = rospy.Time.now()
        occ_msg.header.frame_id = "map"
        
        occ_msg.info.resolution = map_dict['resolution']
        occ_msg.info.width = map_dict['width']
        occ_msg.info.height = map_dict['height']
        occ_msg.info.origin.position.x = map_dict['origin'][0]
        occ_msg.info.origin.position.y = map_dict['origin'][1]
        occ_msg.info.origin.position.z = 0.0
        occ_msg.info.origin.orientation.w = 1.0
        
        occ_msg.data = map_dict['data'].flatten().tolist()
        
        return occ_msg
    
    def _save_image_and_pose(self, image, T_pp, timestamp, frame_id):
        """保存图像和位姿用于重力估计"""
        try:
            ge_info_dir = f"{self.current_dir}/GE_information"
            
            # 保存图像
            image_path = os.path.join(ge_info_dir, "latest_img.png")
            cv2.imwrite(image_path, image)
            
            # 提取位姿
            R_cw = T_pp.rotation().matrix().cpu().numpy()
            t_cw = T_pp.translation().cpu().numpy()
            
            # 保存位姿
            pose_data = {
                'image_path': image_path,
                'timestamp': float(timestamp),
                'frame_id': int(frame_id),
                'R_cw': R_cw.tolist(),
                't_cw': t_cw.tolist()
            }
            
            pose_path = os.path.join(ge_info_dir, "latest_pose.json")
            with open(pose_path, 'w') as f:
                json.dump(pose_data, f, indent=2)
            
            rospy.loginfo_throttle(5, f"💾 已更新图像和位姿: frame_{frame_id}")
        except Exception as e:
            rospy.logwarn(f"保存图像和位姿失败: {e}")
    
    def _load_R_align(self):
        """加载重力对齐矩阵"""
        yaml_path = f"{self.current_dir}/GE_information/rotation_matrices.yaml"
        
        if not os.path.exists(yaml_path):
            return
        
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            R_align_new = np.array(data['R_align'])
            
            if self.R_align is None or not np.allclose(R_align_new, self.R_align):
                self.R_align = R_align_new
                rospy.loginfo(f"✓ 已加载重力对齐矩阵 (timestamp: {data.get('timestamp', 'N/A')})")
                g_aligned = data.get('g_aligned', [0, -1, 0])
                rospy.loginfo(f"  对齐后重力: [{g_aligned[0]:.4f}, {g_aligned[1]:.4f}, {g_aligned[2]:.4f}]")
        except Exception as e:
            rospy.logwarn_throttle(10, f"加载重力对齐矩阵失败: {e}")
    
    def _clear_folder(self, folder_path):
        """清空文件夹"""
        if not os.path.exists(folder_path):
            return
        
        for item in os.listdir(folder_path):
            item_path = os.path.join(folder_path, item)
            try:
                if os.path.isfile(item_path) or os.path.islink(item_path):
                    os.unlink(item_path)
                elif os.path.isdir(item_path):
                    shutil.rmtree(item_path)
            except Exception as e:
                rospy.logwarn(f"删除失败 {item_path}: {e}")
    
    def _load_config(self) -> Dict[str, Any]:
        """加载配置文件"""
        # 尝试从ROS参数服务器获取配置文件路径
        config_path = rospy.get_param('~config_path', None)
        
        if config_path is None:
            # 使用默认配置文件
            config_path = os.path.join(
                os.path.dirname(self.current_dir),
                'config',
                'default_config.yaml'
            )
        
        rospy.loginfo(f"加载配置文件: {config_path}")
        
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        return config
    
    def shutdown(self):
        """节点关闭处理"""
        self.is_shutdown = True
        rospy.loginfo("正在关闭节点...")
        
        # 关闭并行处理线程
        if self.use_parallel:
            self.pipeline.shutdown()
        
        # 等待回调完成
        rospy.sleep(0.5)
        
        # 保存点云
        try:
            output_path = os.path.join(self.current_dir, "pointCloud/HT_vslam.ply")
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            self.pipeline.save_map(output_path)
            rospy.loginfo(f"✓ 点云已保存到: {output_path}")
        except Exception as e:
            rospy.logwarn(f"✗ 保存点云失败: {e}")
        
        # 打印性能摘要
        if self.enable_profiling:
            self.pipeline.print_profiling_summary()
        
        # 关闭可视化
        if self.enable_visualization:
            try:
                self.vis.destroy_window()
                rospy.loginfo("✓ 可视化窗口已关闭")
            except:
                pass
        
        # 关闭OpenCV窗口
        try:
            cv2.destroyAllWindows()
        except:
            pass
        
        rospy.loginfo("节点已安全关闭")


def main():
    """主函数"""
    rospy.init_node("depth_mapping_node")
    
    node = None
    try:
        node = DepthMappingNode()
        rospy.loginfo("Depth Mapping Node 运行中...")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    except Exception as e:
        rospy.logerr(f"节点运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.shutdown()


if __name__ == "__main__":
    main()

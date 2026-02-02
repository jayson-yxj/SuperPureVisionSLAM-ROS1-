#!/usr/bin/env python3
"""
Monster 双目深度估计 ROS 节点（带建图功能）
订阅 /orb_slam3/image_pose_stereo 话题，进行实时深度估计、点云生成和地图构建
"""

import sys
import os

# 添加路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, 'Depth-Anything-V2-list3'))
sys.path.append(os.path.join(script_dir, 'core'))

import rospy
import cv2
import numpy as np
import torch
import pypose as pp
import yaml
import json
import time
from cv_bridge import CvBridge
from pathlib import Path

# 导入 Monster 相关模块
from core.monster import Monster
from core.utils.utils import InputPadder

# 导入点云和地图构建模块
from point_cloud import StereoPointCloudGenerator
from map_builder import OccupancyGridBuilder

# 导入 ROS 消息
from monster.msg import ImagePoseStereo
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'


def numpy_to_tensor(img_np):
    """将 numpy 图像转换为 tensor"""
    img = torch.from_numpy(img_np).permute(2, 0, 1).float()
    return img[None].to(DEVICE)


def disparity_to_depth(disparity, baseline, focal_length):
    """
    将视差转换为深度
    
    Args:
        disparity: 视差图 (像素)
        baseline: 基线距离 (米)
        focal_length: 焦距 (像素)
    
    Returns:
        depth: 深度图 (米)
    """
    # 避免除零
    disparity = np.maximum(disparity, 0.1)
    depth = (baseline * focal_length) / disparity
    return depth


class MonsterStereoNode:
    def __init__(self):
        """初始化 Monster 双目深度估计节点"""
        rospy.init_node('monster_stereo_node', anonymous=False)
        
        # 参数
        self.model_path = rospy.get_param('~model_path', '/home/sunteng/Desktop/HighTorque_vision/MonSter-plusplus/RT-MonSter++/checkpoints/Zero_shot.pth')
        self.encoder = rospy.get_param('~encoder', 'vits')
        self.valid_iters = rospy.get_param('~valid_iters', 2)
        self.baseline = rospy.get_param('~baseline', 0.12)
        self.focal_length = rospy.get_param('~focal_length', 500.0)
        self.max_disp = rospy.get_param('~max_disp', 416)
        self.display = rospy.get_param('~display', True)
        
        # 架构参数
        self.hidden_dims = rospy.get_param('~hidden_dims', [32, 64, 96])
        self.corr_implementation = rospy.get_param('~corr_implementation', 'reg')
        self.corr_levels = rospy.get_param('~corr_levels', 2)
        self.corr_radius = rospy.get_param('~corr_radius', [2, 2, 4])
        self.n_downsample = rospy.get_param('~n_downsample', 2)
        self.n_gru_layers = rospy.get_param('~n_gru_layers', 3)
        self.mixed_precision = rospy.get_param('~mixed_precision', False)
        
        # 点云和地图参数
        self.pc_config = rospy.get_param('~point_cloud', {
            'filter': {
                'max_depth': 100.0,
                'height_filter_mode': 'relative',
                'height_ratio_range': [0.3, 1.0]
            },
            'voxel_size': 1.0
        })
        
        # 相机内参（用于点云生成）
        self.camera_intrinsics = rospy.get_param('~camera_intrinsics', {
            'fx': 365.485,
            'fy': 365.485,
            'cx': 320.0,
            'cy': 180.0
        })
        
        self.map_config = rospy.get_param('~map', {
            'sliding_window': {'enabled': True, 'size': 3},
            'resolution': 0.8,
            'height_range': [0.3, 0.7],
            'use_ratio': True,
            'occupied_thresh': 3
        })
        
        self.ros_config = rospy.get_param('~ros', {
            'topics': {
                'output_point_cloud': '/o3d_pointCloud',
                'output_map': '/projected_map'
            },
            'publish_rate': {
                'point_cloud': 1,
                'map': 1
            }
        })
        
        self.gravity_config = rospy.get_param('~gravity_alignment', {
            'enabled': True,
            'save_interval': 1.0,
            'check_interval': 0.5
        })
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Monster 双目深度估计节点初始化（带建图功能）")
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"设备: {DEVICE}")
        rospy.loginfo(f"模型路径: {self.model_path}")
        rospy.loginfo(f"编码器: {self.encoder}")
        rospy.loginfo(f"迭代次数: {self.valid_iters}")
        rospy.loginfo(f"基线: {self.baseline} m")
        rospy.loginfo(f"焦距: {self.focal_length} px")
        rospy.loginfo(f"显示: {self.display}")
        rospy.loginfo("=" * 50)
        rospy.loginfo("相机内参（点云生成）:")
        rospy.loginfo(f"  fx: {self.camera_intrinsics['fx']:.2f} px")
        rospy.loginfo(f"  fy: {self.camera_intrinsics['fy']:.2f} px")
        rospy.loginfo(f"  cx: {self.camera_intrinsics['cx']:.2f} px")
        rospy.loginfo(f"  cy: {self.camera_intrinsics['cy']:.2f} px")
        rospy.loginfo("=" * 50)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 加载模型
        self.load_model()
        
        # 初始化点云生成器和地图构建器
        self.point_cloud_generator = StereoPointCloudGenerator()
        self.map_builder = OccupancyGridBuilder(self.map_config)
        
        # ROS 发布器
        self.pcl_pub = rospy.Publisher(
            self.ros_config['topics']['output_point_cloud'],
            PointCloud2,
            queue_size=1
        )
        self.map_pub = rospy.Publisher(
            self.ros_config['topics']['output_map'],
            OccupancyGrid,
            queue_size=1,
            latch=True
        )
        
        rospy.loginfo(f"点云话题: {self.ros_config['topics']['output_point_cloud']}")
        rospy.loginfo(f"地图话题: {self.ros_config['topics']['output_map']}")
        
        # 订阅话题
        self.sub = rospy.Subscriber(
            '/orb_slam3/image_pose_stereo',
            ImagePoseStereo,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 统计信息
        self.frame_count = 0
        self.fps_list = []
        self.last_time = rospy.Time.now()
        
        # 重力估计相关
        self.is_first_frame = True
        self.last_gravity_estimate_time = 0
        self.R_align = None
        self.last_R_align_load_time = 0
        
        rospy.loginfo("节点初始化完成，等待图像数据...")
        rospy.loginfo("订阅话题: /orb_slam3/image_pose_stereo")
        
    def load_model(self):
        """加载 Monster 模型"""
        rospy.loginfo("正在加载 Monster 模型...")
        
        class Args:
            def __init__(self, node):
                self.encoder = node.encoder
                self.hidden_dims = node.hidden_dims
                self.corr_implementation = node.corr_implementation
                self.shared_backbone = False
                self.corr_levels = node.corr_levels
                self.corr_radius = node.corr_radius
                self.n_downsample = node.n_downsample
                self.slow_fast_gru = False
                self.n_gru_layers = node.n_gru_layers
                self.max_disp = node.max_disp
                self.mixed_precision = node.mixed_precision
                self.restore_ckpt = node.model_path
        
        args = Args(self)
        
        try:
            self.model = torch.nn.DataParallel(Monster(args), device_ids=[0])
            
            if not Path(self.model_path).exists():
                rospy.logfatal(f"模型文件不存在: {self.model_path}")
                rospy.signal_shutdown("模型文件不存在")
                return
            
            checkpoint = torch.load(self.model_path)
            ckpt = dict()
            if 'state_dict' in checkpoint.keys():
                checkpoint = checkpoint['state_dict']
            for key in checkpoint:
                ckpt['module.' + key] = checkpoint[key]
            
            self.model.load_state_dict(ckpt, strict=True)
            self.model = self.model.module
            self.model.to(DEVICE)
            self.model.eval()
            
            rospy.loginfo("✓ Monster 模型加载成功")
            
        except Exception as e:
            rospy.logfatal(f"模型加载失败: {e}")
            import traceback
            traceback.print_exc()
            rospy.signal_shutdown("模型加载失败")
    
    def image_callback(self, msg):
        """处理接收到的双目图像消息"""
        try:
            start_time = rospy.Time.now()
            
            # 检查跟踪状态
            if not msg.tracking_success:
                rospy.logwarn_throttle(1.0, "SLAM 跟踪失败，跳过当前帧")
                return
            
            # 转换图像
            left_img = self.bridge.imgmsg_to_cv2(msg.image_left, desired_encoding='rgb8')
            right_img = self.bridge.imgmsg_to_cv2(msg.image_right, desired_encoding='rgb8')
            
            # 提取位姿
            translation = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, 
                         msg.pose.orientation.z, msg.pose.orientation.w]
            
            # 构建位姿 (Tcw -> Twc)
            T_pp = pp.SE3(torch.tensor(translation + quaternion))
            T_pp_inv = pp.Inv(T_pp)  # Twc
            
            # 转换为 tensor
            image1 = numpy_to_tensor(left_img)
            image2 = numpy_to_tensor(right_img)
            
            # Padding
            padder = InputPadder(image1.shape, divis_by=32)
            image1, image2 = padder.pad(image1, image2)
            
            # 深度估计
            with torch.no_grad():
                inference_start = rospy.Time.now()
                disp = self.model(image1, image2, iters=self.valid_iters, test_mode=True)
                inference_time = (rospy.Time.now() - inference_start).to_sec()
            
            # Unpad
            disp = padder.unpad(disp)
            disp_np = disp.cpu().numpy().squeeze()
            
            # 将视差转换为深度
            depth = disparity_to_depth(disp_np, self.baseline, self.focal_length)
            
            # 生成点云
            h, w = depth.shape
            camera_params = {
                'fx': self.camera_intrinsics['fx'],
                'fy': self.camera_intrinsics['fy'],
                'cx': self.camera_intrinsics['cx'],
                'cy': self.camera_intrinsics['cy']
            }
            
            points, colors = self.point_cloud_generator.generate(
                depth, left_img, camera_params, T_pp_inv,
                max_depth=self.pc_config['filter']['max_depth']
            )
            points_after_generate = len(points)
            
            # 过滤点云
            if len(points) > 0:
                points, colors = self.point_cloud_generator.filter(
                    points, colors, self.pc_config['filter']
                )
            points_after_filter = len(points)
            
            # 下采样
            if len(points) > 0:
                points, colors = self.point_cloud_generator.downsample(
                    points, colors, self.pc_config['voxel_size']
                )
            points_after_downsample = len(points)
            
            # 更新地图
            if len(points) > 0:
                self.map_builder.update(points, colors)
            
            # 详细诊断日志（每10帧输出一次）
            if self.frame_count % 10 == 0:
                rospy.loginfo(f"🔍 点云处理统计:")
                rospy.loginfo(f"  深度图尺寸: {h}x{w}")
                rospy.loginfo(f"  深度范围: [{depth.min():.2f}, {depth.max():.2f}]m")
                rospy.loginfo(f"  生成后: {points_after_generate} 点")
                rospy.loginfo(f"  过滤后: {points_after_filter} 点 ({points_after_filter/max(points_after_generate,1)*100:.1f}%)")
                rospy.loginfo(f"  下采样后: {points_after_downsample} 点 ({points_after_downsample/max(points_after_filter,1)*100:.1f}%)")
                rospy.loginfo(f"  体素大小: {self.pc_config['voxel_size']}m")
                rospy.loginfo(f"  最大深度: {self.pc_config['filter']['max_depth']}m")
            
            # 发布点云
            if self.frame_count % self.ros_config['publish_rate']['point_cloud'] == 0:
                self.publish_point_cloud()
            
            # 发布地图
            if self.frame_count % self.ros_config['publish_rate']['map'] == 0:
                self.publish_map()
            
            # 重力估计相关
            if self.gravity_config['enabled']:
                self.handle_gravity_estimation(left_img, T_pp, msg.header.stamp.to_sec())
            
            # 显示
            if self.display:
                self.visualize(left_img, disp_np, depth, inference_time)
            
            # 统计
            self.frame_count += 1
            total_time = (rospy.Time.now() - start_time).to_sec()
            fps = 1.0 / total_time if total_time > 0 else 0
            self.fps_list.append(fps)
            if len(self.fps_list) > 30:
                self.fps_list.pop(0)
            
            # 定期输出统计信息
            if self.frame_count % 30 == 0:
                avg_fps = np.mean(self.fps_list)
                total_points, _ = self.map_builder.get_point_cloud()
                rospy.loginfo(f"📊 帧 {self.frame_count}: FPS={avg_fps:.1f}, "
                            f"推理={inference_time*1000:.1f}ms")
                rospy.loginfo(f"  当前帧点数: {points_after_downsample}")
                rospy.loginfo(f"  地图总点数: {len(total_points)}")
                rospy.loginfo(f"  滑动窗口: {self.map_builder.get_frame_count()}/{self.map_config['sliding_window']['size']} 帧")
        
        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")
            import traceback
            traceback.print_exc()
    
    def publish_point_cloud(self):
        """发布点云"""
        try:
            points, colors = self.map_builder.get_point_cloud()
            
            if len(points) == 0:
                return
            
            # 应用重力对齐
            if self.R_align is not None:
                points = points @ self.R_align.T
            
            # 创建 PointCloud2 消息
            if colors is not None:
                points_with_color = np.zeros(len(points), dtype=[
                    ('x', np.float32), ('y', np.float32), ('z', np.float32),
                    ('r', np.uint8), ('g', np.uint8), ('b', np.uint8)
                ])
                points_with_color['x'] = points[:, 0]
                points_with_color['y'] = points[:, 1]
                points_with_color['z'] = points[:, 2]
                colors_uint8 = (colors * 255).astype(np.uint8)
                points_with_color['r'] = colors_uint8[:, 0]
                points_with_color['g'] = colors_uint8[:, 1]
                points_with_color['b'] = colors_uint8[:, 2]
                
                fields = [
                    PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('r', 12, PointField.UINT8, 1),
                    PointField('g', 13, PointField.UINT8, 1),
                    PointField('b', 14, PointField.UINT8, 1)
                ]
            else:
                points_with_color = points
                fields = [
                    PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1)
                ]
            
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            
            pcl_msg = pc2.create_cloud(header, fields, points_with_color)
            self.pcl_pub.publish(pcl_msg)
            
        except Exception as e:
            rospy.logwarn_throttle(5, f"发布点云失败: {e}")
    
    def publish_map(self):
        """发布 2D 占用栅格地图"""
        try:
            grid_map = self.map_builder.get_occupancy_grid(
                resolution=self.map_config['resolution'],
                height_range=tuple(self.map_config['height_range']),
                occupied_thresh=self.map_config['occupied_thresh'],
                use_ratio=self.map_config['use_ratio']
            )
            
            if grid_map is None:
                return
            
            # 构造 OccupancyGrid 消息
            occ_msg = OccupancyGrid()
            occ_msg.header.stamp = rospy.Time.now()
            occ_msg.header.frame_id = "map"
            
            occ_msg.info.resolution = grid_map['resolution']
            occ_msg.info.width = grid_map['width']
            occ_msg.info.height = grid_map['height']
            occ_msg.info.origin.position.x = grid_map['origin'][0]
            occ_msg.info.origin.position.y = grid_map['origin'][1]
            occ_msg.info.origin.position.z = 0.0
            occ_msg.info.origin.orientation.w = 1.0
            
            occ_msg.data = grid_map['data'].flatten().tolist()
            
            self.map_pub.publish(occ_msg)
            
        except Exception as e:
            rospy.logwarn_throttle(5, f"发布地图失败: {e}")
    
    def handle_gravity_estimation(self, image, T_pp, timestamp):
        """处理重力估计相关功能"""
        current_time = time.time()
        
        # 初始化 GE_information 目录
        if self.is_first_frame:
            self.is_first_frame = False
            ge_info_dir = f"{script_dir}/GE_information"
            if not os.path.exists(ge_info_dir):
                os.makedirs(ge_info_dir)
                rospy.loginfo(f"✓ 创建重力估计目录: {ge_info_dir}")
        
        # 定期保存图像和位姿
        if current_time - self.last_gravity_estimate_time >= self.gravity_config['save_interval']:
            self.save_image_and_pose(image, T_pp, timestamp, self.frame_count)
            self.last_gravity_estimate_time = current_time
        
        # 定期检查并加载 R_align
        if current_time - self.last_R_align_load_time >= self.gravity_config['check_interval']:
            self.load_R_align()
            self.last_R_align_load_time = current_time
    
    def save_image_and_pose(self, image, T_pp, timestamp, frame_id):
        """保存图像和位姿用于重力估计"""
        try:
            ge_info_dir = f"{script_dir}/GE_information"
            
            # 保存图像
            image_path = os.path.join(ge_info_dir, "latest_img.png")
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(image_path, image_bgr)
            
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
            
        except Exception as e:
            rospy.logwarn_throttle(10, f"保存图像和位姿失败: {e}")
    
    def load_R_align(self):
        """加载重力对齐矩阵"""
        yaml_path = f"{script_dir}/GE_information/rotation_matrices.yaml"
        
        if not os.path.exists(yaml_path):
            return
        
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            R_align_new = np.array(data['R_align'])
            
            if self.R_align is None or not np.allclose(R_align_new, self.R_align):
                self.R_align = R_align_new
                g_aligned = data.get('g_aligned', [0, 0, 0])
                rospy.loginfo(f"✓ 已加载重力对齐矩阵")
                rospy.loginfo(f"  对齐后重力: [{g_aligned[0]:.4f}, {g_aligned[1]:.4f}, {g_aligned[2]:.4f}]")
                rospy.loginfo(f"  R_align:")
                for i in range(3):
                    rospy.loginfo(f"    [{self.R_align[i,0]:7.4f}, {self.R_align[i,1]:7.4f}, {self.R_align[i,2]:7.4f}]")
                
        except Exception as e:
            rospy.logwarn_throttle(10, f"加载重力对齐矩阵失败: {e}")
    
    def visualize(self, left_img, disp_np, depth, inference_time):
        """可视化深度图"""
        try:
            # 转换视差为彩色图
            disp_color = cv2.applyColorMap(
                np.clip(disp_np * 2.0, 0, 255).astype(np.uint8),
                cv2.COLORMAP_PLASMA
            )
            
            # 转换深度为彩色图
            depth_normalized = np.clip(depth / self.baseline / 10.0, 0, 1)
            depth_color = cv2.applyColorMap(
                (depth_normalized * 255).astype(np.uint8),
                cv2.COLORMAP_JET
            )
            
            # 转换左图为 BGR
            left_img_bgr = cv2.cvtColor(left_img, cv2.COLOR_RGB2BGR)
            
            # 拼接显示
            h, w = left_img.shape[:2]
            disp_color_resized = cv2.resize(disp_color, (w, h))
            depth_color_resized = cv2.resize(depth_color, (w, h))
            
            display_img = np.hstack((left_img_bgr, disp_color_resized, depth_color_resized))
            
            # 添加文字信息
            avg_fps = np.mean(self.fps_list) if self.fps_list else 0
            cv2.putText(display_img, f"Frame: {self.frame_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_img, f"FPS: {avg_fps:.1f}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_img, f"Inference: {inference_time*1000:.1f}ms", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow("Monster Stereo Depth Estimation", display_img)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"可视化错误: {e}")
    
    def run(self):
        """运行节点"""
        rospy.spin()
        
        # 清理
        if self.display:
            cv2.destroyAllWindows()
        
        # 保存点云
        try:
            output_path = os.path.join(script_dir, "pointCloud/monster_map.ply")
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            self.map_builder.save(output_path)
            rospy.loginfo(f"✓ 点云已保存到: {output_path}")
        except Exception as e:
            rospy.logwarn(f"保存点云失败: {e}")
        
        # 输出统计
        if self.frame_count > 0:
            rospy.loginfo("=" * 50)
            rospy.loginfo(f"总共处理了 {self.frame_count} 帧")
            if self.fps_list:
                rospy.loginfo(f"平均 FPS: {np.mean(self.fps_list):.2f}")
            rospy.loginfo("=" * 50)


def main():
    try:
        node = MonsterStereoNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logfatal(f"节点运行错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
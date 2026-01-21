import argparse
import shutil
import cv2
import matplotlib
import numpy as np
import os
import sys
import torch
import open3d as o3d
import pypose as pp
import sensor_msgs.point_cloud2 as pc2
import rospy
import std_msgs.msg
import json
import yaml

# 添加当前脚本目录到 Python 路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from depth_anything_v2.dpt import DepthAnythingV2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from depth_maping.msg import ImagePose
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from sklearn.linear_model import RANSACRegressor # 用于平面拟合

# ROS OccupancyGrid
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

'''
indoor  outdoor
'''

class Img2DepthMaping:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='Depth Anything V2 Metric Depth Estimation')
        
        # 🔥 优化：进一步降低深度估计分辨率（518 -> 392 -> 256）
        self.input_size = 256  # choices=[256, 384, 512, 640]
        self.outdir = './vis_depth'
        self.encoder = 'vitb' # choices=['vits', 'vitb', 'vitl', 'vitg']
        self.load_from = "/home/sunteng/Downloads/depth_anything_v2_metric_hypersim_vitb.pth"
        self.max_depth = 70.0
        
        self.SAVE = False
        self.save_numpy = False
        self.pred_only = True
        self.grayscale = True

        # 是否为第一帧
        self.is_first_frame = True
        # 当前文件绝对路径
        self.current_file_path = os.path.abspath(__file__)
        # 当前文件目录
        self.current_dir = os.path.dirname(self.current_file_path)
        
        # 重力估计相关
        self.last_gravity_estimate_time = 0
        self.gravity_estimate_interval = 1.0  # 每秒保存一次
        self.R_align = None  # 重力对齐矩阵
        self.last_R_align_load_time = 0
        self.R_align_check_interval = 0.5  # 每0.5秒检查一次R_align更新

        # *************************************************************** #
        self.fx = 138.54264656
        self.fy = 138.60053687
        self.cx = 331.89824222
        self.cy = 239.70296783

        self.k1, self.k2, self.k3, self.k4 = -0.05094921, -0.00983458, 0.00521841, -0.00128268

        self.K = np.array([[self.fx, 0.0, self.cx],
                    [0.0, self.fy, self.cy],
                    [0.0, 0.0, 1.0]], dtype=np.float64)

        self.D = np.array([self.k1, self.k2, self.k3, self.k4], dtype=np.float64)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K, (640, 480), cv2.CV_16SC2)
        # ******************************************************************* #

        self.DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
        
        self.model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }
        
        self.depth_anything = DepthAnythingV2(**{**self.model_configs[self.encoder], 'max_depth': self.max_depth})
        self.depth_anything.load_state_dict(torch.load(self.load_from, map_location='cpu'))
        self.depth_anything = self.depth_anything.to(self.DEVICE).eval()
        
        self.cmap = matplotlib.colormaps.get_cmap('Spectral')

        # 平移向量的尺度倍率
        self.translation_size = 18

        # 裁剪深度图的最大深度值（单位：米）
        self.cut_depth = 35.0  

        self.filenames = []
        self.pose_files = []

        # *********************** ros Sub & Pub ************************* #
        self.bridge = CvBridge()
        # 增加 queue_size 避免消息丢失和延迟累积
        self.image_pose = rospy.Subscriber("/orb_slam3/image_pose",ImagePose,self.depth_solver,queue_size=100)
        
        self.rate = rospy.Rate(10)
        self.pcl_pub = rospy.Publisher('/o3d_pointCloud',PointCloud2,queue_size=10)
        
        # 时间戳诊断
        self.last_msg_time = None
        self.processing_times = []
        
        # 发布深度图
        # self.depth_pub = rospy.Publisher('/depth_anything/depth_image', Image, queue_size=10)
        # rospy.loginfo("深度图发布器已初始化: /depth_anything/depth_image")

        # *********************** needed pose ******************* #
        self.now_pose = pp.SE3(torch.tensor([0., 0., 0., 0., 0., 0., 1.])) # 初始化T

        # ************************ 滑动窗口点云功能 ********************** #
        # 是否启用滑动窗口模式（只显示最近N帧）
        self.enable_sliding_window = rospy.get_param('~enable_sliding_window', True)
        # 滑动窗口大小（保留最近N帧点云）
        self.sliding_window_size = rospy.get_param('~sliding_window_size', 3)
        # 存储每一帧的点云
        self.point_cloud_frames = []  # 列表，存储每帧的 PointCloud 对象
        
        rospy.loginfo(f"滑动窗口模式: {'启用' if self.enable_sliding_window else '禁用'}")
        if self.enable_sliding_window:
            rospy.loginfo(f"滑动窗口大小: {self.sliding_window_size} 帧")

        # ************************ 高度过滤功能 ********************** #
        # 是否启用高度过滤
        self.enable_height_filter = rospy.get_param('~enable_height_filter', True)
        # 高度过滤模式：'relative' 或 'absolute'
        self.height_filter_mode = rospy.get_param('~height_filter_mode', 'relative')
        
        if self.height_filter_mode == 'relative':
            # 相对模式：使用相机高度的百分比（适用于单目SLAM，尺度不确定）
            self.height_ratio_min = rospy.get_param('~height_ratio_min', 0.3)  # 保留相机高度30%以上的点
            self.height_ratio_max = rospy.get_param('~height_ratio_max', 2.0)  # 保留相机高度200%以下的点
            rospy.loginfo(f"高度过滤: {'启用' if self.enable_height_filter else '禁用'} (相对模式)")
            if self.enable_height_filter:
                rospy.loginfo(f"高度比例范围: [{self.height_ratio_min:.1f}x ~ {self.height_ratio_max:.1f}x] 相机高度")
        else:
            # 绝对模式：使用固定米数（适用于已知尺度的场景）
            self.height_min = rospy.get_param('~height_min', -2.0)
            self.height_max = rospy.get_param('~height_max', 3.0)
            rospy.loginfo(f"高度过滤: {'启用' if self.enable_height_filter else '禁用'} (绝对模式)")
            if self.enable_height_filter:
                rospy.loginfo(f"高度范围: [{self.height_min:.2f}m, {self.height_max:.2f}m] (Y轴)")

        # **************** OccupancyGrid 地图发布器 **************** #
        self.frame_counter = 0
        self.map_update_interval = 1           # 每 5 帧更新一次地图（可调，5~20 都合理）
        self.map_pub = rospy.Publisher('/projected_map', OccupancyGrid, queue_size=1, latch=True)
        
        # 2D地图高度过滤参数（百分比模式）
        self.map_height_ratio_min = rospy.get_param('~map_height_ratio_min', 0.3)  # 2D地图保留最低30%以上
        self.map_height_ratio_max = rospy.get_param('~map_height_ratio_max', 0.7)  # 2D地图保留最高70%以下
        
        rospy.loginfo("2D OccupancyGrid 地图发布器已初始化: /projected_map")
        rospy.loginfo(f"2D地图高度过滤: [{self.map_height_ratio_min:.1f} ~ {self.map_height_ratio_max:.1f}] (百分比)")


        # ************************ 点云查看器 ********************** #
        # 🔥 优化：默认禁用可视化以提升性能
        self.enable_visualization = rospy.get_param('~enable_visualization', False)
        rospy.logwarn("⚠️  为提升性能，Open3D 可视化已默认禁用。如需启用，请设置 enable_visualization:=true")
        
        if self.enable_visualization:
            try:
                self.vis = o3d.visualization.VisualizerWithKeyCallback()
                self.vis.create_window(window_name="point cloud", width=1280, height=960, visible=True)
                self.all_point_cloud = o3d.geometry.PointCloud()
                self.vis.add_geometry(self.all_point_cloud)
                self.reset_view = True # 是否第一帧
                rospy.loginfo("Open3D 可视化窗口已启用")
            except Exception as e:
                rospy.logwarn(f"无法创建 Open3D 可视化窗口: {e}")
                rospy.logwarn("将禁用可视化功能，仅发布点云话题")
                self.enable_visualization = False
                self.all_point_cloud = o3d.geometry.PointCloud()
                self.reset_view = True
        else:
            rospy.loginfo("Open3D 可视化已禁用（通过参数设置）")
            self.all_point_cloud = o3d.geometry.PointCloud()
            self.reset_view = True

        self.points_with_color = np.array([], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        
        # 添加关闭标志
        self.is_shutdown = False

    def depth_solver(self,data,is_depth_process=False):
        # 检查节点是否正在关闭
        if self.is_shutdown or rospy.is_shutdown():
            return

        # ========== 时间戳诊断 ========== #
        import time
        callback_start_time = time.time()
        
        # 计算消息延迟
        msg_timestamp = data.header.stamp.to_sec()
        current_time = rospy.Time.now().to_sec()
        msg_delay = current_time - msg_timestamp
        
        # 跳过延迟超过 0.1 秒的旧消息
        if msg_delay > 0.1:
            rospy.logwarn_throttle(1, f"⏭️  跳过旧消息（延迟 {msg_delay:.3f}s）")
            return
        
        # 计算消息间隔
        if self.last_msg_time is not None:
            msg_interval = msg_timestamp - self.last_msg_time
            rospy.loginfo_throttle(2, f"📊 消息延迟: {msg_delay:.3f}s | 消息间隔: {msg_interval:.3f}s ({1/msg_interval:.1f} Hz)")
        self.last_msg_time = msg_timestamp
        # ================================ #

        self.frame_counter += 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data.image, "bgr8")
        except Exception as e:
            rospy.logwarn(f"图像转换失败: {e}")
            return

        translation = [data.pose.position.x,
                       data.pose.position.y,
                       data.pose.position.z]
        
        quaternion = [data.pose.orientation.x,
                      data.pose.orientation.y,
                      data.pose.orientation.z,
                      data.pose.orientation.w]
        
        
        '''
        ORB-SLAM3 的矩阵命名是「目标坐标系→源坐标系」（后缀 cw = Camera ← World）：
        Tcw：World → Camera（世界到相机）；
        Twc：Camera → World（相机到世界）；
        '''
        T_pp = pp.SE3(torch.tensor(translation + quaternion)) # Tcw
        T_pp_inv = pp.Inv(T_pp) # Twc

        original_translation = T_pp_inv.translation()
        original_rotation = T_pp_inv.rotation() 

        new_translation = original_translation * self.translation_size

        T_pp_inv_new = pp.SE3(torch.cat([new_translation, original_rotation]))

        print("SE3: ",T_pp)
        print("SE3_inv: ",T_pp_inv)

        raw_image = cv2.cvtColor(cv_image.copy(), cv2.COLOR_BGR2RGB)
        undistorted_frame = cv2.remap(raw_image, self.map1, self.map2, cv2.INTER_LINEAR)
        cv2.imshow("Undistorted Frame", undistorted_frame)
        cv2.waitKey(1)
        
        # 初始化 GE_information 目录
        if self.is_first_frame:
            self.is_first_frame = False
            ge_info_dir = f"{self.current_dir}/GE_information"
            if not os.path.exists(ge_info_dir):
                os.makedirs(ge_info_dir)
                rospy.loginfo(f"✓ 创建重力估计目录: {ge_info_dir}")
            else:
                rospy.loginfo(f"✓ 清空重力估计目录: {ge_info_dir}")
                self.clear_folder(ge_info_dir)
        
        # 定期保存图像和位姿数据用于重力估计
        current_time = time.time()
        if current_time - self.last_gravity_estimate_time >= self.gravity_estimate_interval:
            self.save_image_and_pose(undistorted_frame, T_pp, data.header.stamp.to_sec(), self.frame_counter)
            self.last_gravity_estimate_time = current_time
        
        # 定期检查并加载最新的 R_align
        if current_time - self.last_R_align_load_time >= self.R_align_check_interval:
            self.load_R_align()
            self.last_R_align_load_time = current_time
        
        # 测量深度估计时间
        depth_start = time.time()
        depth = self.depth_anything.infer_image(undistorted_frame, self.input_size)
        depth_time = time.time() - depth_start
        depth_npy = depth.copy()
        
        rospy.loginfo_throttle(2, f"⏱️  深度估计耗时: {depth_time:.3f}s ({1/depth_time:.1f} FPS)")

        # 深度图预处理
        if is_depth_process:
            depth_npy = self.preprocess_depth_map(depth_npy, kernel_size=5, threshold=0.07)
        
        # 发布深度图（使用与位姿相同的时间戳）
        # if not self.is_shutdown and not rospy.is_shutdown():
        #     try:
        #         # 将深度图转换为 ROS Image 消息（32位浮点数格式）
        #         depth_msg = self.bridge.cv2_to_imgmsg(depth_npy, encoding="32FC1")
        #         depth_msg.header.stamp = data.header.stamp  # 使用相同的时间戳
        #         depth_msg.header.frame_id = "camera"
        #         self.depth_pub.publish(depth_msg)
        #     except Exception as e:
        #         rospy.logwarn_throttle(5, f"发布深度图失败: {e}")

        if True : # self.is_needed_pose(T_pp_inv_new,dis_range=2.,yaw_range=15.,pitch_range=15.)
            point_cloud = self.npy_depth_to_point_cloud_cut(depth_npy,
                                                            self.fx,
                                                            self.fy,
                                                            self.cx,
                                                            self.cy,
                                                            T_pp_inv_new,
                                                            depth_scale=1.0,
                                                            rgb_image=undistorted_frame,
                                                            img_cup_size=128,
                                                            voxel_size=1.0
                                                            )

            # 🔥 高度过滤（在世界坐标系中过滤 Y 轴）
            if self.enable_height_filter and len(point_cloud.points) > 0:
                points = np.asarray(point_cloud.points)
                colors = np.asarray(point_cloud.colors) if point_cloud.has_colors() else None
                
                if self.height_filter_mode == 'relative':
                    # 相对模式：基于点云自身高度范围的百分位数过滤
                    # 计算点云的高度范围
                    y_min = points[:, 1].min()
                    y_max = points[:, 1].max()
                    y_range = y_max - y_min
                    
                    if y_range < 0.01:  # 避免除零
                        rospy.logwarn_throttle(10, "点云高度范围太小，跳过过滤")
                        height_mask = np.ones(len(points), dtype=bool)
                    else:
                        # ratio_min=0.3 表示过滤掉最低30%的高度范围
                        # ratio_max=0.7 表示过滤掉最高30%的高度范围
                        # 例如：y_range=10m, ratio_min=0.3, ratio_max=0.7
                        #   保留范围：y_min+3m 到 y_min+7m
                        height_min_abs = y_min + y_range * self.height_ratio_min
                        height_max_abs = y_min + y_range * self.height_ratio_max
                        
                        height_mask = (points[:, 1] >= height_min_abs) & (points[:, 1] <= height_max_abs)
                        
                        rospy.loginfo_throttle(10, f"📏 点云高度: [{y_min:.2f}, {y_max:.2f}] 范围:{y_range:.2f} | 过滤: [{height_min_abs:.2f}, {height_max_abs:.2f}]")
                else:
                    # 绝对模式：使用固定高度范围
                    height_mask = (points[:, 1] >= self.height_min) & (points[:, 1] <= self.height_max)
                
                filtered_points = points[height_mask]
                
                # 创建过滤后的点云
                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
                
                if colors is not None:
                    filtered_colors = colors[height_mask]
                    point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)
                
                # 统计过滤信息
                filtered_count = len(points) - len(filtered_points)
                if filtered_count > 0:
                    rospy.loginfo_throttle(10, f"🔍 高度过滤: 移除 {filtered_count}/{len(points)} 点 ({filtered_count/len(points)*100:.1f}%)")

            # 根据滑动窗口模式更新点云
            if self.enable_sliding_window:
                # 滑动窗口模式：只保留最近N帧
                self.point_cloud_frames.append(point_cloud)
                
                # 如果超过窗口大小，移除最旧的帧
                if len(self.point_cloud_frames) > self.sliding_window_size:
                    self.point_cloud_frames.pop(0)
                
                # 合并当前窗口内的所有点云
                self.all_point_cloud = o3d.geometry.PointCloud()
                for frame_cloud in self.point_cloud_frames:
                    self.all_point_cloud += frame_cloud
                
                rospy.loginfo_throttle(5, f"滑动窗口: 当前显示 {len(self.point_cloud_frames)}/{self.sliding_window_size} 帧")
            else:
                # 累积模式：持续累加所有点云
                self.all_point_cloud += point_cloud
        
        # 点云发布频率
        if not self.is_shutdown and not rospy.is_shutdown() and self.frame_counter % 1 == 0:
            try:
                all_pointCloud_pc2 = self.o3d_to_ros_pointCloud2(self.all_point_cloud,"map")
                self.pcl_pub.publish(all_pointCloud_pc2)
            except rospy.ROSException as e:
                rospy.logwarn_throttle(5, f"发布点云失败（节点可能正在关闭）: {e}")
            except Exception as e:
                rospy.logwarn(f"点云转换失败: {e}")

        # 地图更新频率
        if self.frame_counter % 1 == 0:
            occ_msg = self.project_to_2d_occupancy(
                resolution=0.8,                              # 可调：0.02~0.1
                height_ratio_min=self.map_height_ratio_min,  # 使用百分比
                height_ratio_max=self.map_height_ratio_max,  # 使用百分比
                occupied_thresh=3                           # 点数阈值，可根据密度调 3~10
            )
            if occ_msg is not None:
                self.map_pub.publish(occ_msg)
                rospy.loginfo_throttle(5, f"已发布2D Occ地图 ({occ_msg.info.width}x{occ_msg.info.height}, res={occ_msg.info.resolution}m)")
        
        # 渲染频率
        if self.enable_visualization and not self.is_shutdown and self.frame_counter % 3 == 0:
            try:
                self.vis.poll_events()
                self.vis.update_renderer()
            except Exception as e:
                rospy.logwarn_throttle(10, f"渲染失败: {e}")
        
        # 记录总处理时间
        callback_total_time = time.time() - callback_start_time
        self.processing_times.append(callback_total_time)
        if len(self.processing_times) > 100:
            self.processing_times.pop(0)
        
        avg_processing_time = np.mean(self.processing_times)
        rospy.loginfo_throttle(5, f"🔧 回调总耗时: {callback_total_time:.3f}s | 平均: {avg_processing_time:.3f}s | 理论最大频率: {1/avg_processing_time:.1f} Hz")

    def preprocess_depth_map(self,depth_map, kernel_size=3, threshold=0.05):
        """
        深度图预处理，减少边缘噪声
        """
        
        # 1. 中值滤波（去除椒盐噪声）
        depth_filtered = cv2.medianBlur(depth_map.astype(np.float32), kernel_size)
        
        # 2. 双边滤波（保留边缘）
        depth_filtered = cv2.bilateralFilter(depth_filtered, d=5, sigmaColor=0.1, sigmaSpace=5)
        
        # 3. 形态学操作（填充小孔洞）
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        depth_filtered = cv2.morphologyEx(depth_filtered, cv2.MORPH_CLOSE, kernel)
        
        # 4. 深度一致性检查
        depth_gradient = np.abs(cv2.Sobel(depth_filtered, cv2.CV_64F, 1, 1, ksize=3))
        
        # 5. 梯度阈值（过滤边缘高梯度点）
        edge_mask = depth_gradient < threshold
        depth_filtered[~edge_mask] = 0  # 将高梯度点设为无效
        
        return depth_filtered

    def npy_depth_to_point_cloud(self,depth_map_npy, fx, fy, cx, cy, T_pp, depth_scale=1.0, rgb_image=None):
        depth_map = depth_map_npy
        valid_mask = (depth_map > 0) & (depth_map < 50) & np.isfinite(depth_map)

        hight,width = depth_map.shape
        u,v = np.meshgrid(np.arange(width), np.arange(hight))

        Z = depth_map[valid_mask] / depth_scale
        X = (u[valid_mask]-cx)*Z/fx
        Y = (v[valid_mask]-cy)*Z/fy

        points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)

        # 将点云转换为PyTorch张量
        points_cam_tensor = torch.tensor(points, dtype=torch.float32)

        # 对齐计算设备 对后期gpu优化have帮助
        device = T_pp.device
        points_cam_tensor = points_cam_tensor.to(device)
        
        # 使用*运算符进行变换（pypose会自动处理齐次坐标）
        points_world_tensor = T_pp * points_cam_tensor
        
        # 转换回NumPy
        points_world = points_world_tensor.detach().cpu().numpy()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_world)

        if rgb_image is not None:
            color_img = rgb_image
            color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
            if color_img.shape[:2]!= (hight, width):
                color_img = cv2.resize(color_img, (width, hight))
            colors = color_img[valid_mask].reshape(-1,3)/255.0
            pcd.colors = o3d.utility.Vector3dVector(colors)

        # 降采樣
        pcd = pcd.voxel_down_sample(voxel_size=0.5)
        return pcd
    
    def npy_depth_to_point_cloud_cut(self,depth_map_npy, fx, fy, cx, cy, T_pp, depth_scale=1.0, rgb_image=None, img_cup_size=8,voxel_size=2.0):
        depth_map = depth_map_npy
        hight,width = depth_map.shape
        
        start_x, end_x = int(width/img_cup_size), int(width*(img_cup_size-1)/img_cup_size)
        start_y, end_y = int(hight/img_cup_size), int(hight*(img_cup_size-1)/img_cup_size)

        u,v = np.meshgrid(np.arange(width/img_cup_size,width*(img_cup_size-1)/img_cup_size), np.arange(hight/img_cup_size,hight*(img_cup_size-1)/img_cup_size))

        depth_cropped = depth_map[start_y:end_y, start_x:end_x]
        valid_mask = (depth_cropped > 0) & (depth_cropped < self.cut_depth) & np.isfinite(depth_cropped)
        
        Z = depth_cropped[valid_mask] / depth_scale
        X = (u[valid_mask]-cx)*Z/fx
        Y = (v[valid_mask]-cy)*Z/fy

        points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)

        # 将点云转换为PyTorch张量
        points_cam_tensor = torch.tensor(points, dtype=torch.float32)

        # 对齐计算设备 对后期gpu优化have帮助
        device = T_pp.device
        points_cam_tensor = points_cam_tensor.to(device)
        
        # 使用*运算符进行变换（pypose会自动处理齐次坐标）
        points_world_tensor = T_pp * points_cam_tensor
        
        # 转换回NumPy
        points_world = points_world_tensor.detach().cpu().numpy()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_world)

        if rgb_image is not None:
            color_img = rgb_image
            color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
            if color_img.shape[:2]!= (hight, width):
                color_img = cv2.resize(color_img, (width, hight))
                
            # 裁剪
            color_cropped = color_img[start_y:end_y, start_x:end_x]
            colors = color_cropped[valid_mask].reshape(-1,3)/255.0
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 降采樣
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
       
        voxel_grid_loc = o3d.geometry.VoxelGrid.create_from_point_cloud(
            pcd,voxel_size=1
        )

        return pcd

    def is_needed_pose(self,T,dis_range=4.,yaw_range=20.,pitch_range=20.):
        now_t = self.now_pose.translation()
        now_r = self.now_pose.rotation() 
        t = T.translation()
        r = T.rotation()

        t_diff = t - now_t
        distance = torch.norm(t_diff,p=2) # L2范数

        now_r_inv = now_r.Inv()
        q_diff = r*now_r_inv # 计算相对旋转q  
        r = R.from_quat(q_diff)
        euler_angles_rad = r.as_euler('xyz')
        euler_angles_deg = np.degrees(euler_angles_rad)

        print(f"Roll (X): {euler_angles_deg[0]:.2f}°")
        print(f"Pitch (Y): {euler_angles_deg[1]:.2f}°")
        print(f"Yaw (Z): {euler_angles_deg[2]:.2f}°")

        print(f"\ndistance:{distance}\nq_diff:{q_diff}")

        if distance > dis_range or abs(euler_angles_deg[2])>yaw_range or abs(euler_angles_deg[1])>pitch_range:
            self.now_pose = T
            return True
        return False

    def o3d_to_ros_pointCloud2(self,o3d_points,image_id="odom"):
        points = np.asarray(o3d_points.points)
        R_cam_to_ros = np.array([
            [0,  0,  1],   # ROS X = Cam Z
            [-1, 0,  0],   # ROS Y = -Cam X
            [0, -1,  0]    # ROS Z = -Cam Y
        ])

        # points = points @ R_cam_to_ros.T

        # 应用重力对齐矩阵（如果存在）
        if self.R_align is not None:
            points = points @ self.R_align.T

        # 判断点云有没有颜色
        if o3d_points.has_colors():
            colors = np.asarray(o3d_points.colors)*255
            colors = colors.astype(np.uint8)

            # 初始化有色点云
            self.points_with_color = np.zeros(len(points),dtype=[
                ('x',np.float32),
                ('y',np.float32),
                ('z',np.float32),
                ('r',np.uint8),
                ('g',np.uint8),
                ('b',np.uint8)
            ])
            self.points_with_color['x'] = points[:,0]
            self.points_with_color['y'] = points[:,1]
            self.points_with_color['z'] = points[:,2]
            self.points_with_color['r'] = colors[:,0]
            self.points_with_color['g'] = colors[:,1]
            self.points_with_color['b'] = colors[:,2]

            # 定义pointCloud消息字段？
            fields = [
                PointField('x',0,PointField.FLOAT32,1),
                PointField('y',4,PointField.FLOAT32,1),
                PointField('z',8,PointField.FLOAT32,1),
                PointField('r',12,PointField.UINT8,1),
                PointField('g',13,PointField.UINT8,1),
                PointField('b',14,PointField.UINT8,1)
            ]
        
        else:
            fields = [
                PointField('x',0,PointField.FLOAT32,1),
                PointField('y',4,PointField.FLOAT32,1),
                PointField('z',8,PointField.FLOAT32,1)
            ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = image_id

        pcl_msg = pc2.create_cloud(header,fields,self.points_with_color)
        return pcl_msg
    
    def save_image_and_pose(self, image, T_pp, timestamp, frame_id):
        """
        保存图像和位姿数据用于重力估计（使用固定文件名，每次覆盖）
        
        Args:
            image: 去畸变后的图像
            T_pp: 位姿 (Tcw, Camera ← World)
            timestamp: 时间戳
            frame_id: 帧ID
        """
        try:
            ge_info_dir = f"{self.current_dir}/GE_information"
            
            # 使用固定文件名（每次覆盖旧文件）
            image_filename = "latest_img.png"
            image_path = os.path.join(ge_info_dir, image_filename)
            cv2.imwrite(image_path, image)
            
            # 提取位姿信息
            # T_pp 是 Tcw (World → Camera)
            # 我们需要 R_cw 和 t_cw
            R_cw = T_pp.rotation().matrix().cpu().numpy()  # 3x3
            t_cw = T_pp.translation().cpu().numpy()  # 3x1
            
            # 构建位姿数据
            pose_data = {
                'image_path': image_path,
                'timestamp': float(timestamp),
                'frame_id': int(frame_id),
                'R_cw': R_cw.tolist(),
                't_cw': t_cw.tolist()
            }
            
            # 使用固定文件名保存 JSON（每次覆盖）
            pose_filename = "latest_pose.json"
            pose_path = os.path.join(ge_info_dir, pose_filename)
            with open(pose_path, 'w') as f:
                json.dump(pose_data, f, indent=2)
            
            rospy.loginfo_throttle(5, f"💾 已更新图像和位姿: frame_{frame_id}")
            
        except Exception as e:
            rospy.logwarn(f"保存图像和位姿失败: {e}")
    
    def load_R_align(self):
        """
        加载最新的重力对齐矩阵
        """
        yaml_path = f"{self.current_dir}/GE_information/rotation_matrices.yaml"
        
        if not os.path.exists(yaml_path):
            return
        
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            R_align_new = np.array(data['R_align'])
            
            # 检查是否有更新
            if self.R_align is None or not np.allclose(R_align_new, self.R_align):
                self.R_align = R_align_new
                rospy.loginfo(f"✓ 已加载重力对齐矩阵 (timestamp: {data.get('timestamp', 'N/A')})")
                
                # 打印对齐信息
                g_aligned = data.get('g_aligned', [0, -1, 0])
                rospy.loginfo(f"  对齐后重力: [{g_aligned[0]:.4f}, {g_aligned[1]:.4f}, {g_aligned[2]:.4f}]")
                
        except Exception as e:
            rospy.logwarn_throttle(10, f"加载重力对齐矩阵失败: {e}")


    def project_to_2d_occupancy(self,
                                resolution=0.05,
                                height_ratio_min=0.3,
                                height_ratio_max=0.7,
                                occupied_thresh=5):
        """
        将当前 all_point_cloud 投影为 2D OccupancyGrid
        使用百分比方式过滤高度
        """
        if len(self.all_point_cloud.points) == 0:
            rospy.logwarn_once("点云为空，跳过生成2D地图")
            return None
        
        points = np.asarray(self.all_point_cloud.points)
        
        # 使用百分比方式过滤高度
        y_min = points[:, 1].min()
        y_max = points[:, 1].max()
        y_range = y_max - y_min
        
        if y_range < 0.01:
            rospy.logwarn_once("点云高度范围太小，使用全部点生成2D地图")
            mask = np.ones(len(points), dtype=bool)
        else:
            # 计算绝对高度范围
            height_min_abs = y_min + y_range * height_ratio_min
            height_max_abs = y_min + y_range * height_ratio_max
            
            mask = (points[:, 1] >= height_min_abs) & (points[:, 1] <= height_max_abs)
            
            rospy.loginfo_throttle(10, f"🗺️  2D地图高度过滤: [{height_min_abs:.2f}, {height_max_abs:.2f}] (范围:{y_range:.2f})")
        
        points_xy = np.column_stack((points[mask, 0], points[mask, 2]))  # X, Z
        
        if len(points_xy) < 50:
            rospy.logwarn_once("高度范围内点太少，跳过生成地图")
            return None
        
        # 计算地图边界（加1m margin）
        x_min, y_min = points_xy.min(axis=0) - 1.0
        x_max, y_max = points_xy.max(axis=0) + 1.0
        
        width = int(np.ceil((x_max - x_min) / resolution))
        height = int(np.ceil((y_max - y_min) / resolution))
        
        # 计数网格
        grid_counts = np.zeros((height, width), dtype=np.int16)
        
        # 向量化映射
        ix = np.floor((points_xy[:, 0] - x_min) / resolution).astype(int)
        iy = np.floor((points_xy[:, 1] - y_min) / resolution).astype(int)
        
        valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        ix, iy = ix[valid], iy[valid]
        np.add.at(grid_counts, (-iy, ix), 1) # 注意 Y 轴取反 不然2d occmap会颠倒
        
        # 生成 occupancy 数据
        data = np.zeros((height, width), dtype=np.int8)
        data[grid_counts >= occupied_thresh] = 100      # occupied
        data[(grid_counts > 0) & (grid_counts < occupied_thresh)] = -1  # unknown
        # 其余为 0 (free)
        
        # ROS OccupancyGrid 要求从左下角开始，Y轴向上 → 翻转
        data = data[::-1, :]
        
        # 构造消息
        occ_msg = OccupancyGrid()
        occ_msg.header.stamp = rospy.Time.now()
        occ_msg.header.frame_id = "map"
        
        occ_msg.info.resolution = resolution
        occ_msg.info.width = width
        occ_msg.info.height = height
        occ_msg.info.origin.position.x = x_min
        occ_msg.info.origin.position.y = y_min
        occ_msg.info.origin.position.z = 0.0
        occ_msg.info.origin.orientation.w = 1.0
        
        occ_msg.data = data.flatten().tolist()
        
        return occ_msg


    def align_map_to_ground(self,point_cloud):
        """
        将点云对齐到地面
        
        Args:
            point_cloud: Open3D PointCloud 对象
        
        Returns:
            aligned_cloud: 对齐后的点云
            transform: 变换矩阵
        """
        points = np.asarray(point_cloud.points)
        
        # 1. 选择最低的点作为地面候选
        y_min = np.percentile(points[:, 1], 5) 
        ground_candidates = points[points[:, 1] < y_min + 0.5]
        
        # 2. RANSAC 拟合平面
        X = ground_candidates[:, [0,2]]  # X, Z
        y = ground_candidates[:, 1]   # Y
        
        ransac = RANSACRegressor(residual_threshold=0.1)
        ransac.fit(X, y)
        
        # 3. 计算平面法向量
        # 平面方程: Y = a*X + b*Z + c
        # 法向量: (-a, -b, 1)
        a, b = ransac.estimator_.coef_
        c = ransac.estimator_.intercept_
        
        normal = np.array([-a, -b, 1])
        normal = normal / np.linalg.norm(normal)
        
        # 4. 计算旋转矩阵（将法向量对齐到 Z 轴）
        z_axis = np.array([0, 1, 0])
        
        # 旋转轴
        rotation_axis = np.cross(normal, z_axis)
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        
        # 旋转角度
        angle = np.arccos(np.dot(normal, z_axis))
        
        # Rodrigues 旋转公式
        K = np.array([
            [0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0]
        ])
        
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        
        # 5. 应用变换
        points_aligned = points @ R.T
        
        # 6. 平移到地面 Z=0
        z_offset = np.percentile(points_aligned[:, 2], 5)
        points_aligned[:, 2] -= z_offset
        
        # 创建对齐后的点云
        aligned_cloud = o3d.geometry.PointCloud()
        aligned_cloud.points = o3d.utility.Vector3dVector(points_aligned)
        if point_cloud.has_colors():
            aligned_cloud.colors = point_cloud.colors
        
        # 构建完整的变换矩阵
        transform = np.eye(4)
        transform[:3, :3] = R
        transform[2, 3] = -z_offset
        
        return aligned_cloud, transform

    def align_to_ground(self):
        if len(np.asarray(self.all_point_cloud.points)) == 0:
            rospy.logwarn("点云为空，无法对齐到地面")
            return
        aligned_cloud, transform = self.align_map_to_ground(self.all_point_cloud)
        self.all_point_cloud = aligned_cloud
        rospy.loginfo("点云已对齐到地面")
        
        # 保存变换矩阵
        # np.save("ground_alignment_transform.npy", transform)
        print("Ground alignment transform:\n", transform)

    def clear_folder(self,folder_path):
        """
        快捷清空文件夹下的所有文件和子文件夹（保留根文件夹）
        :param folder_path: 目标文件夹路径
        """
        # 先检查文件夹是否存在，避免报错
        if not os.path.exists(folder_path):
            print(f"文件夹 {folder_path} 不存在，无需删除")
            return
        
        # 遍历并删除所有内容
        for item in os.listdir(folder_path):
            item_path = os.path.join(folder_path, item)
            try:
                # 如果是文件/链接，直接删除
                if os.path.isfile(item_path) or os.path.islink(item_path):
                    os.unlink(item_path)
                # 如果是文件夹，递归删除整个文件夹（包括内容）
                elif os.path.isdir(item_path):
                    shutil.rmtree(item_path)
                print(f"已删除：{item_path}")
            except Exception as e:
                print(f"删除失败 {item_path}：{e}")


if __name__ == "__main__":
    rospy.init_node("depth_maping_node")
    i2d = None
    
    try:
        i2d = Img2DepthMaping()
        rospy.loginfo("Img to maping...")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号，正在关闭节点...")
    except Exception as e:
        rospy.logerr(f"节点运行出错: {e}")
    finally:
        # 设置关闭标志
        if i2d is not None:
            i2d.is_shutdown = True
            rospy.loginfo("正在保存数据并清理资源...")
            
            # 等待一小段时间让回调函数完成
            rospy.sleep(0.5)
            
            # 保存点云
            try:
                output_path = os.path.join(script_dir, "pointCloud/HT_vslam.ply")
                os.makedirs(os.path.dirname(output_path), exist_ok=True)
                o3d.io.write_point_cloud(output_path, i2d.all_point_cloud)
                rospy.loginfo(f"✓ 点云已保存到: {output_path}")
            except Exception as e:
                rospy.logwarn(f"✗ 保存点云失败: {e}")
            
            # 关闭可视化窗口
            if i2d.enable_visualization:
                try:
                    if hasattr(i2d, 'vis'):
                        i2d.vis.destroy_window()
                    rospy.loginfo("✓ 可视化窗口已关闭")
                except Exception as e:
                    rospy.logwarn(f"关闭可视化窗口失败: {e}")
            
            # 关闭 OpenCV 窗口
            try:
                cv2.destroyAllWindows()
            except:
                pass
            
            rospy.loginfo("节点已安全关闭")
            # if i2d.enable_visualization:
            try:
                o3d.visualization.draw_geometries([i2d.all_point_cloud])
            except Exception as e:
                rospy.logwarn(f"显示最终点云失败: {e}")
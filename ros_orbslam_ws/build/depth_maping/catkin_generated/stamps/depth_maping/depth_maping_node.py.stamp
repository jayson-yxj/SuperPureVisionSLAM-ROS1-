import argparse
import cv2
import glob
import matplotlib
import numpy as np
import os
import sys
import torch
import open3d as o3d
import pypose as pp

# 添加当前脚本目录到 Python 路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from depth_anything_v2.dpt import DepthAnythingV2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from depth_maping.msg import ImagePose
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

from plan_path import InteractivePathPlanner

from sklearn.linear_model import RANSACRegressor # 用于平面拟合
'''
indoor  outdoor
'''

class Img2DepthMaping:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='Depth Anything V2 Metric Depth Estimation')
        
        self.input_size = 518
        self.outdir = './vis_depth'
        self.encoder = 'vitb' # choices=['vits', 'vitb', 'vitl', 'vitg']
        self.load_from = "/home/sunteng/Downloads/depth_anything_v2_metric_hypersim_vitb.pth"
        self.max_depth = 70.0
        
        self.SAVE = False
        self.save_numpy = False
        self.pred_only = True
        self.grayscale = True
        
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

        self.filenames = []
        self.pose_files = []

        # *********************** ros Sub & Pub ************************* #
        self.bridge = CvBridge()
        self.image_pose = rospy.Subscriber("/orb_slam3/image_pose",ImagePose,self.depth_solver,queue_size=10)
        
        self.rate = rospy.Rate(10)
        self.pcl_pub = rospy.Publisher('/o3d_pointCloud',PointCloud2,queue_size=10)
        
        # 发布深度图给 Octomap Mapping 节点
        self.depth_pub = rospy.Publisher('/depth_anything/depth_image', Image, queue_size=10)
        rospy.loginfo("深度图发布器已初始化: /depth_anything/depth_image")

        # *********************** needed pose ******************* #
        self.now_pose = pp.SE3(torch.tensor([0., 0., 0., 0., 0., 0., 1.])) # 初始化T

        # ************************ 滑动窗口点云功能 ********************** #
        # 是否启用滑动窗口模式（只显示最近N帧）
        self.enable_sliding_window = rospy.get_param('~enable_sliding_window', True)
        # 滑动窗口大小（保留最近N帧点云）
        self.sliding_window_size = rospy.get_param('~sliding_window_size', 6)
        # 存储每一帧的点云
        self.point_cloud_frames = []  # 列表，存储每帧的 PointCloud 对象
        
        rospy.loginfo(f"滑动窗口模式: {'启用' if self.enable_sliding_window else '禁用'}")
        if self.enable_sliding_window:
            rospy.loginfo(f"滑动窗口大小: {self.sliding_window_size} 帧")

        # ************************ 点云查看器 ********************** #
        # 添加可视化开关，避免 GLX 错误
        self.enable_visualization = rospy.get_param('~enable_visualization', False)
        
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
        # ************************ planer ********************** #
        self.planner = InteractivePathPlanner()
        
        # 添加关闭标志
        self.is_shutdown = False

    def depth_solver(self,data):
        # 检查节点是否正在关闭
        if self.is_shutdown or rospy.is_shutdown():
            return

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
        
        T_pp = pp.SE3(torch.tensor(translation + quaternion)) # Tcw
        T_pp_inv = pp.Inv(T_pp) # Twc

        original_translation = T_pp_inv.translation()
        original_rotation = T_pp_inv.rotation() 

        new_translation = original_translation * self.translation_size

        T_pp_inv_new = pp.SE3(torch.cat([new_translation, original_rotation]))

        print("SE3: ",T_pp)
        print("SE3_inv: ",T_pp_inv)

        raw_image = cv_image.copy()
        undistorted_frame = cv2.remap(raw_image, self.map1, self.map2, cv2.INTER_LINEAR)
        
        depth = self.depth_anything.infer_image(undistorted_frame, self.input_size)
        depth_npy = depth.copy()
        
        # 发布深度图（使用与位姿相同的时间戳）
        if not self.is_shutdown and not rospy.is_shutdown():
            try:
                # 将深度图转换为 ROS Image 消息（32位浮点数格式）
                depth_msg = self.bridge.cv2_to_imgmsg(depth_npy, encoding="32FC1")
                depth_msg.header.stamp = data.header.stamp  # 使用相同的时间戳
                depth_msg.header.frame_id = "camera"
                self.depth_pub.publish(depth_msg)
            except Exception as e:
                rospy.logwarn_throttle(5, f"发布深度图失败: {e}")

        if True : # self.is_needed_pose(T_pp_inv_new,dis_range=2.,yaw_range=15.,pitch_range=15.)
            point_cloud = self.npy_depth_to_point_cloud_cut(depth_npy,
                                                            self.fx,
                                                            self.fy,
                                                            self.cx,
                                                            self.cy,
                                                            T_pp_inv_new,
                                                            depth_scale=1.0,
                                                            rgb_image=undistorted_frame,
                                                            img_cup_size=128
                                                            )

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
            
            # 只在可视化启用时更新窗口
            if self.enable_visualization:
                try:
                    self.vis.update_geometry(self.all_point_cloud)
                    # 如果是第一帧，重置视角
                    if self.reset_view:
                        self.vis.reset_view_point(True)
                        self.reset_view = False
                except Exception as e:
                    rospy.logwarn_once(f"更新可视化失败: {e}")

            print("\n******************* this is the needed pose! ********************\n")
            cv2.imshow(f"depth_maping_node_image",undistorted_frame)
            cv2.waitKey(1)
        
        # 发送pc2点云话题
        if not self.is_shutdown and not rospy.is_shutdown():
            try:
                all_pointCloud_pc2 = self.o3d_to_ros_pointCloud2(self.all_point_cloud,"map")
                self.pcl_pub.publish(all_pointCloud_pc2)
            except rospy.ROSException as e:
                rospy.logwarn_throttle(5, f"发布点云失败（节点可能正在关闭）: {e}")
            except Exception as e:
                rospy.logwarn(f"点云转换失败: {e}")

        # 渲染（仅在可视化启用时）
        if self.enable_visualization and not self.is_shutdown:
            try:
                self.vis.poll_events()
                self.vis.update_renderer()
            except Exception as e:
                rospy.logwarn_throttle(10, f"渲染失败: {e}")


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
    
    def npy_depth_to_point_cloud_cut(self,depth_map_npy, fx, fy, cx, cy, T_pp, depth_scale=1.0, rgb_image=None, img_cup_size=8,voxel_size=1):
        depth_map = depth_map_npy
        hight,width = depth_map.shape
        
        start_x, end_x = int(width/img_cup_size), int(width*(img_cup_size-1)/img_cup_size)
        start_y, end_y = int(hight/img_cup_size), int(hight*(img_cup_size-1)/img_cup_size)

        u,v = np.meshgrid(np.arange(width/img_cup_size,width*(img_cup_size-1)/img_cup_size), np.arange(hight/img_cup_size,hight*(img_cup_size-1)/img_cup_size))

        depth_cropped = depth_map[start_y:end_y, start_x:end_x]
        valid_mask = (depth_cropped > 0) & (depth_cropped < 30) & np.isfinite(depth_cropped)
        
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

        # self.planner.update_voxel_grid(voxel_grid_loc)

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
        # points = np.asarray(o3d_points.points)
        R_cam_to_ros = np.array([
            [0,  0,  1],   # ROS X = Cam Z
            [-1, 0,  0],   # ROS Y = -Cam X
            [0, -1,  0]    # ROS Z = -Cam Y
        ])

        # points = points @ R_cam_to_ros.T

        aligned_cloud, transform = self.align_map_to_ground(self.all_point_cloud)
        points = np.asarray(aligned_cloud.points)

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
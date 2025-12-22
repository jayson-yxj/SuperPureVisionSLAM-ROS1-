import argparse
import cv2
import glob
import matplotlib
import numpy as np
import os
import torch
import open3d as o3d
import pypose as pp

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
'''
indoor  outdoor
'''

class Img2DepthMaping:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='Depth Anything V2 Metric Depth Estimation')
        
        self.input_size = 518
        self.outdir = './vis_depth'
        self.encoder = 'vitb' # choices=['vits', 'vitb', 'vitl', 'vitg']
        self.load_from = "/home/yxj/Hightorque_vision/Depth-Anything-V2/metric_depth/checkpoints/depth_anything_v2_metric_hypersim_vitb.pth"
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
        self.translation_size = 16

        self.filenames = []
        self.pose_files = []

        # *********************** ros Sub ************************* # 
        self.bridge = CvBridge()
        self.image_pose = rospy.Subscriber("/orb_slam3/image_pose",ImagePose,self.depth_solver,queue_size=10)
        
        self.rate = rospy.Rate(10)
        self.pcl_pub = rospy.Publisher('/o3d_pointCloud',PointCloud2,queue_size=10)

        # *********************** needed pose ******************* #
        self.now_pose = pp.SE3(torch.tensor([0., 0., 0., 0., 0., 0., 1.])) # 初始化T

        # ************************ 点云查看器 ********************** #
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="point cloud", width=1280, height=960)
        self.all_point_cloud = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.all_point_cloud)
        self.reset_view = True # 是否第一帧

        self.points_with_color = np.array([], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        # ************************ planer ********************** #
        self.planner = InteractivePathPlanner()

    def depth_solver(self,data):

        cv_image = self.bridge.imgmsg_to_cv2(data.image, "bgr8")

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

        if self.is_needed_pose(T_pp_inv_new):
            point_cloud = self.npy_depth_to_point_cloud(depth_npy, self.fx, self.fy, self.cx, self.cy, T_pp_inv_new, depth_scale=1.0, rgb_image=undistorted_frame)
            self.all_point_cloud += point_cloud
            self.vis.update_geometry(self.all_point_cloud)

            # 如果是第一帧，重置视角
            if self.reset_view:
                self.vis.reset_view_point(True)
                self.reset_view = False

            print("\n******************* this is the needed pose! ********************\n")
            cv2.imshow(f"depth_maping_node_image",undistorted_frame)
            cv2.waitKey(1)
        
        # 发送pc2点云话题 (我靠这个转换有点耗时啊)
        all_pointCloud_pc2 = self.o3d_to_ros_pointCloud2(self.all_point_cloud,"map")
        self.pcl_pub.publish(all_pointCloud_pc2)
        # self.rate.sleep()

        # 渲染
        self.vis.poll_events()
        self.vis.update_renderer()


    def npy_depth_to_point_cloud(self,depth_map_npy, fx, fy, cx, cy, T_pp, depth_scale=1.0, rgb_image:cv2.Mat=None):
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
    
    def npy_depth_to_point_cloud_cut(self,depth_map_npy, fx, fy, cx, cy, T_pp, depth_scale=1.0, rgb_image:cv2.Mat=None, img_cup_size=8):
        depth_map = depth_map_npy
        hight,width = depth_map.shape
        
        start_x, end_x = int(width/img_cup_size), int(width*(img_cup_size-1)/img_cup_size)
        start_y, end_y = int(hight/img_cup_size), int(hight*(img_cup_size-1)/img_cup_size)

        u,v = np.meshgrid(np.arange(width/img_cup_size,width*(img_cup_size-1)/img_cup_size), np.arange(hight/img_cup_size,hight*(img_cup_size-1)/img_cup_size))

        depth_cropped = depth_map[start_y:end_y, start_x:end_x]
        valid_mask = (depth_cropped > 0) & (depth_cropped < 20) & np.isfinite(depth_cropped)
        
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
        pcd = pcd.voxel_down_sample(voxel_size=1)
       
        voxel_grid_loc = o3d.geometry.VoxelGrid.create_from_point_cloud(
            pcd,voxel_size=1
        )

        self.planner.update_voxel_grid(voxel_grid_loc)

        return pcd

    def is_needed_pose(self,T,dis_range=4.,yaw_range=25.,pitch_range=25.):
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




if __name__ == "__main__":
    rospy.init_node("depth_maping_node")
    i2d = Img2DepthMaping()
    rospy.loginfo("Img to maping...")
    rospy.spin()
    o3d.io.write_point_cloud("/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/src/depth_maping/pointCloud/HT_vslam.ply", i2d.all_point_cloud)
    o3d.visualization.draw_geometries([i2d.all_point_cloud])
import open3d as o3d
import numpy as np

# 1. 加载点云
pcd = o3d.io.read_point_cloud("/home/yxj/Hightorque_vision/orbslam_depthmaping_ros/ros_orbslam_ws/src/depth_maping/pointCloud/HT_vslam.ply") # 替换为你的点云文件路径

# 2. 估计法线（可选，但推荐）
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# 3. Alpha Shapes 重建
# 关键参数 alpha：值越小，网格越精细，但可能包含更多噪声；值越大，网格越平滑，但可能丢失细节。
alpha = 0.03
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

# 4. 计算顶点法线（用于渲染）
mesh.compute_vertex_normals()

# 5. 可视化结果
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
import torch
import matplotlib.pyplot as plt
import cv2
import numpy as np
import open3d as o3d
import os
import yaml

from geocalib import viz2d,GeoCalib
from geocalib.camera import camera_models
from geocalib.gravity import Gravity
from geocalib.utils import deg2rad, print_calibration

current_file_path = os.path.abspath(__file__)
current_folder = os.path.dirname(current_file_path)

device = "cuda" if torch.cuda.is_available() else "cpu"
model = GeoCalib(weights="distorted").to(device)
cap = cv2.VideoCapture(2)

def gravity_to_rotation_matrix_direct(g_c):
    """
    直接从重力向量构建旋转矩阵（无欧拉角转换，更高效）
    :param g_c: 相机坐标系下的重力向量（numpy数组，单位向量）
    :return R_wc: 世界→相机旋转矩阵（3x3 numpy数组）
    """
    # 1. 归一化重力向量
    g_c = g_c / np.linalg.norm(g_c)
    
    # 2. 构建世界坐标系的Y轴（重力反方向）
    y_c = g_c  # 相机坐标系中，世界Y轴 = -重力向量
    
    # 3. 构建世界坐标系的X轴（和Z轴、Y轴正交）但是此时Y轴可能不正交
    z_c = np.array([0, 0, 1])  # 相机Z轴（光轴）
    x_c = np.cross(z_c, y_c)   # 叉乘得到X轴（正交）
    x_c = x_c / np.linalg.norm(x_c)  # 归一化
    
    # 4. 重新构建Y轴（确保X/Y/Z正交）
    y_c = np.cross(x_c, z_c)
    y_c = y_c / np.linalg.norm(y_c)
    
    # 5. 构建世界→相机旋转矩阵（列向量为世界轴在相机中的表示）
    R_wc = np.column_stack([x_c, z_c, y_c])
    
    # 相机→世界旋转矩阵（转置）
    R_cw = R_wc.T
    
    return R_cw, R_wc


img = cv2.imread(f"{current_folder}/GE_information/0.png")  # 替换为你的测试图片路径
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_rgb = cv2.resize(img_rgb, (640, 480))
img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).unsqueeze(0).float().to(device) / 255.0
results = model.calibrate(img_tensor)
gravity = results["gravity"][0].cpu().numpy()  # 转为NumPy数组
R_cw,R_wc = gravity_to_rotation_matrix_direct(gravity)

R_cw_list = R_cw.tolist()
R_wc_list = R_wc.tolist()

data_to_save = {
    'R_cw': R_cw_list,
    'R_wc': R_wc_list
}

with open(f"{current_folder}/GE_information/rotation_matrices.yaml", 'w') as file:
    yaml.dump(data_to_save, file)

print("单张图像测试：")
print("相机→世界旋转矩阵 R_cw：")
print(R_cw)
print("世界→相机旋转矩阵 R_wc：")
print(R_wc)

h, w, _ = img.shape
f = results["camera"].f[0, 0].item()  # 假设fx=fy
cx, cy = results["camera"].c[0].cpu().numpy()
scale = 10  # 投影线长度缩放因子
g_img_x = int(cx + scale * gravity[0] / gravity[2])
g_img_y = int(cy + scale * gravity[1] / gravity[2])
cv2.arrowedLine(img, (int(cx), int(cy)), (g_img_x, g_img_y), (0, 0, 255), 2)
cv2.imshow("Single Image Test", img)
cv2.waitKey(0)
cv2.destroyAllWindows()


while True:
    ret,img = cap.read()
    if not ret:
        print("无法打开摄像头！")
        break
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_rgb = cv2.resize(img_rgb, (640, 480))
    img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).unsqueeze(0).float().to(device) / 255.0  
    
    results = model.calibrate(img_tensor)
    gravity = results["gravity"][0].cpu().numpy()  # 转为NumPy数组，比如[0.03, -0.95, -0.32]
    
    print(f"重力向量：gx={gravity[0]:.4f}, gy={gravity[1]:.4f}, gz={gravity[2]:.4f}")
    print(f'重力向量数据类型：{type(gravity)}, shape: {gravity.shape}')

    R_cw,R_wc = gravity_to_rotation_matrix_direct(gravity)
    print("相机→世界旋转矩阵 R_cw：")
    print(R_cw)
    print("世界→相机旋转矩阵 R_wc：")
    print(R_wc)

    # 可视化重力方向在图像上的投影
    h, w, _ = img.shape
    f = results["camera"].f[0, 0].item()  # 假设fx=fy
    cx, cy = results["camera"].c[0].cpu().numpy()
    scale = 10  # 投影线长度缩放因子
    g_img_x = int(cx + scale * gravity[0] / gravity[2])
    g_img_y = int(cy + scale * gravity[1] / gravity[2])
    cv2.arrowedLine(img, (int(cx), int(cy)), (g_img_x, g_img_y), (0, 0, 255), 2)

    cv2.imshow("Camera Test", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
#!/usr/bin/env python3
"""
Monster 双目深度估计 ROS 节点
订阅 /orb_slam3/image_pose_stereo 话题，进行实时深度估计并显示
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
from cv_bridge import CvBridge
from pathlib import Path

# 导入 Monster 相关模块
from core.monster import Monster
from core.utils.utils import InputPadder

# 导入 ROS 消息
from monster.msg import ImagePoseStereo

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
        self.baseline = rospy.get_param('~baseline', 0.12)  # 默认基线 12cm
        self.focal_length = rospy.get_param('~focal_length', 500.0)  # 默认焦距
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
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Monster 双目深度估计节点初始化")
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"设备: {DEVICE}")
        rospy.loginfo(f"模型路径: {self.model_path}")
        rospy.loginfo(f"编码器: {self.encoder}")
        rospy.loginfo(f"迭代次数: {self.valid_iters} ⚠️ 关键性能参数")
        rospy.loginfo(f"基线: {self.baseline} m")
        rospy.loginfo(f"焦距: {self.focal_length} px")
        rospy.loginfo(f"最大视差: {self.max_disp}")
        rospy.loginfo(f"混合精度: {self.mixed_precision}")
        rospy.loginfo(f"显示: {self.display}")
        rospy.loginfo("=" * 50)
        rospy.loginfo("架构参数:")
        rospy.loginfo(f"  hidden_dims: {self.hidden_dims}")
        rospy.loginfo(f"  corr_implementation: {self.corr_implementation}")
        rospy.loginfo(f"  corr_levels: {self.corr_levels}")
        rospy.loginfo(f"  corr_radius: {self.corr_radius}")
        rospy.loginfo(f"  n_downsample: {self.n_downsample}")
        rospy.loginfo(f"  n_gru_layers: {self.n_gru_layers}")
        rospy.loginfo("=" * 50)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 加载模型
        self.load_model()
        
        # 订阅话题
        self.sub = rospy.Subscriber(
            '/orb_slam3/image_pose_stereo',
            ImagePoseStereo,
            self.image_callback,
            queue_size=1,
            buff_size=2**24     # （16MB）这玩意儿才是实时处理的关键 调大缓冲区 ROS 的默认 buff_size 只有 65536 字节 (64KB)，这对于大图像消息来说远远不够
        )
        
        # 统计信息
        self.frame_count = 0
        self.fps_list = []
        self.last_time = rospy.Time.now()
        
        rospy.loginfo("节点初始化完成，等待图像数据...")
        rospy.loginfo("订阅话题: /orb_slam3/image_pose_stereo")
        
    def load_model(self):
        """加载 Monster 模型"""
        rospy.loginfo("正在加载 Monster 模型...")
        
        # 创建参数对象
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
        
        # 加载模型
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
            
            # 转换图像
            convert_start = rospy.Time.now()
            left_img = self.bridge.imgmsg_to_cv2(msg.image_left, desired_encoding='rgb8')
            right_img = self.bridge.imgmsg_to_cv2(msg.image_right, desired_encoding='rgb8')
            convert_time = (rospy.Time.now() - convert_start).to_sec()
            
            # 检查跟踪状态
            if not msg.tracking_success:
                rospy.logwarn_throttle(1.0, "SLAM 跟踪失败，跳过当前帧")
                return
            
            # 转换为 tensor
            tensor_start = rospy.Time.now()
            image1 = numpy_to_tensor(left_img)
            image2 = numpy_to_tensor(right_img)
            tensor_time = (rospy.Time.now() - tensor_start).to_sec()
            
            # 只在第一帧输出图像尺寸
            if self.frame_count == 0:
                rospy.loginfo(f"图像尺寸: {left_img.shape[1]}x{left_img.shape[0]}")
            
            # Padding
            pad_start = rospy.Time.now()
            padder = InputPadder(image1.shape, divis_by=32)
            image1, image2 = padder.pad(image1, image2)
            pad_time = (rospy.Time.now() - pad_start).to_sec()
            
            # 深度估计
            with torch.no_grad():
                inference_start = rospy.Time.now()
                disp = self.model(image1, image2, iters=self.valid_iters, test_mode=True)
                inference_time = (rospy.Time.now() - inference_start).to_sec()
            
            # Unpad
            unpad_start = rospy.Time.now()
            disp = padder.unpad(disp)
            disp_np = disp.cpu().numpy().squeeze()
            unpad_time = (rospy.Time.now() - unpad_start).to_sec()
            
            # 将视差转换为深度
            depth_start = rospy.Time.now()
            depth = disparity_to_depth(disp_np, self.baseline, self.focal_length)
            depth_time = (rospy.Time.now() - depth_start).to_sec()
            
            # 显示
            if self.display:
                vis_start = rospy.Time.now()
                self.visualize(left_img, disp_np, depth, inference_time)
                vis_time = (rospy.Time.now() - vis_start).to_sec()
            else:
                vis_time = 0
            
            # 统计
            self.frame_count += 1
            total_time = (rospy.Time.now() - start_time).to_sec()
            fps = 1.0 / total_time if total_time > 0 else 0
            self.fps_list.append(fps)
            if len(self.fps_list) > 30:
                self.fps_list.pop(0)
            
            # 定期输出详细统计信息
            if self.frame_count % 30 == 0:
                avg_fps = np.mean(self.fps_list)
                rospy.loginfo(f"帧 {self.frame_count}: FPS={avg_fps:.1f}, "
                            f"总耗时={total_time*1000:.1f}ms")
                rospy.loginfo(f"  转换={convert_time*1000:.1f}ms, "
                            f"Tensor={tensor_time*1000:.1f}ms, "
                            f"Pad={pad_time*1000:.1f}ms")
                rospy.loginfo(f"  推理={inference_time*1000:.1f}ms, "
                            f"Unpad={unpad_time*1000:.1f}ms, "
                            f"深度={depth_time*1000:.1f}ms, "
                            f"显示={vis_time*1000:.1f}ms")
                rospy.loginfo(f"  深度范围=[{depth.min():.2f}, {depth.max():.2f}]m")
        
        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")
            import traceback
            traceback.print_exc()
    
    def visualize(self, left_img, disp_np, depth, inference_time):
        """可视化深度图"""
        try:
            # 转换视差为彩色图
            disp_color = cv2.applyColorMap(
                np.clip(disp_np * 2.0, 0, 255).astype(np.uint8),
                cv2.COLORMAP_PLASMA
            )
            
            # 转换深度为彩色图
            depth_normalized = np.clip(depth / self.baseline / 10.0, 0, 1)  # 归一化到 0-1
            depth_color = cv2.applyColorMap(
                (depth_normalized * 255).astype(np.uint8),
                cv2.COLORMAP_JET
            )
            
            # 转换左图为 BGR
            left_img_bgr = cv2.cvtColor(left_img, cv2.COLOR_RGB2BGR)
            
            # 拼接显示：原图 | 视差图 | 深度图
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
            cv2.putText(display_img, f"Depth: [{depth.min():.2f}, {depth.max():.2f}]m", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 添加标签
            cv2.putText(display_img, "Original", (w//2 - 50, h - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_img, "Disparity", (w + w//2 - 50, h - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_img, "Depth", (2*w + w//2 - 40, h - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
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
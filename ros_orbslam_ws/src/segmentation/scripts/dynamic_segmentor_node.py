#!/usr/bin/env python3
"""
动态物体分割ROS节点
订阅图像,发布动态物体掩码用于SLAM过滤
"""
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class DynamicSegmentorNode:
    def __init__(self):
        rospy.init_node('dynamic_segmentor', anonymous=False)
        
        # 参数
        model_path = rospy.get_param('~model_path', '../checkpoints/yolo26n-seg.pt')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.5)
        self.enabled = rospy.get_param('~enabled', True)
        self.mask_mode = rospy.get_param('~mask_mode', 'segmentation')  # 'segmentation' or 'bbox'
        
        # 动态物体类别ID
        self.dynamic_classes = [0, 1, 2, 3, 5, 7, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]
        
        # 加载模型
        rospy.loginfo(f"Loading YOLOv8-seg model: {model_path}")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        
        # 订阅左图像
        self.image_sub = rospy.Subscriber(
            '/stereo/raw_left', 
            Image, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 发布掩码 (0=动态物体需过滤, 255=静态区域可用)
        self.mask_pub = rospy.Publisher(
            '/dynamic_mask',
            Image,
            queue_size=1
        )
        
        # 可视化发布(可选)
        self.vis_pub = rospy.Publisher(
            '/dynamic_mask_vis',
            Image,
            queue_size=1
        )
        
        # 统计
        self.frame_count = 0
        self.total_time = 0.0
        
        rospy.loginfo("Dynamic Segmentor Node initialized")
        rospy.loginfo(f"  Confidence threshold: {self.conf_threshold}")
        rospy.loginfo(f"  Enabled: {self.enabled}")
        rospy.loginfo(f"  Mask mode: {self.mask_mode}")
    
    def image_callback(self, msg):
        if not self.enabled:
            # 如果禁用,发布全白掩码(不过滤)
            mask = np.ones((msg.height, msg.width), dtype=np.uint8) * 255
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
            return
        
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 推理
            start_time = rospy.get_time()
            results = self.model(
                cv_image,
                conf=self.conf_threshold,
                classes=self.dynamic_classes,
                verbose=False
            )[0]
            inference_time = rospy.get_time() - start_time
            
            # 生成掩码
            h, w = cv_image.shape[:2]
            dynamic_mask = np.zeros((h, w), dtype=np.uint8)
            
            if self.mask_mode == 'segmentation':
                # 模式1: 精确分割掩码
                if results.masks is not None:
                    for mask_data in results.masks.data:
                        # 调整掩码尺寸
                        mask_np = mask_data.cpu().numpy()
                        mask_resized = cv2.resize(mask_np, (w, h))
                        mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255
                        
                        # 合并到总掩码
                        dynamic_mask = cv2.bitwise_or(dynamic_mask, mask_binary)
                    
                    # 形态学操作平滑
                    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                    dynamic_mask = cv2.dilate(dynamic_mask, kernel)
            
            elif self.mask_mode == 'bbox':
                # 模式2: 边界框掩码(更快但不精确)
                if results.boxes is not None:
                    for box in results.boxes.data:
                        x1, y1, x2, y2 = map(int, box[:4])
                        # 限制在图像范围内
                        x1 = max(0, min(x1, w-1))
                        y1 = max(0, min(y1, h-1))
                        x2 = max(0, min(x2, w-1))
                        y2 = max(0, min(y2, h-1))
                        
                        # 填充边界框区域
                        dynamic_mask[y1:y2, x1:x2] = 255
            
            # 反转掩码: 0=动态(过滤), 255=静态(保留)
            static_mask = cv2.bitwise_not(dynamic_mask)
            
            # 发布掩码
            mask_msg = self.bridge.cv2_to_imgmsg(static_mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
            
            # 发布可视化(可选)
            if self.vis_pub.get_num_connections() > 0:
                vis = cv_image.copy()
                vis[dynamic_mask > 0] = vis[dynamic_mask > 0] * 0.5 + np.array([0, 0, 255]) * 0.5
                vis_msg = self.bridge.cv2_to_imgmsg(vis.astype(np.uint8), encoding='bgr8')
                vis_msg.header = msg.header
                self.vis_pub.publish(vis_msg)
            
            # 统计
            self.frame_count += 1
            self.total_time += inference_time
            
            if self.frame_count % 30 == 0:
                avg_time = self.total_time / self.frame_count
                fps = 1.0 / avg_time if avg_time > 0 else 0
                dynamic_ratio = np.sum(dynamic_mask > 0) / (h * w) * 100
                rospy.loginfo(f"FPS: {fps:.1f} | Inference: {inference_time*1000:.1f}ms | Dynamic: {dynamic_ratio:.1f}%")
        
        except Exception as e:
            rospy.logerr(f"Error in segmentation: {e}")
            # 发布全白掩码作为fallback
            mask = np.ones((msg.height, msg.width), dtype=np.uint8) * 255
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

def main():
    try:
        node = DynamicSegmentorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
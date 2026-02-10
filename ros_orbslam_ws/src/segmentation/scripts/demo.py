#!/usr/bin/env python3
"""YOLOv8-seg 分割可视化Demo - 超简洁版"""
import cv2
import numpy as np
from ultralytics import YOLO

# 加载模型
model = YOLO("../checkpoints/yolo26n-seg.pt")

# 动态物体类别ID (COCO)
DYNAMIC_IDS = [0, 1, 2, 3, 5, 7, 14, 15, 16]  # person, bicycle, car, motorcycle, bus, truck, bird, cat, dog

# 测试图片
img_path = "images/2.jpg"

# 推理
results = model(img_path)[0]
img = cv2.imread(img_path) if not img_path.startswith('http') else cv2.imdecode(np.frombuffer(
    __import__('urllib.request').request.urlopen(img_path).read(), np.uint8), cv2.IMREAD_COLOR)

h, w = img.shape[:2]
mask = np.zeros((h, w), dtype=np.uint8)
overlay = img.copy()

# 处理分割结果
if results.masks:
    for m, box in zip(results.masks.data, results.boxes):
        if int(box.cls) in DYNAMIC_IDS:
            # 生成掩码
            m_resized = cv2.resize(m.cpu().numpy(), (w, h))
            m_binary = (m_resized > 0.5).astype(np.uint8) * 255
            mask = cv2.bitwise_or(mask, m_binary)
            
            # 叠加颜色
            color = np.random.randint(50, 255, 3).tolist()
            overlay[m_binary > 0] = overlay[m_binary > 0] * 0.5 + np.array(color) * 0.5

# 显示结果
vis = np.hstack([img, overlay, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
vis = cv2.resize(vis, (1800, 600))
cv2.putText(vis, "Original", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
cv2.putText(vis, "Segmentation", (620, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
cv2.putText(vis, "Mask", (1220, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

print(f"动态物体像素占比: {np.sum(mask > 0) / mask.size * 100:.2f}%")
cv2.imshow("YOLOv8-seg Demo", vis)
cv2.waitKey(0)
cv2.destroyAllWindows()

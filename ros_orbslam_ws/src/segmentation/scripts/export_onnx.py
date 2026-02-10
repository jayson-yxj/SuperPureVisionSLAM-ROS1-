#!/usr/bin/env python3
"""
将YOLOv8-seg模型转换为ONNX格式
支持FP32和FP16精度
"""
from ultralytics import YOLO
import os

def export_to_onnx(model_path, half=False, output_dir="../checkpoints"):
    """
    导出ONNX模型
    
    Args:
        model_path: .pt模型路径
        half: 是否使用FP16半精度(需要GPU支持)
        output_dir: 输出目录
    """
    print(f"加载模型: {model_path}")
    model = YOLO(model_path)
    
    precision = "FP16" if half else "FP32"
    print(f"开始导出ONNX ({precision})...")
    
    # 导出ONNX
    onnx_path = model.export(
        format='onnx',
        imgsz=640,           # 输入尺寸
        simplify=True,       # 简化模型
        dynamic=False,       # 固定输入尺寸(更快)
        opset=12,           # ONNX opset版本
        half=half,          # FP16半精度
    )
    
    print(f"✓ ONNX模型已保存: {onnx_path}")
    
    # 显示模型信息
    import onnx
    onnx_model = onnx.load(onnx_path)
    print(f"\n模型信息:")
    print(f"  精度: {precision}")
    print(f"  输入: {onnx_model.graph.input[0].name}")
    print(f"  输出数量: {len(onnx_model.graph.output)}")
    for i, output in enumerate(onnx_model.graph.output):
        print(f"  输出{i}: {output.name}")
    
    # 获取文件大小
    size_mb = os.path.getsize(onnx_path) / (1024 * 1024)
    print(f"  文件大小: {size_mb:.2f} MB")
    
    return onnx_path

if __name__ == "__main__":
    model_path = "../checkpoints/yolo26n-seg.pt"
    
    print("=" * 60)
    print("YOLOv8-seg ONNX导出工具")
    print("=" * 60)
    
    # 导出FP32版本
    print("\n[1/2] 导出FP32版本 (默认,兼容性最好)")
    fp32_path = export_to_onnx(model_path, half=False)
    
    # 导出FP16版本
    print("\n" + "=" * 60)
    print("[2/2] 导出FP16版本 (需要GPU,速度更快)")
    try:
        fp16_path = export_to_onnx(model_path, half=True)
    except Exception as e:
        print(f"⚠ FP16导出失败: {e}")
        print("  提示: FP16需要CUDA GPU支持")
        fp16_path = None
    
    # 性能对比说明
    print("\n" + "=" * 60)
    print("精度对比:")
    print("=" * 60)
    print("FP32 (Float32):")
    print("  ✓ 精度: 最高,无损")
    print("  ✓ 兼容性: CPU/GPU都支持")
    print("  ✗ 速度: 较慢")
    print("  ✗ 文件大小: 较大")
    
    if fp16_path:
        print("\nFP16 (Float16):")
        print("  ✓ 速度: 2-3倍加速(GPU)")
        print("  ✓ 文件大小: 减半")
        print("  ✗ 精度: 轻微损失(通常<1%)")
        print("  ✗ 兼容性: 仅GPU支持")
    
    print("\n" + "=" * 60)
    print("推荐使用:")
    print("=" * 60)
    print("• CPU部署: 使用FP32")
    print("• GPU部署: 优先FP16,精度要求高时用FP32")
    print("• SLAM应用: FP32更稳定(精度优先)")
    
    print("\n使用方法:")
    print(f"  C++: cv::dnn::readNetFromONNX(\"{fp32_path}\")")
    if fp16_path:
        print(f"  C++ (GPU): cv::dnn::readNetFromONNX(\"{fp16_path}\")")
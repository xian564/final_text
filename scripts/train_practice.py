#!/usr/bin/env python3
# 练习用训练脚本：YOLO pose
# pip install ultralytics
from ultralytics import YOLO
import os

DATA = os.path.join(os.path.dirname(__file__), '..', 'practice', 'data.yaml')
model = YOLO('yolov8n-pose.pt')          # 预训练权重
model.train(
    data=DATA,
    epochs=50,
    imgsz=640,
    batch=8,
    project='runs',
    name='practice',
    patience=20,
)
# 导出部署尺寸（与 C++ 预处理一致）
model.export(format='onnx', imgsz=[480, 640], opset=12, simplify=True)

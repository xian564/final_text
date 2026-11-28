# 练习数据集（pseudo-label）

- 来源：`bag/red_4.bag` 截 20 帧；
- 标签：用现有 `armor_allfp32.onnx` 推理得到（**伪标签**，不是人工标注）；
- 划分：每 5 帧取 1 帧做 val；
- 格式：YOLO pose，一行 13 个数：
  `cls cx cy w h x1 y1 x2 y2 x3 y3 x4 y4`（归一化，顺序 TL/TR/BR/BL）。

## 训练
```bash
pip install ultralytics
python3 scripts/train_practice.py
```
或手动：
```bash
yolo pose train model=yolov8n-pose.pt \
  data=practice/data.yaml epochs=50 imgsz=640 batch=8 \
  project=runs name=practice
```
导出：
```bash
yolo export model=runs/practice/weights/best.pt \
  format=onnx imgsz=[480,640] opset=12 simplify=True
```

## 注意
- 伪标签会有错，训练前建议人工抽查 `labels/`；
- 20 张只够跑通流程，真正训练要几百张、多场景；
- `data.yaml` 的 `path` 改成你自己的绝对路径。

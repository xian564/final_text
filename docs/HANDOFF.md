# 剩余工作交接：流程 / 方法 / 代码

> 你按每节命令执行即可；代码位置都标了文件。

---

## A. 提交（tag + push）
```bash
cd ~/catkin_ws/src/final_text
git status                 # 确认干净
git push origin main
git tag -f vision-final-test-submit
git push -f origin vision-final-test-submit
```
检查：
```bash
git rev-parse --short HEAD
git rev-list -n1 vision-final-test-submit
```
两者应相同。

---

## B. 旋转模型：换干净旋转片段再评

### B1. 找旋转片段
```bash
rosbag play bag/red_4.bag --clock -l   # 已知 yaw 范围 ~124°
rosrun final_text detect_node_v3_1
rosbag record -O /tmp/rot_det.bag --duration=25 /ft/detection
```
离线看 yaw 范围：
```python
import rosbag, math
from ft_msgs.msg import Detections
def yaw(q):
    s=2*(q.w*q.z+q.x*q.y); c=1-2*(q.y*q.y+q.z*q.z)
    return math.degrees(math.atan2(s,c))
b=rosbag.Bag('/tmp/rot_det.bag'); ys=[]
for _,m,_ in b.read_messages(topics=['/ft/detection']):
    if m.detections: ys.append(yaw(m.detections[0].pose.orientation))
print(min(ys), max(ys), max(ys)-min(ys))
```

### B2. 同时跑两个跟踪器
```bash
rosrun final_text ft_tracker_node
rosrun final_text ft_rotation_tracker_node
rosbag record -O /tmp/rot.bag --duration=25 /ft/detection /ft/track /ft/track_rot
```

### B3. 评估
```bash
rosrun final_text eval_prediction.py /tmp/rot.bag /ft/track
rosrun final_text eval_prediction.py /tmp/rot.bag /ft/track_rot
```
（脚本在 `scripts/eval_prediction.py`，默认检测话题 `/ft/detection`，
NN 时加第三个参数 `/ft/detection_nn`。）

### B4. 若还要改进：关联
`src/ft_rotation_tracker_node.cpp` 的 `detCb`：
```cpp
double d = std::sqrt(...);
if (d < best) { best = d; pick = i; }
```
可加约束：
```cpp
if (lock_ratio_ > 0 && std::abs(det.is_large_armor - last_large_)) continue;
```
或在 `config/rotation.yaml` 调 `assoc_gate`（0.5–0.8）。

### B5. 高转速段统计
`docs/ROTATION_COMPARE.md` 里已有按 `>30°/s` 分档的脚本思路；
把 `rate(t)` 阈值改 20/40 度，分别统计均值/中位。

---

## C. 神经网络训练（真训练）

### C1. 安装
```bash
pip install ultralytics
python3 -c "import torch, ultralytics; print(torch.__version__, ultralytics.__version__)"
```
> 无 GPU 时：`pip install torch --index-url https://download.pytorch.org/whl/cpu`

### C2. 数据
现有：`practice/`（20 帧伪标签，只够跑通）。
真数据：
```bash
python3 scripts/make_practice_dataset.py \
  --bag bag/red_1\&3\(1\).bag --out practice_more --n 200
```
或人工用 labelme 标四点，转 YOLO pose：
```
一行 13 个数：cls cx cy w h x1 y1 x2 y2 x3 y3 x4 y4
顺序 TL/TR/BR/BL，全部 / 宽高 归一化
```

### C3. 训练
```bash
python3 scripts/train_practice.py
# 或
yolo pose train model=yolov8n-pose.pt \
  data=practice/data.yaml epochs=100 imgsz=640 batch=8 \
  project=runs name=practice
```

### C4. 导出 + 接入
```bash
yolo export model=runs/practice/weights/best.pt \
  format=onnx imgsz=[480,640] opset=12 simplify=True
rosrun final_text detect_node_nn _model_path:=/path/best.onnx
```
检查：
```
NN ready=1
NN armors=1 infer_avg=xx ms
```

### C5. 训练代码（`scripts/train_practice.py`）
```python
from ultralytics import YOLO
model = YOLO('yolov8n-pose.pt')
model.train(data='practice/data.yaml', epochs=50, imgsz=640,
            batch=8, project='runs', name='practice', patience=20)
model.export(format='onnx', imgsz=[480,640], opset=12, simplify=True)
```

---

## D. 视频补拍 / 换段

### D1. 录帧
```bash
rosrun final_text record_debug_video.py /tmp/vid/s4 40
# 或
python3 scripts/record_debug_video.py /tmp/vid/s4 40
```
（订阅 `/ft/debug/image`，10Hz 存 jpg。）

### D2. 拼视频
```bash
ffmpeg -framerate 10 -i /tmp/vid/s4/f_%05d.jpg \
  -vf scale=960:720 -c:v libx264 -crf 23 -pix_fmt yuv420p seg4.mp4
# 再加标题卡、concat
```

### D3. 建议
- 第三段换 `red_1&3(1).bag` 近距或 `red_4.bag` 正常亮度段；
- 保留 `blue_guard` 作为“低照度”单独一页。

---

## E. RViz 里显示图像 + TF

编辑 `rviz/armor_v3.rviz`，加一个 Image display：
```yaml
- Class: rviz/Image
  Enabled: true
  Image Topic: /ft/debug/image
  Name: DebugImage
  Queue Size: 2
  Transport Hint: raw
```
或启动后手动 Add → Image → topic `/ft/debug/image`。
截图：
```bash
export DISPLAY=:0
xwininfo -root -tree | grep -i rviz
import -window <id> /tmp/rviz.png
```

---

## F. 编号准确率统计

录 `/ft/detection_nn`，逐帧统计：
```python
import rosbag
from ft_msgs.msg import Detections
b=rosbag.Bag('/tmp/nn.bag')
nums=[]
for _,m,_ in b.read_messages(topics=['/ft/detection_nn']):
    for d in m.detections:
        nums.append(d.number)
print({k: nums.count(k) for k in set(nums)})
```
人工抽 20 帧核对，算：
```
准确率 = 正确帧 / 有检测帧
```

---

## G. 仓库体积（可选）

```bash
# 不再跟踪 28MB 的 so（历史仍保留）
git rm --cached third_party/ort/lib/libonnxruntime.so
echo "third_party/ort/lib/*.so" >> .gitignore
git commit -m "chore: 不再跟踪ORT二进制"
```
或用 Git LFS：
```bash
git lfs track "*.so" "*.mp4"
```

---

## H. 其它可做

| 项 | 做法 |
|---|---|
| `omega_min` 再调 | `config/rotation.yaml` 0.15/0.25 对比 |
| 旋转只在高转速启用 | 已在代码里用 `rot_mode_` 滞回 |
| 传统编号 | `models/digit_1..5.png` 放入后重跑 |
| 漏检分析 | 抽帧看 `z/面积/亮度`，归类写 README |

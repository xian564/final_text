# RViz 截图笔记（问题 + 步骤）

## 1. 现象
- 截出来的图是空的/只有黑底；
- 没有 Displays 面板；
- 或者截到了 “ROS 1 End-of-Life” 弹窗。

## 2. 原因
| 现象 | 原因 |
|---|---|
| 3D 区域空 | `armor_v3.rviz` 只有 TF/Marker，没有 Image display；且没在跑检测时 TF 也不存在 |
| 截到旧窗口 | `xdotool search --class rviz` 会列出多个 RViz，`head -1` 可能选到旧的 |
| 截到弹窗 | `xwininfo | grep rviz` 也会匹配 EOL 弹窗 |
| 图很小 | 窗口只有 659×326，或 Image 面板被挤在左上角 |

## 3. 正确步骤

### 3.1 用带 Image 的配置启动
```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
rviz -d $(rospack find final_text)/rviz/image_demo.rviz
```
配置内容：
- Image display：`/ft/debug/image`
- Fixed Frame：`hk_camera_ca`
- Displays 面板

### 3.2 关掉弹窗/旧窗口
```bash
pkill -x rviz          # 先全关，避免选错
# 再重新起 image_demo
```

### 3.3 找“这次”的窗口
```bash
PID=$(pgrep -n rviz)                 # 最新的 rviz
ID=$(xdotool search --pid $PID | head -1)
echo $PID $ID
```

### 3.4 放大并截图
```bash
export DISPLAY=:0
xdotool windowsize $ID 1400 900
xdotool windowmove $ID 0 0
import -window $ID /tmp/rviz.png
```

### 3.5 检查
```bash
ls -lh /tmp/rviz.png
```
- 应 >50KB；
- 能看到 Displays → Image；
- 图像里有检测框。

## 4. 不要做的事
- `import -window root`（会报 `Resource temporarily unavailable`）；
- 用 `grep rviz | head -1` 选窗口（可能选到弹窗/边框）；
- 截 `mutter-x11-frames` 的边框窗口；
- 在没跑检测/跟踪时截 TF（没有 `armor_v3`）。

## 5. 证据怎么用
| 图 | 证明什么 |
|---|---|
| `docs/rviz_pose.png` | RViz 的 Image display 正常、话题 `/ft/debug/image` 有数据 |
| `docs/nn_pose.png` | 四角点 + 三轴 + `t=(x,y,z)m`，真正的位姿证据 |
| `docs/demo.mp4` | 运行效果 |

## 6. 常见错误
| 错误 | 修复 |
|---|---|
| `import: unable to read X window image 'root'` | 指定窗口 id，不要 root |
| `xdotool: There are no windows in the stack` | PID 不对，重新 `pgrep -n rviz` |
| 图里没有 Displays | 用了 `armor_v3.rviz`，换 `image_demo.rviz` |
| 图很小 | `xdotool windowsize` 放大；或拖 Image 面板分隔条 |

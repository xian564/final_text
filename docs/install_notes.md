# 神经网络模式环境安装记录（可复现）

## 0. 环境
- Ubuntu 20.04 容器，ROS Noetic，系统 Python 3.8
- 初始状态：没有 `pip3`、没有 `onnx` / `onnxruntime`
- 网络：apt 可用；PyPI 官方源很慢；GitHub 直连/raw 不通

## 1. 先装 pip3（系统包）
```bash
apt-get update
apt-get install -y python3-pip
pip3 --version
```

## 2. 用国内镜像装 onnx / onnxruntime
官方源会卡在下载（16MB 的 wheel 十几分钟不动），换清华镜像：
```bash
pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple onnx
pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple onnxruntime
```
验证：
```bash
python3 -c "import onnx; print(onnx.__version__)"
python3 -c "import onnxruntime as ort; print(ort.__version__, ort.get_device())"
# 记录到：onnx 1.17.0 / onnxruntime 1.16.3 (CPU)
```
注意：`onnx` 会顺带把系统 numpy 升到 1.24.4（ROS 的 Python 工具一般还能用，但要知道这件事）。

## 3. 尝试过的错误路线
1. `apt-get install python3-onnx` → 仓库里没有这个包（E: Unable to locate package）。
2. OpenCV DNN 加载原模型：
   - OpenCV 4.2：`Unsupported data type: FLOAT16`
   - 手转 FP32 后：`Conv` 层 3 个输入不支持（4.2/4.5.5 都一样）
3. `pip3 install onnxsim` → 源码编译失败（没有预编译 wheel）。
4. GitHub Release/raw 下载 ORT C++ 包 → 404/超时（网络不通）。

## 4. 模型转换（关键）
原模型：FP16 权重 + opset 22 + IR 10，ORT 1.16 只支持 opset ≤19、IR ≤9。
转换脚本 `scripts/convert_onnx_fp32.py` 做了 4 件事：
1. 所有 FP16 initializer 用 `numpy_helper` 转成 float32；
2. 所有 `Cast` 节点的 `to=FLOAT16(10)` 改成 `FLOAT(1)`；
3. `value_info` / graph 输入输出里 FP16 类型改成 float32；
4. `onnx.version_converter.convert_version(model, 19)`，再设 `ir_version = 9`。

```bash
python3 scripts/convert_onnx_fp32.py armor.onnx armor_allfp32.onnx
```

验证（不跑 ROS，先确认模型能推）：
```python
import onnxruntime as ort, numpy as np
s = ort.InferenceSession("armor_allfp32.onnx", providers=["CPUExecutionProvider"])
print(s.get_inputs()[0].shape)       # [1, 3, 480, 640]
y = s.run(None, {s.get_inputs()[0].name: np.zeros((1,3,480,640), np.float32)})[0]
print(y.shape)                       # (1, 21, 6300)
```

## 5. 为什么最后用 Python ORT
- pip 的 `onnxruntime` wheel 只带 Python 绑定，**不带 C++ 头文件和 `libonnxruntime.so`**；
- 环境里也没有 ORT C++ 开发包，GitHub 又下不动；
- 所以 NN 节点先用 `scripts/nn_detector_node.py`（ORT Python）跑通，C++ 接口
  `include/final_text/detector_interface.hpp` 保留。拿到 ORT C++ 包后可替换后端。

## 6. 如果拿到 ORT C++ 包怎么接（备忘）
解压后应有：
```
onnxruntime-linux-x64-1.16.3/
  include/onnxruntime_cxx_api.h ...
  lib/libonnxruntime.so
```
CMake 里：
```cmake
include_directories(${ORT_ROOT}/include)
target_link_libraries(detect_node_nn ${ORT_ROOT}/lib/libonnxruntime.so)
set_target_properties(detect_node_nn PROPERTIES BUILD_RPATH ${ORT_ROOT}/lib)
```
然后把 `OnnxDetector` 里的 `cv::dnn` 换成 `Ort::Session`。

## 7. 跑 NN 模式
```bash
roslaunch final_text test_nn.launch model_path:=/root/catkin_ws/bag/armor_allfp32.onnx
# 输出 /ft/detection_nn、/ft/debug/image_nn
```

## 8. 踩坑清单
- 不要在 `pip install` 的同时 `pkill -f pip3 install`：会把自己那条 shell 杀掉（退出码 143）。
- 先用官方源会卡；直接上清华镜像。
- 模型 IR/opset 版本必须和 ORT 版本匹配：报 `Unsupported model IR version` 改 IR，报 `opset` 降 opset。
- FP16 模型只改 initializer 不够，`Cast`/`value_info` 里残留 FP16 会报 `bound to different types`。
- OpenCV DNN 不是万能的：这个模型 4.2/4.5.5 都跑不了。

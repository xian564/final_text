# 02 · 首帧崩溃：`findContours` 只接受 CV_8UC1

- **版本**：同上（`std::endl` 修复后，365 行）
- **命令**：
  ```bash
  rosbag play bag/red_4.bag --clock -l &
  roslaunch final_text test.launch
  ```

## 现象

`roslaunch` 后 `detect_node` 第一帧就退出：
- `grep -c "^ok$"` = **0**
- `rostopic hz /ft/debug/image` → `no new messages`
- rqt 无图

## 报错原文

```
terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.2.0) ../modules/imgproc/src/contours.cpp:195: error: (-210:Unsupported
  format or combination of formats) [Start]FindContours supports only CV_8UC1 images when
  mode != CV_RETR_FLOODFILL otherwise supports CV_32SC1 images only in function
  'cvStartFindContours_Impl'
```

## 原因

`colorSegment()` 末尾 `cv::bitwise_and(bgr, bgr, res, mask_all)` 输出的是 **3 通道 BGR（CV_8UC3）**，
而 `imageCallback()` 第 138 行直接把它传给 `cv::findContours()`，后者只接受 **单通道 CV_8UC1**。
抛出的 `cv::Exception` 不在 `catch (const cv_bridge::Exception&)` 捕获范围内 → `std::terminate`。

## 处理

按约定**未自行修改**，先记录报错并上报，等确认后再改。

## 结果

节点每帧崩一次，无法产出任何图像；`ok` 计数始终为 0。

## 遗留

需要把传给 `findContours` 的输入改成单通道 mask（见 03）。

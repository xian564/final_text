# v3 变量数据流（detection_node_v3.cpp）

## 总览
```
Image msg
  └─ bgr ──► colorSegment ──► colored
        │                        ├─ cv::cvtColor ─► hsv_full ─► v_chan
        │                        └─ cv::cvtColor ─► seg_mask（灰度掩膜）
        │                                              └─ findContours ─► contours
        │                                                                   └─► bars
        │                                                                        └─► targets
        │                                                                             └─► best ─► mt
        │                                                                                          └─► tvec/rvec
        │                                                                                               ├─► avg_t/avg_yaw ─► TF armor_v3
        │                                                                                               └─► /ft/detection
        └─ canvas（绘图底板）──► /ft/debug/image
丢帧：vel_ + v_prev_pos_ ─► 预测 TF（PRED）；FCS 用 prev_center2d_ 残影补位
```

## 1. 输入与缓存（imageCallback ≈380）
| 变量 | 内容 | 写入 | 去向 |
|---|---|---|---|
| `msg` | 图像消息 | 回调参数 | `bgr`、`stamp`、`frame_id` |
| `bgr` | BGR 原图 | cv_bridge | `colorSegment`、`canvas` |
| `canvas` | 绘图底板 | `bgr.clone()` | debug 绘制、`image_pub_` |
| `stamp` | 图像时间戳 | `msg->header.stamp` | 所有平滑/外推/消息头 |

## 2. 颜色与亮度（colorSegment ≈37 / 397）
| 变量 | 内容 | 写入 | 去向 |
|---|---|---|---|
| `colored` | 只保留红/蓝/黄色的图 | `colorSegment` | `hsv_full`、`seg_mask` |
| `hsv_full` | HSV 图 | `cvtColor` | `inRange` 颜色阈值 |
| `v_chan` | V 亮度通道 | `extractChannel` | `max_v`、`high_mask`、灯条亮度过滤 |
| `max_v` / `max_loc` | 全图最亮值/位置 | `minMaxLoc` | `guard_block` 地面反光判断 |
| `guard_block` | 底部反光标志 | 比较 `max_loc.y` | 屏蔽底部区域后继续 |
| `dyn_v_thresh` / `high_mask` | 动态高亮掩膜 | `threshold/compare` | 过曝区域融合 |
| `seg_mask` | 灰度分割掩膜 | `cvtColor`(+形态学) | `findContours` |

## 3. 灯条提取（≈441-650）
| 变量 | 内容 | 写入 | 去向 |
|---|---|---|---|
| `contours` | 外轮廓集合 | `findContours` | 面积/长宽比/亮度筛选 |
| `rr` | 最小外接旋转矩形 | `minAreaRect` | `Bar.center/long_h/tilt` |
| `cmask` / `mean_v` | 轮廓掩膜 / 框内均值亮度 | `drawContours`、`mean` | 亮度过滤 `bar_v_min_` |
| `Bar` | 灯条：`center/ pts/ rr/ long_h/ tilt/ v` | 筛选通过后构造 | `bars` |
| `bars` | 本帧灯条集合 | push 后排序/合并 | 断笔合并、共面过滤、配对 |

## 4. 配对与 PnP（≈652-850）
| 变量 | 内容 | 写入 | 去向 |
|---|---|---|---|
| `dx/dy` | 两灯条中心差 | 配对循环 | 间距/错位判断 |
| `corrected` | 透视修正间距 `dx/cosθ` | 计算 | `ratio` |
| `long_h` | 两灯条较长者 | 计算 | `ratio`、阈值 |
| `ratio` | `corrected/long_h` | 计算 | `is_large` 大/小板 |
| `is_large` | 大板标志 | `ratio > ratio_s_max_` | 选择物体宽高 |
| `lt/lb/rt/rb` | 旋转四点 TL/TR/BR/BL | 边缘点 + 高度拉伸 | `obj/img`、绘制 |
| `obj` | 3D 物体点（`pnp_scale`） | 由板宽高构造 | `solvePnP` |
| `rvec/tvec` | 旋转向量 / 平移 | `solvePnP` | `Target.t`、TF、Detections、投影 |
| `tg` / `Target` | 单个候选目标 | 配对成功构造 | `targets` |

## 5. 选择、平滑与输出（≈917-1150）
| 变量 | 内容 | 写入 | 去向 |
|---|---|---|---|
| `targets` | 本帧候选目标 | 配对/清洗后 | 目标选择 |
| `best` | 选中目标索引 | 锁定/|tilt| 择优 | `mt` |
| `mt` | 主目标 | `targets[best]` | 平滑、TF、消息、debug |
| `t2` | 光学系坐标 `(x,-y,z)` | `mt.t` 变换 | `pushSmooth` |
| `avg_t` / `avg_yaw` | 平滑后位姿 | `pushSmooth` | TF 平移、yaw |
| `z_hist_` | z 中值窗口 | 每帧 push | `avg_t[2]` |
| `pos_queue_/yaw_queue_` | 平滑队列 | 每帧 push | `avg_t/avg_yaw` |
| `ts` | TF 消息 | 填充 | `tf_broadcaster_` |
| `detection_array_pub_` | `/ft/detection` 发布器 | 构造 `Detections` | 下游跟踪/调试 |
| `image_pub_` | `/ft/debug/image` 发布器 | `canvas` | rqt/rviz |

## 6. 丢帧：FCS 与匀速外推（≈1292-1400）
| 变量 | 内容 | 写入 | 去向 |
|---|---|---|---|
| `prev_center2d_` | 上一帧目标中心 | 成功帧 | 锁定选择、FCS 距离门限 |
| `prev_had_target_` | 上一帧是否有目标 | 成功/丢失 | 锁定、外推、FCS 条件 |
| `lost_frames_` | 连续丢帧数 | 丢帧累加 | 外推窗口、5 帧重置 |
| `vel_` | 最近两帧速度 | 成功帧差分 | 预测位姿 |
| `v_prev_pos_/v_prev_stamp_` | 上一位姿/时间 | 成功帧 | `vel_`、预测 |
| `pp` | 外推位置 `t_last+v·dt` | 丢帧计算 | 预测 TF、`PRED` |
| `fcs_ok/best_rect` | 残影补位结果 | FCS 匹配 | 画面虚线框、`prev_center2d_` |

## 7. 固定参数/状态
| 变量 | 内容 | 去向 |
|---|---|---|
| `K_/D_` | 相机内参/畸变 | PnP、projectPoints |
| `armor_*_w/h` | 灯条尺寸 230×60 / 135×60 | `obj` 构造 |
| `pnp_scale_` | 物体点缩放（默认 1.0） | `obj` |
| `camera_frame_/tf_child_` | TF 父子 frame | `ts` |
| `hold_frames_/hold_max_speed_` | 外推窗口/限速 | 丢帧预测 |

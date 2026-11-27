# v3 参数 / 变量 / 结构体字段总表

## 1. 参数（构造函数 pnh_.param）

### 颜色与形态学
| 参数 | 默认 | 含义 |
|---|---|---|
| morph_size | 3 | 形态学核大小 |
| hsv_sv_min | 100 | HSV 通用 S/V 下限 |
| red_h_min / red_h_max | 0 / 10 | 红色第一段 Hue |
| blue_h_min / blue_h_max | 85 / 140 | 蓝色 Hue 范围 |
| blue_s_min / blue_v_min | 50 / 50 | 蓝色 S/V 下限 |
| yellow_h_min / yellow_h_max | 10 / 35 | 黄色 Hue |
| yellow_s_min / yellow_v_min | 100 / 100 | 黄色 S/V 下限 |
| roi_y_ratio | 0.0 | 画面下部 ROI 比例 |
| bar_v_min | 170 | 灯条框内平均亮度下限 |
| bar_keep_n | 6 | 最多保留灯条数 |
| min_area | 40 | 轮廓最小面积 |
| near_area | 3000 | 近战重分割触发面积 |
| near_sv | 30 | 近战重分割 S/V 下限 |
| edge_fuse | false | 边缘融合开关 |
| merge_bars | true | 断笔合并开关 |
| guard_y_ratio | 0.85 | 底部反光守卫线 |
| dyn_v_offset | 50 | 动态高亮阈值偏移 |
| pair_y_ratio | 0.75 | 配对 y 比例（旧逻辑） |
| white_v_min | 240 | 白场兜底 V 下限 |
| white_s_max | 30 | 白场兜底 S 上限 |
| white_area_min | 500 | 白场兜底最小面积 |
| white_aspect_min / max | 1.3 / 6.0 | 白场兜底长宽比 |
| white_enable | false | 白场兜底开关 |

### 灯条几何与配对
| 参数 | 默认 | 含义 |
|---|---|---|
| aspect_min / aspect_max | 1.3 / 6.0 | 灯条长宽比范围 |
| min_dist_x / max_dist_x | 15 / 500 | 两灯条水平间距范围 |
| max_angle_diff | 10 | 两灯条倾角差上限（度） |
| ratio_s_max | 3.0 | 大/小板分界 |
| ratio_l_max | 5.0 | 比例上限（幽灵剔除） |
| ratio_min | 0.4 | 比例下限 |
| bar_h_ratio_min | 0.6 | 两灯条高度比下限 |
| pair_dy_ratio | 0.4 | 竖直错位 × 灯条长 上限 |
| quad_align_min | 0.6 | 四边形对边一致性下限 |
| quad_diag_tol | 0.35 | 对角线相对差上限 |

### 尺寸与 PnP
| 参数 | 默认 | 含义 |
|---|---|---|
| armor_large_w / h | 0.230 / 0.060 | 大装甲灯条中心距 / 灯条长（米） |
| armor_small_w / h | 0.135 / 0.060 | 小装甲 |
| pnp_scale | 1.0 | 物体点缩放（实测标定） |
| max_z | 8.0 | 最大有效距离 |
| z_median_win | 13 | z 中值窗口 |
| max_z_jump | 1.5 | z 离群阈值 |

### 时空校验与外推
| 参数 | 默认 | 含义 |
|---|---|---|
| deadzone_px | 5.0 | 坐标死区（像素） |
| uniqueness_dt | 0.15 | 尺寸沿用时间窗 |
| max_speed | 8.0 | 物理速度上限 m/s |
| dedup_px | 120.0 | 目标去重距离 |
| teleport_margin | 20.0 | 瞬移判决余量 |
| hold_frames | 8 | 丢帧外推最大帧数 |
| hold_max_speed | 8.0 | 外推速度上限 |

### 共面过滤 / FCS
| 参数 | 默认 | 含义 |
|---|---|---|
| plane_filter | true | 共面过滤开关 |
| plane_dist_ratio | 1.0 | 离面阈值 = 灯条长 × ratio |
| fcs_min_area | 150.0 | FCS 候选最小面积 |
| fcs_four_span | 30.0 | 四点亮度差上限 |
| fcs_four_tol | 60.0 | 四点亮度与上帧差上限 |

### 话题 / 队列
| 参数 | 默认 | 含义 |
|---|---|---|
| image_topic | /hk_camera/image_raw | 图像话题 |
| camera_info_topic | /hk_camera/camera_info | 内参话题 |
| debug_topic | /ft/debug/image | 调试图 |
| detection_topic | /ft/detection | 检测输出 |
| sub_queue / pub_queue | 1 / 1 | 队列长度 |
| tf_child_frame | armor_v3 | TF 子 frame |

## 2. 成员变量 / 状态
| 变量 | 类型 | 含义 |
|---|---|---|
| K_ / D_ | Mat | 相机内参 / 畸变 |
| width_ / height_ | int | 图像尺寸 |
| camera_frame_ | string | 相机 frame |
| is_ready_ | atomic<bool> | 内参是否就绪 |
| pos_queue_ | deque<Vec3d> | 位置平滑队列 |
| yaw_queue_ | deque<double> | yaw 平滑队列 |
| z_hist_ | deque<double> | z 中值窗口 |
| last_z_id_ / last_z_large_ | int/bool | z 窗口归属 |
| size_hist_ | deque<bool> | 尺寸众数历史 |
| yaw_hist_ | deque<pair> | yaw 历史 |
| has_last_ | bool | 是否有上一帧 |
| last_stamp_ | double | 上帧时间 |
| last_center2d_ | Point2f | 上帧像素中心 |
| last_is_large_ | bool | 上帧大小 |
| last_pos_ | Vec3d | 上帧平滑位置 |
| last_yaw_ | double | 上帧 yaw |
| omega_deg_s_ | double | 角速度估计 |
| lost_frames_ | int | 连续丢帧数 |
| has_sent_ | bool | 是否发过 TF |
| last_sent_px_ | Point2f | 上次发 TF 的像素 |
| has_vsample_ / has_vel_ | bool | 速度样本 / 有效 |
| v_prev_pos_ | Vec3d | 上一次成功位置 |
| v_prev_stamp_ | double | 上一次成功时间 |
| vel_ | Vec3d | 估计速度 |
| prev_had_target_ | bool | 上帧是否有目标 |
| prev_id_ | int | 上帧 ID |
| prev_is_large_ | bool | 上帧大小 |
| prev_center2d_ | Point2f | 上帧中心（锁定/FCS） |
| prev_stamp_ | double | 上帧时间（FCS） |
| prev_aspect_ | double | 上帧长宽比 |
| prev_grad_ | double | 上帧亮度梯度 |
| prev_bar_ratio_ | double | 上帧灯条比例 |
| prev_four_[4] | double | 上帧四点亮度 |
| assist_mode_ / assist_left_ / assist_lost_ | bool/int | 兜底模式状态 |

## 3. 局部变量（imageCallback）
| 变量 | 含义 |
|---|---|
| stamp | 图像时间戳 |
| bgr | 原图 |
| canvas | 绘图底板 |
| colored | 颜色分割结果 |
| hsv_full | HSV 图 |
| v_chan | V 亮度通道 |
| max_v / max_loc | 最亮值 / 位置 |
| guard_block | 底部反光标志 |
| high_mask | 高光掩膜 |
| seg_mask | 灰度掩膜 |
| contours | 外轮廓 |
| bars | 灯条集合 |
| targets | 候选装甲 |
| best | 选中目标索引 |
| mt | 主目标 |
| t2 | (x,-y,z) 坐标 |
| avg_t / avg_yaw | 平滑位姿 |
| ts | TF 消息 |
| q | 四元数 |
| d | 单条 Detection |
| arr | 本帧 Detections |
| vc | 整车中心（装甲平均） |
| moved | 是否超过死区 |

## 4. 结构体字段
### Bar
| 字段 | 含义 |
|---|---|
| rr | 最小外接旋转矩形 |
| pts[4] | 四角点 |
| center | 矩质心 |
| tilt | 倾角 |
| long_h | 长边 |
| v | 平均亮度 |
| contour | 原始轮廓 |

### Target
| 字段 | 含义 |
|---|---|
| id | 帧内编号 |
| is_large | 大/小板 |
| ratio | 宽高比 |
| avg_tilt | 平均倾角 |
| corners | TL/TR/BR/BL |
| center2d | 像素中心 |
| t | tvec |
| yaw | 航向角 |
| z | 距离 |
| has_pose | 有 PnP |
| R | 旋转矩阵 |
| has_r | 有旋转 |

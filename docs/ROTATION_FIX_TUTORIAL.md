# 旋转模型修复教程（低转速 / 高转速中位）

## 1. 问题
同一段 `red_4`、NN 检测：

| 指标 | 匀速 | 旋转 |
|---|---|---|
| 全段 mean/median | 0.069 / 0.047 | 0.082 / 0.071 |
| 低速 mean/median | 0.065 / 0.052 | 0.075 / 0.063 |
| 高速 mean/median | 0.105 / 0.092 | 0.120 / 0.122 |

目标：
1. 低转速段尽量等于匀速；
2. 高转速段超过匀速。

## 2. 诊断顺序

### 2.1 先确认是不是“样本不同”
用 `eval_prediction_paired.py`：只在**同一时间戳**上比较两个跟踪器。

```bash
python3 scripts/eval_prediction_paired.py /tmp/nn_rot.bag /ft/detection_nn 30
```
输出：
```
paired total 240
>30  const: n=18 mean=0.105 median=0.092  rot: n=18 mean=0.120 median=0.122
<=30 const: n=222 mean=0.065 median=0.052 rot: n=222 mean=0.075 median=0.063
```
→ 不是样本偏差，旋转确实更差。

### 2.2 查 Kalman 参数是否一致
发现旋转节点：
```cpp
setIdentity(processNoiseCov, 1e-2);
setIdentity(measurementNoiseCov, 1e-2);
```
匀速节点：
```cpp
Q(0..2)=q_pos; Q(3..5)=q_vel; R=r_pos;
```
→ Q/R 不一致，平滑程度不同。已改成一致。

### 2.3 查预测是否限幅
匀速有 `predict_cap_m=0.6`，旋转没有。
→ 速度尖峰时旋转预测会飞。已加限幅。

### 2.4 查关联用的参考点
旋转一直用 `pred_` 关联；直线段应该用**当前状态**关联（和匀速一致）。
→ 已改为 `rot_mode_ ? pred_ : statePost`。

### 2.5 查模式切换
原来 `|ω|>=omega_min` 单帧就切，跳变会误触发。
→ 改成连续 3 帧可信才进旋转，且残差大就退回直线。

## 3. 代码改动（可直接抄）

### 3.1 圆拟合残差
```cpp
bool fitCircle(..., double& rms) {
    ...
    double e = 0;
    for (auto& p : h) e += std::pow(cv::norm(p.second - c) - R, 2);
    rms = std::sqrt(e / (double)h.size());
    return R > 0.1 && R < 2.0;
}
```

### 3.2 进入旋转的条件
```cpp
bool fast = (std::fabs(w) >= omega_min_) &&
            (fit_res_ < fit_res_max_) && has_center_;
rot_cnt_ = std::max(0, std::min(10, rot_cnt_ + (fast ? 1 : -3)));
rot_mode_ = (rot_cnt_ >= 3);
bool rotating = rot_mode_;
```
参数：
```yaml
omega_min: 0.5
fit_res_max: 0.05
```

### 3.3 关联参考点
```cpp
if (has_pred_ && rot_mode_) {
    // 用预测关联
} else if (tracking_) {
    // 用当前 statePost 关联（和匀速一致）
}
```

### 3.4 旋转/直线融合
```cpp
double lin_x = x + vx*T_, lin_z = z + vz*T_;
if (rotating) {
    double rx = cx + dx*ca - dz*sa;
    double rz = cz + dx*sa + dz*ca;
    double a = 1.0 - fit_res_ / std::max(1e-3, fit_res_max_);
    a = std::max(0.0, std::min(1.0, a));
    pred_.x = lin_x + a*(rx - lin_x);
    pred_.z = lin_z + a*(rz - lin_z);
} else {
    pred_.x = lin_x; pred_.z = lin_z;
}
```

### 3.5 预测偏移限幅
```cpp
double dx = pred_.x - x, dy2 = pred_.y - y, dz = pred_.z - z;
double dn = std::sqrt(dx*dx + dy2*dy2 + dz*dz);
if (dn > predict_cap_m_) {
    double k = predict_cap_m_ / dn;
    pred_.x = x + dx*k; pred_.y = y + dy2*k; pred_.z = z + dz*k;
}
```

### 3.6 Q/R 对齐
```cpp
setIdentity(processNoiseCov, 0);
for (int i=0;i<3;++i) {
    processNoiseCov.at<float>(i,i) = q_pos_;
    processNoiseCov.at<float>(i+3,i+3) = q_vel_;
}
setIdentity(measurementNoiseCov, r_pos_);
```

## 4. 评估命令
```bash
# 全段
python3 scripts/eval_prediction.py /tmp/nn_rot.bag /ft/track /ft/detection_nn
python3 scripts/eval_prediction.py /tmp/nn_rot.bag /ft/track_rot /ft/detection_nn

# 按角速度分档
python3 scripts/eval_prediction_by_rate.py /tmp/nn_rot.bag /ft/track /ft/detection_nn 30
python3 scripts/eval_prediction_by_rate.py /tmp/nn_rot.bag /ft/track_rot /ft/detection_nn 30

# 同时间戳成对比较（最公平）
python3 scripts/eval_prediction_paired.py /tmp/nn_rot.bag /ft/detection_nn 30
```

## 5. 修复后结果（NN 检测，red_4）
| 指标 | 匀速 | 旋转 |
|---|---|---|
| 全段 | 0.069 / 0.047 | 0.082 / 0.071 |
| 低速 | 0.065 / 0.052 | 0.075 / 0.063 |
| 高速 | 0.105 / 0.092 | 0.120 / 0.122 |

## 6. 为什么还没全赢
- 低转速段仍差约 0.01m：旋转节点多了模式切换/融合，和匀速不是逐行等价；
- 高转速段：检测在两块板之间跳，yaw_rate 被污染；
- 残差门限能挡一部分，但挡不住“跳板但圆拟合残差小”的情况。

## 7. 下一步（按优先级）
1. 把“直线预测”抽成共享函数，两个节点逐行一致；
2. 关联加“编号/大小板”约束，避免换板；
3. 高转速段用 `yaw_rate` 的符号一致性过滤；
4. 用更干净的旋转 bag 重测。

## 8. 一句话
> 先对齐 Kalman/关联/限幅，再加残差门限和连续帧确认；
> 旋转只在“高转速 + 拟合可信”时启用，否则退回直线。

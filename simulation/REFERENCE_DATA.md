# 仿真数据参考手册

## 配置参数

| 参数 | 值 | 说明 |
|------|-----|------|
| ACTION_SCALE | 0.25 | 网络输出到目标位置的缩放因子 |
| KNEE_GEAR_RATIO | 1.667 | 膝关节减速比 |
| 控制频率 | 50 Hz | 策略推理频率 |
| PD Kp | 25.0 | 比例增益 |
| PD Kd | 0.5 | 微分增益 |

---

## 测试序列

| 阶段 | 动作 | 时长 | 命令 (x, y, yaw) |
|------|------|------|------------------|
| 0 | Stand | 1s | (0, 0, 0) |
| 1 | Forward | 2s | (1.0, 0, 0) m/s |
| 2 | Backward | 2s | (-1.0, 0, 0) m/s |
| 3 | Left | 1s | (0, 0.5, 0) m/s |
| 4 | Right | 1s | (0, -0.5, 0) m/s |
| 5 | TurnL | 1s | (0, 0, 1.0) rad/s |
| 6 | TurnR | 1s | (0, 0, -1.0) rad/s |
| 7 | Stand | 1s | (0, 0, 0) |

**总时长: 10秒**

---

## 膝关节减速比处理

### 观测构建 (与 main.cpp observations.cpp 一致)

```python
# 膝关节索引 8-11
for i in range(8, 12):
    obs_joint_pos[i] = joint_pos[i] / KNEE_GEAR_RATIO  # 除以减速比
    obs_joint_vel[i] = joint_vel[i] / KNEE_GEAR_RATIO  # 速度也除
```

### 输出处理 (与 main.cpp 一致)

```python
# 膝关节索引 8-11
for i in range(8, 12):
    action_for_target[i] = action[i] * KNEE_GEAR_RATIO  # 乘以减速比

target_q = action_for_target * ACTION_SCALE
```

---

## 网络输入 (45维)

### [0:3] Angular Velocity (角速度)

| 分量 | 范围 | 单位 |
|------|------|------|
| wx | [-2.3, 2.9] | rad/s |
| wy | [-2.2, 1.8] | rad/s |
| wz | [-1.3, 1.3] | rad/s |

### [3:6] Projected Gravity (重力投影)

| 分量 | 范围 | 说明 |
|------|------|------|
| gx | [-0.1, 0.2] | 重力在机体X轴投影 |
| gy | [-0.1, 0.1] | 重力在机体Y轴投影 |
| gz | [-1.0, -0.97] | 重力在机体Z轴投影 (≈-1) |

### [6:9] Velocity Command (速度命令)

| 分量 | 范围 | 单位 |
|------|------|------|
| cmd_x | [-1.0, 1.0] | m/s |
| cmd_y | [-0.5, 0.5] | m/s |
| cmd_yaw | [-1.0, 1.0] | rad/s |

### [9:21] Joint Position (关节位置)

**注意：膝关节已除以减速比 1.667**

| 关节 | 范围 (rad) | 说明 |
|------|------------|------|
| LF_HipA | [-0.18, 0.08] | 左前髋外展 |
| LR_HipA | [-0.15, 0.23] | 左后髋外展 |
| RF_HipA | [-0.11, 0.19] | 右前髋外展 |
| RR_HipA | [-0.16, 0.15] | 右后髋外展 |
| LF_HipF | [-0.55, 0.29] | 左前髋屈伸 |
| LR_HipF | [-0.50, 0.27] | 左后髋屈伸 |
| RF_HipF | [-0.27, 0.29] | 右前髋屈伸 |
| RR_HipF | [-0.18, 0.50] | 右后髋屈伸 |
| LF_Knee | [-0.44, 0.43] | 左前膝 (已除减速比) |
| LR_Knee | [-0.34, 0.47] | 左后膝 (已除减速比) |
| RF_Knee | [-0.45, 0.47] | 右前膝 (已除减速比) |
| RR_Knee | [-0.36, 0.48] | 右后膝 (已除减速比) |

### [21:33] Joint Velocity (关节速度)

**注意：膝关节已除以减速比 1.667**

| 关节类型 | 范围 (rad/s) |
|----------|--------------|
| HipA | ±4.6 |
| HipF | ±12.3 |
| Knee | ±10.4 (已除减速比) |

### [33:45] Last Action (上一帧动作)

与网络输出范围相同。

---

## 网络输出 (12维)

| 关节 | 范围 | 目标位置 (×0.25) |
|------|------|------------------|
| LF_HipA | [-1.4, 1.0] | [-0.36, 0.24] rad |
| LR_HipA | [-0.9, 1.7] | [-0.23, 0.43] rad |
| RF_HipA | [-0.8, 1.6] | [-0.19, 0.40] rad |
| RR_HipA | [-1.6, 1.0] | [-0.39, 0.24] rad |
| LF_HipF | [-4.7, 1.6] | [-1.17, 0.40] rad |
| LR_HipF | [-3.1, 1.5] | [-0.77, 0.37] rad |
| RF_HipF | [-1.3, 3.2] | [-0.34, 0.81] rad |
| RR_HipF | [-1.1, 2.4] | [-0.28, 0.61] rad |
| LF_Knee | [-2.2, 2.9] | [-0.55, 0.72] rad (已乘减速比) |
| LR_Knee | [-1.7, 3.2] | [-0.42, 0.81] rad (已乘减速比) |
| RF_Knee | [-3.2, 2.3] | [-0.81, 0.57] rad (已乘减速比) |
| RR_Knee | [-3.0, 2.2] | [-0.76, 0.54] rad (已乘减速比) |

---

## 目标位置 (12维)

**计算公式：**
```
target_q = action × 0.25 (膝关节已乘减速比)
```

| 关节类型 | 范围 (rad) |
|----------|------------|
| HipA | [-1.3, 1.0] |
| HipF | [-1.3, 1.4] |
| Knee | [-1.3, 0.9] (已乘减速比) |

---

## 关节顺序 (Policy顺序)

```
[LF_HipA, LR_HipA, RF_HipA, RR_HipA,  # HipA: 0-3
 LF_HipF, LR_HipF, RF_HipF, RR_HipF,  # HipF: 4-7
 LF_Knee, LR_Knee, RF_Knee, RR_Knee]  # Knee: 8-11
```

**与 main.cpp 一致，不需要重排。**

---

## 与 main.cpp 对比

| 项目 | 仿真 | main.cpp | 一致？ |
|------|------|----------|--------|
| 关节顺序 | Policy | Policy | ✅ |
| ACTION_SCALE | 0.25 | 0.25 | ✅ |
| KNEE_GEAR_RATIO | 1.667 | 1.667 | ✅ |
| 观测膝关节 | 除以 1.667 | 除以 1.667 | ✅ |
| 输出膝关节 | 乘以 1.667 | 乘以 1.667 | ✅ |
| 网络输出 Clip | ±100 | 无 | ⚠️ |
| 关节限位 Clip | 无 | 有 | ⚠️ |

---

## 使用方法

### 运行测试
```bash
cd /home/wufy/git_resp/dog/simulation
uv run python auto_test.py
```

### 查看数据
```bash
# 打印摘要
uv run python view_test_data.py --no_plot

# 显示图表
uv run python view_test_data.py
```

### 输出文件
- `auto_test_data.npz` - 数据文件
- `auto_test_plot.png` - 图表
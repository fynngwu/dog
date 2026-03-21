# 真实部署调试指南

## 问题现象
真实部署时机器狗运动异常，需要对比仿真和真实数据找出问题。

---

## 调试方案

### 方案1：数据记录对比（推荐）

#### 步骤1：收集仿真数据
```bash
cd /home/wufy/git_resp/dog/simulation

# 使用手柄/ROS2控制，收集数据
uv run python data_recorder.py --load_model ../policy.onnx --max_samples 5000 --output sim_data.npz

# 或使用ROS2控制
uv run python data_recorder.py --load_model ../policy.onnx --ros2 --output sim_data.npz
```

#### 步骤2：收集真实部署数据
确保 `main.cpp` 中的 `DataLogger` 正常工作，运行一段时间后获取 `deploy_log.bin`。

#### 步骤3：对比分析
```bash
# 仅查看仿真数据
uv run python analyze_data.py --sim_data sim_data.npz --plot_sim

# 对比仿真和部署数据
uv run python analyze_data.py --sim_data sim_data.npz --deploy_data /path/to/deploy_log.bin
```

---

### 方案2：检查关键差异点

#### 1. 关节顺序映射 ⚠️

**仿真 (MuJoCo) 顺序：**
```
[LF_A, LF_F, LF_K, LR_A, LR_F, LR_K, RF_A, RF_F, RF_K, RR_A, RR_F, RR_K]
  0     1     2     3     4     5     6     7     8     9    10    11
```

**Policy (Isaac Lab) 顺序：**
```
[LF_A, LR_A, RF_A, RR_A, LF_F, LR_F, RF_F, RR_F, LF_K, LR_K, RF_K, RR_K]
  0     1     2     3     4     5     6     7     8     9    10    11
```

**检查点：**
- `main.cpp` 中的 `motor_ids` 和 `joint_offsets` 顺序是否正确？
- 网络输出到电机的映射是否正确？

#### 2. 关节偏移量 ⚠️

**main.cpp 中定义：**
```cpp
constexpr float HIPA_OFFSET = 0.37;
constexpr float HIPF_OFFSET = 0.13;
constexpr float KNEE_OFFSET = 1.06 * 1.667;
```

**检查点：**
- 这些偏移量是否与仿真中的 `default_dof_pos` 一致？
- 仿真中 `default_dof_pos = np.zeros(12)`，意味着站立时关节角度为0
- 真实部署中，站立时关节角度应该是多少？

#### 3. 膝关节减速比 ✅

**main.cpp 中：**
```cpp
// 观测构建 (observations.cpp)
if (idx >= 8 && idx <= 11) {
    obs[idx] = (pos - offsets[idx]) / 1.667f;      // 除以减速比
    obs[joint_count + idx] = vel / 1.667f;         // 速度也除
}

// 输出处理 (main.cpp)
if (i >= 8 && i <= 11) act = act * 1.667f;         // 乘以减速比
```

**仿真中 (auto_test.py)：**
```python
KNEE_GEAR_RATIO = 1.667

# 观测构建
for i in range(8, 12):  # 膝关节索引 8-11
    obs_joint_pos[i] = joint_pos[i] / KNEE_GEAR_RATIO
    obs_joint_vel[i] = joint_vel[i] / KNEE_GEAR_RATIO

# 输出处理
for i in range(8, 12):
    action_for_target[i] = action[i] * KNEE_GEAR_RATIO
```

**状态：仿真与 main.cpp 一致 ✅**

#### 4. IMU 坐标系 ⚠️

**observations.cpp 中：**
```cpp
// IMU 坐标系转换
obs.push_back(gyro[1] * M_PI / 180.0f);   // 前向: IMU_Y
obs.push_back(-gyro[0] * M_PI / 180.0f);  // 左向: -IMU_X
obs.push_back(gyro[2] * M_PI / 180.0f);   // 上向: IMU_Z

// 重力投影
obs.push_back(-gy);  // 前向
obs.push_back(gx);   // 左向
obs.push_back(-gz);  // 上向
```

**检查点：**
- IMU 安装方向是否正确？
- 坐标系转换是否与仿真一致？

#### 5. 网络输出处理 ✅

**main.cpp 中：**
```cpp
float scale = 0.25f;  // action_scale

// 膝关节减速比处理
if (i >= 8 && i <= 11) act = act * 1.667f;

// 目标位置 = action * scale + offset
float desired = act * scale + joint_offsets[i];

// 限幅
float clipped = std::clamp(desired, lower, upper);
```

**仿真中 (auto_test.py)：**
```python
ACTION_SCALE = 0.25
KNEE_GEAR_RATIO = 1.667

# 膝关节减速比处理
for i in range(8, 12):
    action_for_target[i] = action[i] * KNEE_GEAR_RATIO

target_q = action_for_target * ACTION_SCALE + default_dof_pos
```

**检查点：**
- `action_scale` 一致 ✅
- 膝关节减速比处理一致 ✅
- 关节限位：仿真无，main.cpp 有 ⚠️

---

### 方案3：单元测试

#### 测试1：验证关节顺序
```python
# 在仿真中设置特定关节角度，观察真实机器人对应关节是否正确
# 例如：只动 LF_HipA，检查是否只有左前髋关节动
```

#### 测试2：验证IMU方向
```cpp
// 在真实部署中打印IMU原始数据
std::cout << "IMU gyro: " << gyro[0] << " " << gyro[1] << " " << gyro[2] << std::endl;
std::cout << "IMU quat: " << quaternion[0] << " " << quaternion[1] << " "
          << quaternion[2] << " " << quaternion[3] << std::endl;
```

#### 测试3：验证网络输出
```cpp
// 固定输入，检查网络输出是否与仿真一致
std::vector<float> test_obs(450, 0.0f);  // 全零输入
inference_engine.infer(test_obs, action_vec);
// 打印输出并与仿真对比
```

---

## 常见问题排查清单

- [ ] 关节顺序映射是否正确？ (Policy顺序，已确认一致)
- [ ] 关节偏移量是否正确？
- [x] 膝关节减速比是否正确应用？ ✅ 仿真与 main.cpp 一致
- [ ] IMU坐标系是否正确？
- [x] 网络输出 action_scale 是否一致？ ✅ 0.25
- [ ] 关节限位是否正确？ (仿真无，main.cpp 有)
- [ ] 控制频率是否一致（仿真50Hz，部署50Hz）？
- [ ] PD参数是否一致？
- [ ] 扭矩限制是否一致？

---

## 数据格式说明

### 仿真数据 (sim_data.npz)
| 字段 | 维度 | 说明 |
|------|------|------|
| obs_angular_vel | (N, 3) | 角速度（网络输入） |
| obs_gravity | (N, 3) | 重力投影（网络输入） |
| obs_commands | (N, 3) | 速度命令 |
| obs_joint_pos | (N, 12) | 关节位置（Policy顺序） |
| obs_joint_vel | (N, 12) | 关节速度（Policy顺序） |
| action_raw | (N, 12) | 网络原始输出 |
| target_q_sim | (N, 12) | 目标关节位置（Sim顺序） |
| tau | (N, 12) | 扭矩 |

### 部署数据 (deploy_log.bin)
- 格式：二进制 float32
- 每帧：450 (obs) + 12 (action) = 462 floats
- 需要解析 obs 的 45 维单帧结构

---

## 快速调试命令

```bash
# 1. 收集仿真数据
cd /home/wufy/git_resp/dog/simulation
uv run python data_recorder.py --load_model ../policy.onnx --max_samples 3000

# 2. 查看仿真数据
uv run python analyze_data.py --sim_data sim_data.npz --plot_sim

# 3. 对比部署数据
uv run python analyze_data.py --sim_data sim_data.npz --deploy_data /path/to/deploy_log.bin
```
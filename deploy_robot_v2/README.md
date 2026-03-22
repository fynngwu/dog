# Deploy Robot V2

模块化的四足机器人部署框架，支持安全的 policy 测试和闭环控制。

## 目录结构

```
deploy_robot_v2/
├── app/                          # 主入口程序
│   ├── policy_test_main.cpp      # 主入口：支持 hold/shadow/run 三种模式
│   └── zero_calibration_main.cpp # 校零工具（独立使用，不在正常启动流程中）
│
├── core/                         # 核心模块
│   ├── robot_config.hpp          # 机器人配置常量（电机ID、offset、限位等）
│   ├── robot_bringup.hpp         # 机器人启动管理器
│   └── robot_bringup.cpp
│
├── policy/                       # Policy 模块
│   ├── policy_runner.hpp         # Policy 推理包装器
│   └── policy_runner.cpp
│
├── logging/                      # 日志模块
│   ├── session_logger.hpp        # 会话日志记录器
│   └── session_logger.cpp
│
├── input/                        # 输入源模块
│   ├── input_source.hpp          # ICommandSource 接口
│   ├── gamepad_command_source.*  # 手柄输入
│   └── keyboard_command_source.* # 键盘输入
│
├── tools/                        # 工具程序
│   ├── inspect_feedback_main.cpp # 检测电机反馈
│   ├── ramp_to_offset_main.cpp   # 安全上电并平滑移动到 offset
│   └── shadow_policy_main.cpp    # Shadow 模式测试
│
├── include/                      # 公共头文件
├── src/                          # 源文件
│   ├── observations.cpp          # 观测组件
│   ├── robstride.cpp             # 电机控制
│   ├── tensorrt_inference.cpp    # TensorRT 推理
│   ├── can/can_interface.cpp     # CAN 接口
│   └── imu/                      # IMU 驱动
│
└── CMakeLists.txt                # 构建配置
```

## 构建要求

- CMake >= 3.14
- GCC >= 9.0
- CUDA >= 12.0
- TensorRT >= 8.0

## 编译

```bash
cd /home/wufy/git_resp/dog/deploy_robot_v2

# 创建 build 目录
mkdir -p build && cd build

# 配置（根据实际 CUDA 版本调整）
cmake -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.4 ..

# 编译
make -j4
```

编译成功后生成以下可执行文件：
- `policy_test_main` - 主入口程序
- `shadow_policy` - Shadow 模式测试
- `zero_calibration` - 校零工具
- `inspect_feedback` - 检测电机反馈
- `ramp_to_offset` - 安全上电

## 使用方法

### 1. policy_test_main（主入口）

```bash
cd /home/wufy/git_resp/dog/deploy_robot_v2/build

# hold 模式：只观测并 hold 在 offset，不推理（最安全）
./policy_test_main hold

# shadow 模式：观测 + 推理 + 日志，不发控制（默认）
./policy_test_main shadow

# run 模式：观测 + 推理 + 日志 + 发送控制（闭环）
./policy_test_main run

# 指定输入源
./policy_test_main run --input gamepad   # 手柄控制
./policy_test_main run --input keyboard  # 键盘控制
./policy_test_main run --input auto      # 自动选择（默认）

# 低增益模式（地面测试推荐）
./policy_test_main run --input keyboard --low-gain
./policy_test_main shadow --input keyboard --low-gain
```

**命令行参数：**
- `hold / shadow / run`：运行模式
- `--input <source>`：输入源（gamepad/keyboard/auto）
- `--low-gain`：使用低增益模式（kp=10, kd=0.3）

**闭环保护机制：**
- 1 秒渐入（blend-in）
- 每拍最大变化 0.03 rad
- 跟踪误差 > 0.6 rad 时紧急 hold
- 发送失败时紧急 hold

**键盘控制说明：**
- `W/S`: 前进/后退 (vx)
- `A/D`: 左移/右移 (vy)
- `Q/E`: 左转/右转 (yaw)
- `Space/R`: 命令归零
- `Ctrl+C`: 退出

**手柄控制说明：**
- 左摇杆：前后左右移动
- 右摇杆：转向
- 按键：命令归零

### 2. inspect_feedback（检测电机反馈）

```bash
./inspect_feedback
```

检测所有电机是否正常反馈，3 秒超时。用于验证通信链路。

### 3. ramp_to_offset（安全上电）

```bash
./ramp_to_offset
```

安全上电并平滑移动到 offset 姿态。流程：
1. 绑定 CAN 接口
2. 绑定电机
3. 使能电机
4. 开启自动上报
5. 等待反馈
6. 从当前姿态平滑移动到 offset

### 4. shadow_policy（Shadow 模式）

```bash
./shadow_policy
```

推理但不发送控制，用于验证 policy 是否正常工作。

### 5. zero_calibration（校零工具）

```bash
./zero_calibration
```

**警告：此工具会发送 SetZero 命令，仅在电机安装/维护时使用！**

## 硬件配置

### CAN 接口

使用 candle USB-CAN 适配器，设备名为 `candle0` ~ `candle3`。

### 电机 ID 映射

| 关节 | LF | LR | RF | RR |
|------|----|----|----|----|
| HipA | 1  | 5  | 9  | 13 |
| HipF | 2  | 6  | 10 | 14 |
| Knee | 3  | 7  | 11 | 15 |

### 关节 Offset

| 关节 | Offset (rad) |
|------|--------------|
| HipA | ±0.37 |
| HipF | ±0.13 |
| Knee | ±1.767 (1.06 × 1.667) |

### IMU

设备路径：`/dev/ttyCH341USB0`

## 安全机制

### 1. 启动流程

```
INIT
  -> BIND_CAN (检查 IsValid)
  -> BIND_MOTORS
  -> ENABLE_MOTORS
  -> ENABLE_REPORT
  -> WAIT_FEEDBACK (使用 IsMotorOnline)
  -> READ_CURRENT_POSE
  -> RAMP_TO_OFFSET (从当前姿态平滑移动)
  -> PREWARM_HISTORY (10帧真实观测)
  -> SHADOW_POLICY (可选)
  -> CLOSED_LOOP
```

### 2. 运行期检查

- **传感器 Freshness**：IMU 和电机必须在 100ms 内有更新
- **发送失败检测**：CAN 发送失败时自动降级到 hold 模式
- **推理失败保护**：推理异常时自动降级到 hold 模式

### 3. 信号处理

- 使用 `std::atomic<bool>` 保证线程安全
- 信号 handler 只置位，不做 I/O（async-signal-safe）
- 退出时平滑 hold 100 帧

### 4. SetZero 隔离

- `SetZero()` 只在 `zero_calibration` 工具中使用
- 正常启动流程**绝对不调用** `SetZero()`
- 避免上电时"先冲零点再回 offset"的问题

## 日志系统

每次运行生成一个 session 目录：

```
logs/session_YYYYMMDD_HHMMSS/
  ├── meta.json      # 元数据（模式、配置等）
  ├── events.log     # 事件日志
  └── frames.csv     # 帧数据（观测、动作、状态）
```

## 配置修改

修改 `core/robot_config.hpp` 来调整：

```cpp
namespace cfg {

// 电机 ID 映射
static const std::vector<int> kMotorIds = {
    1, 5, 9, 13,   // HipA: LF, LR, RF, RR
    2, 6, 10, 14,  // HipF: LF, LR, RF, RR
    3, 7, 11, 15,  // Knee: LF, LR, RF, RR
};

// 关节 Offset
constexpr float HIPA_OFFSET = 0.37f;
constexpr float HIPF_OFFSET = 0.13f;
constexpr float KNEE_OFFSET = 1.06f * 1.667f;

// MIT 模式参数
constexpr float kMitKp = 40.0f;
constexpr float kMitKd = 0.5f;

// 设备路径
static const std::string kImuDevice = "/dev/ttyCH341USB0";
static const std::string kPolicyEnginePath = "/home/wufy/git_resp/dog/pure_cpp/policy.engine";

}  // namespace cfg
```

## 测试流程

### 标准测试顺序

**重要：不要跳步直接上 run！必须按顺序逐步验证。**

```bash
# 第 1 步：查通信，不跑 policy
./build/inspect_feedback

# 第 2 步：平滑上电到 offset，只看 hold
./build/ramp_to_offset

# 第 3 步：跑 hold 模式，验证观测链不崩
./build/policy_test_main hold --input keyboard --low-gain

# 第 4 步：跑 shadow，不下发 policy
./build/policy_test_main shadow --input keyboard --low-gain

# 第 5 步：低增益 run（首次闭环）
./build/policy_test_main run --input keyboard --low-gain

# 第 6 步：正常增益 run（仅当前面都稳定）
./build/policy_test_main run --input keyboard
```

**注意**：`zero_calibration` 只在安装/维护时使用，**不要放进正常 bringup 或闭环测试流程**。

### 各步骤判定标准

| 步骤 | 命令 | 判定标准 | 不通过时排查 |
|------|------|----------|--------------|
| 1 | `inspect_feedback` | 12 个电机都 online，位置/速度/力矩正常刷新 | CAN 接口、电机供电、电机 ID |
| 2 | `ramp_to_offset` | 地上 offset hold 稳定，无抖动/漂移 | 零位、offset、kp/kd、机械装配 |
| 3 | `hold --low-gain` | bringup、IMU、history 预热正常，无崩溃 | IMU 初始化、传感器连接 |
| 4 | `shadow --low-gain` | `raw_action` 接近 0，无饱和，gravity 正常 | 观测定义、IMU 轴向、joint order |
| 5 | `run --low-gain` | 首拍无跳变，跟踪误差小，机身稳定 | MIT 增益、执行层方向/比例 |
| 6 | `run` | 正常增益下稳定运行 | 增益参数、接触刚度 |

### 判定速查表

| 测试结果 | 根因方向 | 优先排查 |
|----------|----------|----------|
| hold 不稳 | 硬件/执行层 | offset、零位、kp/kd |
| hold 稳，shadow 不稳 | 观测定义 | IMU 轴向、joint order、膝关节比例 |
| shadow 稳，run 不稳 | 执行层 | MIT 增益、软切入、命令限幅 |
| 低增益稳，高增益不稳 | 增益问题 | 调整 kp/kd |

### 参数说明

#### 闭环保护参数（`policy_test_main.cpp`）

```cpp
const double kBlendInTime = 1.0;        // 渐入时间 1 秒
const float kMaxStepPerTick = 0.03f;    // 每拍最大变化 0.03 rad
const float kMaxTrackErr = 0.6f;        // 最大跟踪误差 0.6 rad
```

#### MIT 增益参数（`robot_bringup.cpp`）

| 模式 | kp | kd | 用途 |
|------|----|----|------|
| 低增益 | 10.0 | 0.3 | 地面排查 |
| 正常 | cfg::kMitKp | cfg::kMitKd | 正常运行 |

#### 核心配置入口

| 文件 | 内容 |
|------|------|
| `policy_test_main.cpp` | 运行模式、闭环保护参数 |
| `robot_bringup.cpp` | bringup、MIT 参数、低增益设置 |
| `policy_runner.cpp` | action scale、joint offsets、xml limit |
| `observations.cpp` | IMU 轴映射、joint pos/vel、膝关节比例 |
| `robot_config.hpp` | kJointOffsets、kMitKp/Kd、kActionScale 等 |

### 日志字段说明

`frames.csv` 包含以下字段：

| 字段 | 维度 | 说明 |
|------|------|------|
| `single_obs` | 45 | policy 输入（IMU + command + joint + prev_action） |
| `raw_action` | 12 | 网络原始输出 |
| `desired_abs` | 12 | 目标绝对位置（加 offset 前） |
| `clipped_abs` | 12 | 限幅后目标位置 |
| `cmd_sent_abs` | 12 | 实际发送的命令 |
| `cmd_minus_pos` | 12 | 命令与实际位置差 |
| `cmd_delta` | 12 | 命令相对上一拍变化 |
| `max_track_err` | 1 | 最大跟踪误差 |
| `clamp_count` | 1 | clamp 次数 |
| `imu_fresh` | 1 | IMU 是否新鲜 |
| `motors_fresh` | 1 | 电机是否新鲜 |
| `motor_pos_abs` | 12 | 电机实际位置 |
| `motor_vel_abs` | 12 | 电机速度 |
| `motor_torque` | 12 | 电机力矩 |

### 空间说明

- **电机空间**：`motor_pos_abs`、`cmd_sent_abs` 等是绝对位置
- **关节观测空间**：`single_obs` 中的 joint pos/vel 已减去 offset，膝关节已除 1.667
- **网络 action 空间**：`raw_action` 是网络输出，还未加 offset 和 scale

### 常见问题排查

#### Q: hold 时抖动
- kp/kd 过高或过低
- 零位不准
- 机械问题

#### Q: shadow 时 action 不为 0
- IMU 零偏
- gravity projection 不对
- prev_action 初始化问题

#### Q: run 时首拍跳变
- blend-in 不够（增大 `kBlendInTime`）
- offset 与 standing pose 不一致
- `last_cmd` 初始化问题

#### Q: 某关节一直 clamp
- 该关节 offset 不对
- joint limit 设置问题
- 膝关节比例问题

#### Q: 地面一接触就乱踢
1. 先完成 hold → shadow → low-gain run 全流程
2. 检查跟踪误差保护是否触发
3. 分析日志中 `cmd_sent` vs `motor_pos`
4. 对比 shadow 和 run 的 `raw_action` 是否一致

## CAN ID 协议

Robstride 电机 CAN ID 布局：

**发送帧（主机 -> 电机）：**
- Bits 0-7: motor_id（目标电机 ID）
- Bits 8-15: host_id（主机 ID）
- Bits 16-23: reserved
- Bits 24-31: msg_type（命令类型）

**接收帧（电机 -> 主机）：**
- Bits 0-7: host_id（目标主机 ID）
- Bits 8-15: motor_id（源电机 ID）
- Bits 16-23: reserved / error
- Bits 24-31: msg_type（反馈类型）

**注意：发送和接收的 motor_id/host_id 位置是互换的！**

## 常见问题

### Q: 编译时找不到 cudart

确保 CUDA 路径正确：
```bash
cmake -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.4 ..
```

### Q: 运行时找不到 policy.engine

检查 `core/robot_config.hpp` 中的 `kPolicyEnginePath` 是否正确。

### Q: CAN 接口打开失败

检查设备是否存在：
```bash
ls /dev/candle*
```

### Q: IMU 打开失败

检查设备是否存在：
```bash
ls /dev/ttyCH341USB*
```

### Q: 电机不在线

1. 检查 CAN 接口是否正常
2. 检查电机供电
3. 运行 `inspect_feedback` 检查反馈
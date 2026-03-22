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
```

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

推荐的测试顺序：

1. `inspect_feedback` - 验证通信链路
2. `ramp_to_offset` - 验证安全上电
3. `policy_test_main hold` - 验证观测和 hold
4. `policy_test_main shadow` - 验证 policy 推理（不发控制）
5. `policy_test_main run` - 闭环控制

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
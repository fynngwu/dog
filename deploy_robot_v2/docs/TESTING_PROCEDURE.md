# 机器人闭环测试流程

本文档定义了从吊架到地面闭环的标准排查流程，用于定位"落地乱踢"等问题的根因。

---

## 前置条件

- 零位已校准（使用 `zero_calibration` 工具）
- CAN 接口正常（`candle0`, `candle1` 等）
- IMU 设备正常（`/dev/ttyUSB0` 或指定设备）
- Policy engine 文件存在

---

## 测试流程总览

```
Step 1: hold 测试     → 排除硬件/零位/增益问题
Step 2: shadow 测试   → 排除观测定义问题
Step 3: IMU 校验      → 确认 IMU 轴向/符号
Step 4: joint 校验    → 确认关节映射/比例/offset
Step 5: 低增益 run    → 首次地面闭环
```

---

## Step 1: Hold 测试

**目的**：确认不是硬件/零位/增益问题。

**命令**：
```bash
./build/policy_test_main hold --input keyboard
```

**行为**：
- 电机使能后平滑移动到 offset 位置
- 持续 hold 在 offset，不进行推理
- 不发送任何 policy 输出

**观察点**：
1. 电机是否能平滑移动到 offset 位置
2. hold 时位置是否稳定（无抖动、无漂移）
3. 手动轻推关节，是否有异常响声或抖动

**判定规则**：

| 现象 | 可能原因 | 下一步 |
|------|----------|--------|
| hold 时抖动 | kp/kd 过高或过低 | 调整 MIT 参数 |
| hold 时漂移 | 零位不准 | 重新校准零位 |
| 某关节异常 | 机械问题/电机问题 | 检查硬件 |
| hold 稳定 | ✅ 通过 | 进入 Step 2 |

**如果 hold 都不稳**：
- 优先查 `kJointOffsets`
- 检查零位校准
- 检查关节方向和 joint order
- 调整 kp/kd

---

## Step 2: Shadow 测试

**目的**：在不下发 policy 的情况下，检查观测和推理是否正常。

**命令**：
```bash
./build/policy_test_main shadow --input keyboard
```

**行为**：
- 电机 hold 在 offset
- 每帧更新观测、运行推理
- 记录 obs / action / desired_abs / clipped_abs
- **不发送 policy 输出到电机**

**观察点**：
1. 静止站立时 `raw_action` 是否接近 0
2. 哪些关节的 `desired_abs / clipped_abs` 一直贴边（clamp）
3. 轻轻推机身时，输出是否平滑、方向是否合理
4. `history` 是否正常填满

**日志分析**：
```bash
# 查看最新日志目录
ls -lt logs/

# 分析 action 分布
python3 tools/analyze_shadow_log.py logs/session_xxx/
```

**判定规则**：

| 现象 | 可能原因 | 下一步 |
|------|----------|--------|
| shadow 时 action 已经很大 | 观测定义错误 | 检查 IMU/joint 映射 |
| 某关节一直 clamp | offset 或 ratio 不对 | 检查关节定义 |
| 推机身时输出方向反 | IMU 轴向或关节符号反 | 进入 Step 3 |
| shadow 输出正常 | ✅ 观测链可能正常 | 进入 Step 5 |

**如果 shadow 就已经乱**：
- 根因更像观测定义问题
- 检查 IMU 轴向、重力投影、joint order、膝关节比例、prev_action/history

**如果 shadow 正常但 run 乱**：
- 根因更像执行层问题
- 检查 MIT 增益、首拍切入、命令变化率

---

## Step 3: IMU 校验

**目的**：确认 IMU 轴向和符号是否与训练一致。

**方法**：在 hold 或 shadow 模式下，手动推机身，观察观测值变化。

**命令**：
```bash
./build/policy_test_main hold --input keyboard
# 或
./build/shadow_policy --input keyboard
```

**IMU 观测映射**（当前定义）：
```
single_obs[0..2] = gyro (roll_rate, pitch_rate, yaw_rate)
single_obs[3..5] = gravity projection (gx, gy, gz)
```

**校验步骤**：

1. **前俯（pitch forward）**
   - 机身前倾
   - 预期：`gyro[1]` (pitch_rate) 应该为正或负（取决于定义）
   - 预期：重力投影前后分量应该变化

2. **左滚（roll left）**
   - 机身左倾
   - 预期：`gyro[0]` (roll_rate) 应该有变化
   - 预期：重力投影左右分量应该变化

3. **原地左转（yaw left）**
   - 机身逆时针旋转
   - 预期：`gyro[2]` (yaw_rate) 应该为正或负

**打印观测值**（临时调试）：
```cpp
// 在主循环中每 10 帧打印
if (tick % 10 == 0) {
    printf("IMU: gyro=[%.2f,%.2f,%.2f] grav=[%.2f,%.2f,%.2f]\n",
           single_obs[0], single_obs[1], single_obs[2],
           single_obs[3], single_obs[4], single_obs[5]);
}
```

**判定规则**：

| 现象 | 可能原因 | 修复 |
|------|----------|------|
| 前俯时符号反 | gyro 或重力投影符号反 | 修改 IMU 映射 |
| 左右反 | X 轴方向定义反 | 交换或取反 |
| yaw 方向反 | yaw 定义反 | 取反 yaw |

---

## Step 4: Joint 校验

**目的**：确认关节映射、比例、offset 是否与训练一致。

**当前关节定义**：
```cpp
// 关节顺序（12个）
// FR(0-2), FL(3-5), RR(6-8), RL(9-11)
// 每条腿: hip, thigh, knee

// 膝关节比例
knee_position = motor_position / 1.667f;

// offset
kJointOffsets[12] = { ... };  // standing pose
```

**校验清单**：

1. **关节顺序**
   - 确认 12 个关节顺序与训练完全一致
   - 确认 FR/FL/RR/RL 的定义一致

2. **左右腿镜像**
   - 检查是否需要符号镜像
   - 某些训练中左右腿关节方向相反

3. **膝关节比例**
   - 当前代码除以 1.667
   - 确认训练时是否使用相同比例

4. **Offset 定义**
   - `kJointOffsets` 应该是训练时的 standing pose
   - 不是机械零位

**验证方法**：
```bash
# 使用 inspect_feedback 查看电机状态
./build/inspect_feedback

# 对比 policy 输出的 desired_abs 和实际 motor_pos_abs
```

---

## Step 5: 低增益 Run 测试

**目的**：首次地面闭环，使用保守参数。

**前置条件**：
- Step 1-4 全部通过
- 已添加软切入、命令限幅、跟踪误差保护

**命令**：
```bash
# 首次使用低增益模式
./build/policy_test_main run --input keyboard --low-gain
```

**低增益参数**：
```cpp
kp = 10.0f;   // 原 50-100
kd = 0.3f;    // 原 1.0-2.0
```

**软切入参数**：
```cpp
blend_in_time = 1.0s;    // 1 秒渐入
max_step_per_tick = 0.03f; // 每拍最大变化 0.03 rad
```

**测试步骤**：

1. 启动后先观察 1-2 秒，确认 blend-in 平滑
2. 轻按 W 键给小前进命令
3. 观察机身是否平稳前进
4. 松开按键，确认归零平滑
5. 逐步增大命令幅度

**观察点**：
1. 首拍命令是否有跳变
2. 命令和实际位置跟踪误差
3. 是否有关节一直 clamp
4. 机身是否稳定

**判定规则**：

| 现象 | 可能原因 | 下一步 |
|------|----------|--------|
| 首拍跳变大 | blend-in 不够 | 增大 blend_in_time |
| 跟踪误差大 | 增益过低或机械问题 | 调整增益 |
| 某关节抖动 | 增益不匹配 | 单独调整该关节 |
| 稳定运行 | ✅ 通过 | 逐步增大增益 |

---

## 判定速查表

| 测试结果 | 根因方向 | 优先排查 |
|----------|----------|----------|
| hold 不稳 | 硬件/执行层 | offset、零位、kp/kd |
| hold 稳，shadow 不稳 | 观测定义 | IMU 轴向、joint order、ratio |
| shadow 稳，run 不稳 | 执行层 | MIT 增益、软切入、命令限幅 |
| 低增益稳，高增益不稳 | 增益问题 | 调整 kp/kd |

---

## 安全检查清单

每次测试前确认：

- [ ] 人员已撤离安全距离
- [ ] 急停按钮可用
- [ ] 吊架/支撑已就位（首次测试）
- [ ] 日志目录可写
- [ ] 电池电量充足
- [ ] CAN/IMU 连接正常

---

## 日志分析

日志目录结构：
```
logs/session_xxx/
├── meta.json        # 测试元信息
├── events.log       # 事件日志
└── frames.csv       # 帧数据
```

关键字段：
- `single_obs[0..44]`: 单帧观测（IMU + command + joint + prev_action）
- `raw_action[0..11]`: policy 原始输出
- `desired_abs[0..11]`: 目标绝对位置（加 offset 前）
- `clipped_abs[0..11]`: 限幅后目标位置
- `motor_pos_abs[0..11]`: 电机实际位置
- `motor_vel_abs[0..11]`: 电机速度
- `motor_torque[0..11]`: 电机力矩

分析脚本：
```bash
# 统计 action 分布
python3 tools/analyze_log.py logs/session_xxx/ --stat action

# 绘制关节轨迹
python3 tools/analyze_log.py logs/session_xxx/ --plot joint

# 检查 clamp 情况
python3 tools/analyze_log.py logs/session_xxx/ --check-clamp
```

---

## 常见问题排查

### Q1: 电机使能后立即抖动
- 检查零位是否正确
- 检查 kp/kd 是否过大
- 检查电机方向设置

### Q2: Shadow 时 action 不为 0
- 检查 IMU 零偏
- 检查 gravity projection
- 检查 prev_action 初始化

### Q3: Run 时首拍跳变
- 检查 blend-in 是否生效
- 检查 offset 是否与 standing pose 一致
- 检查 last_cmd 初始化

### Q4: 某关节一直 clamp
- 检查该关节 offset
- 检查 joint limit 设置
- 检查膝关节比例

### Q5: 地面一接触就乱踢
- 先完成 Step 1-4
- 使用低增益模式
- 检查跟踪误差保护是否触发
- 分析日志中 cmd_sent vs motor_pos

---

## 附录：测试记录模板

```
日期：____/____/____
测试人员：__________
机器编号：__________
Policy 版本：__________

Step 1 Hold:  [ ] 通过  [ ] 失败
  备注：____________________

Step 2 Shadow: [ ] 通过  [ ] 失败
  备注：____________________

Step 3 IMU:   [ ] 通过  [ ] 失败
  备注：____________________

Step 4 Joint: [ ] 通过  [ ] 失败
  备注：____________________

Step 5 Run:   [ ] 通过  [ ] 失败
  增益设置：kp=____ kd=____
  备注：____________________

问题总结：
____________________
____________________
```
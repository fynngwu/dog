# robstride.cpp / robstride.hpp

## 角色

这是 Robstride 电机协议层。  
它负责：

- 绑定多个 SocketCAN 接口
- 绑定电机
- 保存每个电机的最新状态
- MIT 参数维护
- MIT 指令编码
- 反馈帧解析
- 在线状态与 freshness 统计

## 核心数据结构

### `motor_state`
- `position`
- `velocity`
- `torque`

### `MIT_params`
- `kp`
- `kd`
- `vel_limit`
- `torque_limit`

### `MotorFault`
- `has_fault`
- `motor_index`
- `motor_id`
- `joint_name`
- `code`
- `message`
- `feedback_age_ms`

### `motor_command`
记录最后一次发送的 MIT tuple。

---

## 主要接口

- `BindCAN(...)`
- `BindMotor(...)`
- `GetMotorState(...)`
- `GetAllMotorStates(...)`
- `IsMotorOnline(...)`
- `GetLastOnlineAgeMs(...)`
- `AllMotorsOnlineFresh(...)`
- `GetOfflineMotors(...)`
- `SetMITParams(...)`
- `GetMITParams(...)`
- `SendMITCommand(...)`
- `EnableMotor(...)`
- `DisableMotor(...)`
- `EnableAutoReport(...)`
- `DisableAutoReport(...)`
- `SetZero(...)`
- `HandleCANMessage(...)`

---

## 语义重点

这个层仍然是“电机原始语义”，还没完全升到“关节语义”。  
真正的 offset、direction、joint limit 在 `MotorIO` 才统一处理。

---

## 你要排查硬件时从这里看什么

1. CAN filter 是否正确
2. 电机 online 状态是否更新
3. `last_online_time` 是否在刷新
4. `SendMITCommand()` 是否返回失败
5. 反馈解析出来的位置/速度/扭矩是否异常

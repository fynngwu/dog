# 状态推流协议

## 1. 基本约定

- 端口：`47002`
- 频率：20Hz
- 传输：TCP
- 每条状态一行 JSON
- 适用于：
  - 常驻前端订阅
  - 图表刷新
  - 回放状态展示
  - fault / offline 状态灯

---

## 2. 完整结构

```json
{
  "ok": true,
  "mode": "enabled",
  "seq": 123,
  "state": {
    "joint_positions": [0.0, "... x12"],
    "joint_velocities": [0.0, "... x12"],
    "joint_torques": [0.0, "... x12"],
    "target_joint_positions": [0.0, "... x12"],
    "offline_motors": ["RF_HipA"]
  },
  "motion": {
    "active": false,
    "name": "",
    "last_error": ""
  },
  "replay": {
    "loaded": true,
    "csv_path": "/tmp/dog_replay_csv/demo.csv",
    "cursor": 12,
    "total_frames": 600,
    "speed_factor": 1.0,
    "status": "idle"
  },
  "fault": {
    "code": "no_feedback",
    "joint_name": "RF_HipA",
    "motor_index": 2,
    "feedback_age_ms": 731,
    "message": "..."
  }
}
```

注意：
- `fault` 是可选字段
- 没故障时这个字段可能不存在

---

## 3. 字段语义

## 顶层

### `ok`
恒为 `true`，表示这是有效状态包。

### `mode`
- `enabled`
- `disabled`

### `seq`
服务端自增序号。  
前端可用它判断状态是否连续更新。

---

## `state`

### `joint_positions`
长度 12。  
语义：相对 offset 的 joint position。

### `joint_velocities`
长度 12。  
语义：相对 offset 的 joint velocity。

### `joint_torques`
长度 12。  
语义：底层控制器反馈的 torque telemetry。当前建议作为“诊断值”使用，而不要过度解释成严格换算后的关节扭矩。

### `target_joint_positions`
长度 12。  
语义：daemon 记录的最近一次目标关节相对位置。

### `offline_motors`
字符串数组。  
是 `motor_io_.GetOfflineJoints(500)` 的结果，也就是 500ms 内未见新反馈的关节名。

---

## `motion`

### `active`
当前是否有 motion worker 线程在跑。

### `name`
当前运动名字，典型值：
- `joint_sine`
- `replay`

### `last_error`
上一次 motion 的结束错误。  
正常结束时通常为空字符串。

---

## `replay`

### `loaded`
是否已经加载 CSV。

### `csv_path`
当前回放 CSV 路径。

### `cursor`
当前游标帧。

### `total_frames`
总帧数。

### `speed_factor`
本次回放的速度倍率。

### `status`
当前只会是：
- `playing`
- `idle`

---

## `fault`

### `code`
典型值：
- `no_feedback`
- `send_failed`
- `enable_failed`

### `joint_name`
故障关节名。

### `motor_index`
控制器内部 motor index。

### `feedback_age_ms`
反馈陈旧时间。

### `message`
给用户展示的文字。

---

## 4. 前端使用建议

推荐把状态拆成四个 UI 区：

1. **姿态区**
   - joint_positions
   - joint_velocities
   - target_joint_positions

2. **诊断区**
   - offline_motors
   - fault
   - motion.last_error

3. **回放区**
   - replay.cursor
   - replay.total_frames
   - replay.status

4. **连接健康区**
   - seq 是否持续递增
   - 最近一次状态包到达时间

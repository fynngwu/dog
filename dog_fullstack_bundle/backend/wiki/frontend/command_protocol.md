# 命令协议

## 1. 传输格式

- TCP
- 文本命令
- UTF-8
- 一条命令一行
- 以 `\n` 结束
- 服务端回复一条 JSON 行

示例：

```text
joint_test 0,4 0.100000\n
```

---

## 2. 通用成功返回

```json
{
  "ok": true,
  "msg": "ok"
}
```

---

## 3. 通用失败返回

```json
{
  "ok": false,
  "msg": "text message",
  "error": {
    "code": "bad_command",
    "message": "text message",
    "detail": "optional"
  }
}
```

---

## 4. 命令清单

## `ping`
作用：连通性测试

请求：
```text
ping
```

成功返回：
```json
{"ok": true, "msg": "pong"}
```

---

## `get_state`
作用：请求一次状态快照  
返回结构见 [状态推流协议](state_stream_spec.md)

请求：
```text
get_state
```

---

## `enable`
作用：使能所有电机

请求：
```text
enable
```

---

## `disable`
作用：
- 失能所有电机
- 立即中止当前运动线程
- 清零 `last_joint_targets_`

请求：
```text
disable
```

---

## `init [duration_sec]`
作用：
- EnableAll
- EnableAllAutoReport
- 重试 offline 电机
- MoveToOffset
- 发送全零相对目标作为 hold

请求：
```text
init 2.5
```

成功时可能带 `offline_motors`

---

## `set_joint <12 floats>`
作用：直接设置 12 个关节相对 offset 目标

请求示例：
```text
set_joint 0 0 0 0 0 0 0 0 0 0 0 0
```

限制：
- 必须正好 12 个浮点数
- 每个目标都会过 clamp

---

## `joint_test <idx[,idx...]> <target_rad>`
作用：将一个或多个关节设置到相对 offset 目标角

请求示例：
```text
joint_test 0,4,8 0.15
```

语义：
- 非选中关节保持 `last_joint_targets_`
- 选中关节目标统一设置为 `target_rad`

---

## `joint_sine <idx[,idx...]> <amp_rad> <freq_hz> <duration_sec>`
作用：启动一个短时 motion worker，对选中关节做正弦摆动

请求示例：
```text
joint_sine 2,6 0.2 0.5 5.0
```

语义：
- 正弦中心是 **offset=0**
- 非选中关节保持原目标
- 选中关节发 `amp * sin(2πft)`

---

## `set_mit_param <kp> <kd> <vel_limit> <torque_limit>`
作用：修改全局 MIT 参数配置

请求示例：
```text
set_mit_param 40 0.5 44 17
```

---

## `load_replay_csv <csv_path>`
作用：加载 Jetson 本地 CSV 到回放缓存

请求示例：
```text
load_replay_csv /tmp/dog_replay_csv/demo.csv
```

---

## `replay_start [speed_factor]`
作用：从当前 `replay_cursor` 开始连续播放

请求示例：
```text
replay_start 1.0
```

说明：
- `speed_factor` 必须大于 0
- 当前产品约束是不支持“播放中动态改速”

---

## `replay_stop`
作用：中止 replay motion thread

---

## `replay_step`
作用：发送当前游标帧，并在发送成功后把游标推进一帧

---

## `replay_prev`
作用：游标后退一帧并立即发送该帧

边界：
- 已经在第 0 帧时，会保持在 0 并重发第 0 帧

---

## `replay_seek <frame_idx>`
作用：只移动游标，不发送动作

请求示例：
```text
replay_seek 120
```

---

## `replay_status`
作用：返回与 `get_state` 同结构的状态快照

---

## `replay <csv_path> [speed_factor]`
作用：兼容性快捷命令
- 先加载 CSV
- 再开始 replay

---

## 5. 运动互斥规则

当 `motion_active == true` 时，只有下列命令允许执行：
- `disable`
- `ping`
- `get_state`
- `replay_stop`

其余命令会返回：
- `error.code = "motion_active"`

前端要据此禁用冲突按钮。

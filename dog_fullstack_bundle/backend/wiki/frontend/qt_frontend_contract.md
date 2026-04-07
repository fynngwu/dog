# Qt 页面与交互建议

本页给 Gemini 或人工前端开发一个清晰的落地约束。

## 技术建议

- **PySide6**
- **pyqtgraph** 做实时曲线
- 采用 **Model / Service / View** 分层
- UI 层不要直接写 socket
- 后台线程通过 signal/slot 回传结果

---

## 页面结构

## 1. Dashboard
元素：
- 连接配置
- Connect / Disconnect
- Init / Enable / Disable
- 当前模式
- 状态流在线灯
- 最近 fault 摘要

## 2. Joint Debug
元素：
- 12 关节多选
- target 输入框
- sine 参数输入框
- `Apply Test`
- `Start Sine`
- 实时曲线
- 当前每个选中关节：
  - target
  - real
  - error
  - velocity
  - torque

## 3. Replay
元素：
- CSV 文件选择
- 上传到 Jetson
- Load
- Start / Stop / Step / Prev
- Seek slider / spinbox
- 当前帧 / 总帧
- replay status

## 4. Diagnostics
元素：
- 原始 JSON
- offline motors
- last fault
- last motion_error
- 命令日志
- 错误日志

---

## 交互规则

- `motion.active == true` 时，禁用会冲突的按钮
- `replay.loaded == false` 时，禁用 Start/Step/Prev/Seek
- `mode == "disabled"` 时，禁用 set_joint / joint_test / joint_sine / replay_step / replay_start
- 如果 `offline_motors` 非空，UI 应高亮对应关节
- 如果存在 `fault`，显示红色告警条

---

## 曲线建议

每个选中关节可显示 3~4 条：
- real position
- target position
- error
- velocity

如果图太拥挤：
- position / target 放主图
- error / velocity 分开子图或用 tab

---

## 性能建议

- 状态推流 20Hz，UI 不需要每包都重绘全部图元
- 推荐缓存最近 5~10 秒数据窗口
- 使用 ring buffer
- 只更新可见页面的控件

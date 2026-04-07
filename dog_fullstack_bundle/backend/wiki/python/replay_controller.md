# replay_controller.py

## 角色

给前端提供“上传 CSV → Jetson 落盘 → daemon load”这一条最核心的回放链路。

## 核心对象

### `ReplayController`
- 持有 `RobotBackend`
- 持有 `ReplayCsvStore`

## 主方法

- `stage_and_load_csv(filename, csv_text)`
- `start()`
- `stop()`
- `step()`
- `prev()`
- `seek(frame_idx)`
- `status()`

## 关键语义

### `stage_and_load_csv`
它会把前端传来的 `csv_text` 落到：
- 默认目录 `/tmp/dog_replay_csv`

然后再调用：
- `load_replay_csv(path)`

所以对 Qt 前端来说，这就是“上传文件”的最推荐入口。

## 评价

这是一个非常适合前端直接复用的类。  
它把“前端文件”和“Jetson 本地路径”之间的转换封装掉了。

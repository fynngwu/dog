# recorder.py

## 角色

这里只负责把 replay CSV 文本或二进制保存到 Jetson 固定目录。  
它不是“录制真实机器人状态”的 recorder，而是“前端上传回放 CSV 的 staging store”。

## 核心类

### `ReplayCsvStore`
- `base_dir`
- `save_text(filename, text)`
- `save_bytes(filename, data)`

## 默认目录

- `/tmp/dog_replay_csv`

## 评价

这个模块保持 filesystem-only 是对的。  
它没有把网络传输耦合进去，所以以后放到本地 GUI、HTTP 服务、RPC 服务里都能复用。

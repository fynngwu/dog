# main.cpp

## 角色

进程入口。  
负责：
- 解析 `--cmd-port`
- 解析 `--state-port`
- 安装 `SIGINT/SIGTERM`
- 启动 `TwinAgent`
- 轮询直到退出

## 输入参数

- `--cmd-port`
- `--state-port`

默认：
- `47001`
- `47002`

## 退出逻辑

收到 `SIGINT` 或 `SIGTERM` 后：
- `g_stop = 1`
- 主循环结束
- 调用 `agent.Stop()`

## 维护建议

这个文件已经足够精简，不建议再塞业务逻辑进去。

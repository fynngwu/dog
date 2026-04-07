# can_interface.cpp / can_interface.hpp

## 角色

最薄的一层 Linux SocketCAN 封装。

职责：
- 打开指定接口，如 `candle0`
- 启动 RX 线程
- 注册软件 filter
- 接收帧后回调分发
- 发送 raw CAN frame

## 主接口

- `CANInterface(const char* can_if)`
- `~CANInterface()`
- `GetName()`
- `IsValid()`
- `SendMessage(const can_frame*)`
- `SetFilter(filter, callback, user_data)`

## 设计特点

- 每个 CAN 口一个对象
- 每个对象一个 RX 线程
- filter 在用户态匹配
- callback 风格与上层兼容

## 维护建议

这个文件已经比较单一。  
后续如果要进一步提升稳定性，可以考虑：
- 更强的错误日志
- socket 断开后的重建策略
- 发送失败统计

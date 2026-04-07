# daemon 总览

C++ daemon 的职责只有四件事：

1. 管理 TCP 命令协议
2. 管理 20Hz 状态推流
3. 管理 motion worker（单电机正弦 / replay）
4. 通过 MotorIO 驱动 SocketCAN + Robstride 电机

## 线程模型

- 主线程：进程生命周期控制
- `cmd_thread_`：命令 accept + 读写
- `state_thread_`：状态推流 accept + 20Hz send
- `motion_thread_`：运行中的运动任务

## 核心边界

- `TwinAgent`：系统编排层
- `MotorIO`：关节空间抽象层
- `RobstrideController`：电机/CAN 编码层
- `CANInterface`：Linux SocketCAN 适配层

## 阅读顺序

1. [main.cpp](main.md)
2. [twin_agent.cpp / .hpp](twin_agent.md)
3. [motor_io.cpp / .hpp](motor_io.md)
4. [robstride.cpp / .hpp](robstride.md)
5. [can_interface.cpp / .hpp](can_interface.md)
6. [twin_protocol.hpp](twin_protocol.md)

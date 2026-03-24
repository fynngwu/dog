# TwinAgent + 本地滑杆/MuJoCo 控制台

## 1. Jetson 端
把下面这些文件放到同一目录并与现有 `can_interface.* / robstride.* / motor_io.*` 放在一起：

- `main.cpp`
- `twin_agent.hpp`
- `twin_agent.cpp`
- `twin_protocol.hpp`
- `CMakeLists.txt`

### 构建
```bash
mkdir -p build
cd build
cmake ..
make -j
```

### 运行
```bash
./twin_agent --cmd-port 47001 --state-port 47002
```

### 命令
- `ping`
- `get_state`
- `enable`
- `disable`
- `init 2.5`  -> 缓慢回到 offset 并 hold
- `hold`
- `set_joint <12 floats>`

## 2. 本地 PC 端
需要：
```bash
pip install numpy mujoco
```

### 一键启动
```bash
bash launch_twin_local.sh <JETSON_IP> <YOUR_MODEL.xml>
```

例如：
```bash
bash launch_twin_local.sh 192.168.50.20 /home/user/dog_scene.xml --auto-init-seconds 2.5
```

## 3. 说明
- 滑杆值含义：**关节相对 offset 的目标角**，单位 rad。
- 真实狗：通过 `set_joint` 发给 Jetson，Jetson 内部会用你现在的 HIPA/HIPF 方向映射和 knee ratio 下发到电机。
- 仿真狗：本地 MuJoCo viewer 里把 `qpos[qpos_start:qpos_start+12] = ref_qpos + slider_target`。
- 界面实时显示 `real / sim / diff(real-sim)`。
- `init` 命令已经实现为：`EnableAll -> MoveToOffset(duration) -> HoldOffsets`。

# 四足机器人仿真与部署项目

基于 MuJoCo 的四足机器人 Sim-to-Real 项目，使用 ONNX 策略进行控制。

## 目录结构

```
dog/
├── leggedrobot_flat.xml      # MuJoCo 机器人模型文件
├── policy.onnx               # 训练好的策略模型 (Isaac Lab -> ONNX)
├── main.cpp                  # 真实机器人部署主程序
├── observations.cpp          # 观测值计算 (C++)
├── observations.hpp          # 观测值计算头文件
├── deploy_robot              # 编译后的可执行文件
│
├── stubs/                    # 【占位符/桩代码】实际硬件 SDK 需替换
│   ├── Makefile
│   ├── robstride.hpp/.cpp   # Robstride 电机驱动 (stub)
│   ├── serial.h/.c           # 串口通信 (stub)
│   ├── wit_c_sdk.h/.c        # WIT IMU 传感器 SDK (stub)
│   └── tensorrt_inference.hpp # TensorRT 推理引擎 (stub)
│
├── debug_utils/              # 调试工具
│   ├── data_logger.hpp       # C++ 日志记录模块
│   ├── visualize_log.py       # Python 可视化脚本
│   ├── deploy_log.bin        # 运行后生成的日志文件
│   └── README.md
│
├── simulation/               # Python 仿真相关代码
│   ├── s2s_trot_joystick.py  # 主仿真脚本 (摇杆控制)
│   ├── s2s_data_collector.py # 数据收集与可视化
│   └── ...
│
├── mujoco-3.6.0/             # MuJoCo 库
└── .venv/                    # Python 虚拟环境
```

## 环境配置

### 1. 安装 uv (Python 包管理器)

uv 是一个极速的 Python 包管理器，比 pip 快 10-100 倍。

**Linux (国内镜像加速):**

```bash
# 方法一：使用国内镜像安装 (推荐)
curl -LsSf https://astral.org.cn/uv/install.sh | sh

# 方法二：使用 cnrio 镜像
curl -LsSf https://cnrio.cn/install.sh | sh

# 添加环境变量 (如果未自动添加)
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# 验证安装
uv --version
```

**配置全局镜像加速 (可选但推荐):**

```bash
# 设置 PyPI 镜像源 (清华大学源)
echo 'export UV_DEFAULT_INDEX="https://pypi.tuna.tsinghua.edu.cn/simple"' >> ~/.bashrc
source ~/.bashrc
```

### 2. 创建虚拟环境并安装依赖

```bash
# 进入项目目录
cd /home/wufy/project2026/dog

# 创建虚拟环境 (使用系统已安装的 Python，带 system-site-packages)
# 这样可以使用系统已安装的包，减少重复下载
uv venv --system-site-packages

# 激活虚拟环境
source .venv/bin/activate

# 安装项目依赖 (会自动读取 pyproject.toml)
uv sync
```

**如果系统没有安装所需 Python 版本:**

```bash
# uv 可以自动安装 Python
uv python install 3.12

# 然后创建虚拟环境
uv venv --python 3.12 --system-site-packages
source .venv/bin/activate
uv sync
```

### 3. 验证安装

```bash
# 激活环境后测试
python -c "import mujoco; print(f'MuJoCo version: {mujoco.__version__}')"
python -c "import onnxruntime as ort; print(f'ONNX Runtime: {ort.__version__}')"
```

### 4. 常用 uv 命令

```bash
# 添加新依赖
uv add package-name

# 添加开发依赖
uv add --dev package-name

# 移除依赖
uv remove package-name

# 同步依赖 (根据 pyproject.toml)
uv sync

# 运行脚本
uv run python simulation/view_robot.py
```

### 5. C++ 编译环境 (真实部署)

```bash
# 安装编译工具
sudo apt update && sudo apt install -y build-essential g++

# 编译 C++ 代码
cd dog/stubs
make clean && make

# 可执行文件会生成在 dog/deploy_robot
```

**注意**: `stubs/` 目录下的代码是占位符，实际部署时需要替换为真实的 SDK。

## 调试工具

### 数据记录与可视化

```bash
# 1. 运行部署程序（会自动记录数据）
./deploy_robot

# 2. 可视化日志
cd debug_utils
python visualize_log.py --stats-only   # 只看统计量
python visualize_log.py                 # 显示图表
```

详见 [debug_utils/README.md](debug_utils/README.md)

## 使用方法

### 1. 查看机器人模型

```bash
cd /home/wufy/project2026/dog
source .venv/bin/activate
python simulation/view_robot.py
```

启动 MuJoCo viewer 查看机器人模型。

### 2. 运行仿真 (摇杆控制)

```bash
source .venv/bin/activate
python simulation/s2s_trot_joystick.py --load_model policy.onnx
```

**控制方式：**
- 需要连接 Linux 兼容的游戏手柄 (`/dev/input/js0`)
- 左摇杆：控制前后左右移动
- 右摇杆：控制转向

**按键退出：** 关闭 viewer 窗口或按 `Esc`

### 3. 收集数据并可视化

```bash
source .venv/bin/activate
python simulation/s2s_data_collector.py --load_model policy.onnx --max_samples 5000
```

**参数：**
- `--load_model`: ONNX 模型路径 (必需)
- `--max_samples`: 最大收集样本数 (默认 5000)
- `--no_plot`: 收集完成后不显示图表

**输出：**
- `simulation/collected_data.npz`: 收集的数据
- `simulation/data_plot.png`: 主数据图表
- `simulation/action_plot.png`: 动作分析图表

### 4. 绘制已收集的数据

```bash
source .venv/bin/activate
python simulation/plot_collected_data.py --data simulation/collected_data.npz
```

**参数：**
- `--data`: 数据文件路径
- `--save`: 保存图表到文件
- `--no_show`: 不显示图表窗口

### 5. 分析/修复 XML 模型

```bash
# 分析 geom-inertial 对应性
python simulation/analyze_detailed.py

# 修复 geom 位置
python simulation/fix_xml_geom.py
```

### 6. 测试 ONNX 模型

```bash
python simulation/test_onnx.py --load_model policy.onnx
```

## 策略输入输出

### 输入观测 (单帧 45 维，堆叠 10 帧 = 450 维)

| 分量 | 维度 | 说明 |
|-----|-----|-----|
| `omega` | 3 | 基座角速度 (body frame) |
| `projected_gravity` | 3 | 重力在基座坐标系投影 |
| `commands` | 3 | 速度命令 (vx, vy, yaw_rate) |
| `joint_pos` | 12 | 关节位置相对默认位置偏移 |
| `joint_vel` | 12 | 关节速度 |
| `last_action` | 12 | 上一帧策略输出动作 |

### 输出

| 分量 | 维度 | 说明 |
|-----|-----|-----|
| `action` | 12 | 关节位置偏移量 |

**目标位置计算：**
```python
target_q = action * action_scale + default_dof_pos
# action_scale = 0.25
```

### 关节顺序映射

**MuJoCo 顺序 (Sim):**
```
[LF_A, LF_F, LF_K, LR_A, LR_F, LR_K, RF_A, RF_F, RF_K, RR_A, RR_F, RR_K]
```

**策略顺序 (Policy):**
```
[LF_A, LR_A, RF_A, RR_A, LF_F, LR_F, RF_F, RR_F, LF_K, LR_K, RF_K, RR_K]
```

代码中通过 `sim2policy_indices` 和 `policy2sim_indices` 进行映射。

## 控制参数

| 参数 | 值 | 说明 |
|-----|-----|-----|
| `dt` | 0.005s | 仿真时间步长 |
| `decimation` | 4 | 控制频率降频 (50Hz 控制) |
| `kp` | 25.0 | PD 控制比例增益 |
| `kd` | 0.5 | PD 控制微分增益 |
| `action_scale` | 0.25 | 动作缩放因子 |

## 数据收集内容

| 数据 | 维度 | 说明 |
|-----|-----|-----|
| `timestamps` | 1 | 时间戳 |
| `omega` | 3 | 角速度 |
| `gravity` | 3 | 重力投影 |
| `commands` | 3 | 控制命令 |
| `joint_pos` | 12 | 关节位置 |
| `joint_vel` | 12 | 关节速度 |
| `action` | 12 | 策略输出 |
| `target_q` | 12 | 目标关节位置 |
| `tau` | 12 | 关节扭矩 |
| `base_pos` | 3 | 基座位置 |
| `base_vel` | 3 | 基座速度 |
| `foot_contact` | 4 | 足端接触状态 |

## 文件详细说明

### `leggedrobot_flat.xml`
MuJoCo 机器人模型文件，定义了：
- 机器人连杆和关节
- 惯性参数
- 碰撞几何体
- 执行器和传感器

### `policy.onnx`
从 Isaac Lab 训练导出的策略模型，输入 450 维观测，输出 12 维动作。

### `s2s_trot_joystick.py`
主仿真脚本，实现：
- 加载 MuJoCo 模型和 ONNX 策略
- 摇杆控制接口
- 观测值构造和动作执行
- PD 控制器

### `s2s_data_collector.py`
数据收集脚本，在仿真基础上增加：
- 数据记录功能
- 自动保存和绘图

### `joystick_interface.py`
Linux 游戏手柄接口，支持：
- 读取 `/dev/input/js0` 设备
- 线性/角速度命令映射

### `observations.cpp` / `observations.hpp`
C++ 实现的观测值计算，用于真实机器人部署。

### `main.cpp`
真实机器人部署的主程序框架。

## 常见问题

### Q: uv 安装后找不到命令？
```bash
# 检查 PATH
echo $PATH

# 手动添加
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

### Q: 仿真中机器人不稳定？
检查：
1. XML 中 geom 和 inertial 位置是否匹配
2. PD 参数是否合适
3. 策略模型是否正确加载

### Q: 找不到摇杆设备？
```bash
# 检查设备是否存在
ls /dev/input/js*

# 添加权限
sudo chmod 666 /dev/input/js0
```

### Q: ONNX 加载失败？
确保 ONNX Runtime 已安装：
```bash
uv add onnxruntime
```

### Q: 依赖安装很慢？
确保已配置国内镜像源：
```bash
export UV_DEFAULT_INDEX="https://pypi.tuna.tsinghua.edu.cn/simple"
```

## 国内镜像源列表

| 镜像源 | 地址 |
|-------|------|
| 清华大学 | https://pypi.tuna.tsinghua.edu.cn/simple |
| 阿里云 | https://mirrors.aliyun.com/pypi/simple |
| 华为云 | https://mirrors.huaweicloud.com/repository/pypi/simple |
| 腾讯云 | https://mirrors.cloud.tencent.com/pypi/simple |

## 在另一台电脑上测试

### 1. 克隆仓库

```bash
git clone <your-repo-url>
cd dog
```

### 2. 安装 uv 并配置环境

```bash
# 安装 uv
curl -LsSf https://astral.org.cn/uv/install.sh | sh
source ~/.bashrc

# 创建虚拟环境并安装依赖
uv venv --system-site-packages
source .venv/bin/activate
uv sync
```

### 3. 运行仿真测试

#### 方式一：手柄控制（需要游戏手柄）

```bash
# 激活环境
source .venv/bin/activate

# 运行仿真
python simulation/s2s_trot_joystick.py --load_model policy.onnx
```

#### 方式二：ROS2 控制（需要 ROS2 环境）

```bash
# 先 source ROS2 环境
source /opt/ros/humble/setup.bash

# 激活项目环境
source .venv/bin/activate

# 运行仿真
python simulation/s2s_trot_joystick.py --load_model policy.onnx --ros2

# 在另一个终端发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

#### 方式三：自动化测试（无需手柄/ROS2）

```bash
source .venv/bin/activate
python simulation/auto_test.py
```

自动化测试会执行以下序列：
- 静止 1s → 前进 2s → 后退 2s → 左移 1s → 右移 1s → 左转 1s → 右转 1s → 静止 1s

测试完成后会生成：
- `simulation/auto_test_data.npz` - 数据文件
- `simulation/auto_test_plot.png` - 图表

### 4. 查看测试数据

```bash
source .venv/bin/activate

# 打印数据摘要
python simulation/view_test_data.py --no_plot

# 显示图表
python simulation/view_test_data.py
```

### 5. 数据收集与对比

```bash
# 收集仿真数据（手柄控制）
python simulation/data_recorder.py --load_model policy.onnx --max_samples 5000 --output sim_data.npz

# 收集仿真数据（ROS2 控制）
python simulation/data_recorder.py --load_model policy.onnx --ros2 --output sim_data.npz

# 查看仿真数据
python simulation/analyze_data.py --sim_data sim_data.npz --plot_sim

# 对比仿真与部署数据
python simulation/analyze_data.py --sim_data sim_data.npz --deploy_data /path/to/deploy_log.bin
```

---

## 策略使用说明

### 网络输入 (45维 × 10帧 = 450维)

| 分量 | 维度 | 说明 |
|------|------|------|
| angular_vel | 3 | 基座角速度 (body frame) |
| projected_gravity | 3 | 重力在基座坐标系投影 |
| commands | 3 | 速度命令 (vx, vy, yaw_rate) |
| joint_pos | 12 | 关节位置相对默认位置偏移 |
| joint_vel | 12 | 关节速度 |
| last_action | 12 | 上一帧策略输出动作 |

### 网络输出 (12维)

| 分量 | 维度 | 说明 |
|------|------|------|
| action | 12 | 关节位置偏移量 |

### 目标位置计算

```python
target_q = action * action_scale + default_dof_pos
# action_scale = 0.25
```

### 关节顺序 (Policy 顺序)

```
[LF_HipA, LR_HipA, RF_HipA, RR_HipA,  # HipA: 0-3
 LF_HipF, LR_HipF, RF_HipF, RR_HipF,  # HipF: 4-7
 LF_Knee, LR_Knee, RF_Knee, RR_Knee]  # Knee: 8-11
```

### 控制参数

| 参数 | 值 | 说明 |
|------|-----|------|
| dt | 0.005s | 仿真时间步长 |
| decimation | 4 | 控制频率降频 (50Hz 控制) |
| kp | 25.0 | PD 控制比例增益 |
| kd | 0.5 | PD 控制微分增益 |
| action_scale | 0.25 | 动作缩放因子 |

---

## 调试指南

详见 [simulation/DEBUG_GUIDE.md](simulation/DEBUG_GUIDE.md)

### 关键检查点

1. **关节顺序映射** - Policy 顺序与 MuJoCo 顺序不同，需要正确映射
2. **膝关节减速比** - 观测中除以减速比，输出中乘以减速比
3. **IMU 坐标系** - 确认 IMU 安装方向和坐标系转换
4. **action_scale** - 确保仿真与部署一致 (0.25)

---

## License

SPDX-License-Identifier: BSD-3-Clause
#!/usr/bin/env python3
"""加载MuJoCo模型并启动viewer"""

from pathlib import Path
import mujoco
import mujoco.viewer

# 加载修正后的模型
model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent.parent / "leggedrobot_flat.xml"))
data = mujoco.MjData(model)

print("模型加载成功!")
print(f"自由度: {model.nv}")
print(f"关节数: {model.njnt}")
print(f"Body数: {model.nbody}")

# 启动viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
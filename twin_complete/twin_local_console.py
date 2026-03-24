#!/usr/bin/env python3
"""本地一键双端控制台：滑杆 + Jetson 真机 + MuJoCo viewer。"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import messagebox, ttk
from typing import List, Optional

try:
    import numpy as np
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"需要 numpy: {exc}")

try:
    import mujoco
    import mujoco.viewer
    HAS_MUJOCO = True
except Exception:
    HAS_MUJOCO = False


JOINT_NAMES = [
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
]

JOINT_LIMITS = [
    (-0.80, 0.80), (-0.80, 0.80), (-0.80, 0.80), (-0.80, 0.80),
    (-1.25, 0.95), (-1.25, 0.95), (-0.95, 1.25), (-0.95, 1.25),
    (-0.75, 0.75), (-0.75, 0.75), (-0.75, 0.75), (-0.75, 0.75),
]


@dataclass
class RobotState:
    ok: bool = False
    mode: str = ""
    seq: int = 0
    motors_online: int = 0
    feedback_age_ms: int = -1
    joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    joint_velocities: List[float] = field(default_factory=lambda: [0.0] * 12)
    motor_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    motor_velocities: List[float] = field(default_factory=lambda: [0.0] * 12)
    motor_torques: List[float] = field(default_factory=lambda: [0.0] * 12)
    target_joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)

    @classmethod
    def from_json(cls, text: str) -> "RobotState":
        data = json.loads(text)
        return cls(
            ok=data.get("ok", False),
            mode=data.get("mode", ""),
            seq=data.get("seq", 0),
            motors_online=data.get("motors_online", 0),
            feedback_age_ms=data.get("feedback_age_ms", -1),
            joint_positions=data.get("joint_positions", [0.0] * 12),
            joint_velocities=data.get("joint_velocities", [0.0] * 12),
            motor_positions=data.get("motor_positions", [0.0] * 12),
            motor_velocities=data.get("motor_velocities", [0.0] * 12),
            motor_torques=data.get("motor_torques", [0.0] * 12),
            target_joint_positions=data.get("target_joint_positions", [0.0] * 12),
        )


class TwinRemoteClient:
    def __init__(self, host: str, cmd_port: int, state_port: int):
        self.host = host
        self.cmd_port = cmd_port
        self.state_port = state_port
        self.cmd_sock: Optional[socket.socket] = None
        self.state_sock: Optional[socket.socket] = None
        self.cmd_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.latest_state: Optional[RobotState] = None
        self.last_error: str = ""
        self.running = False
        self.state_thread: Optional[threading.Thread] = None

    def connect(self) -> None:
        self.cmd_sock = socket.create_connection((self.host, self.cmd_port), timeout=3.0)
        self.cmd_sock.settimeout(3.0)
        self.state_sock = socket.create_connection((self.host, self.state_port), timeout=3.0)
        self.state_sock.settimeout(1.0)
        self.running = True
        self.state_thread = threading.Thread(target=self._state_loop, daemon=True)
        self.state_thread.start()

    def close(self) -> None:
        self.running = False
        for sock in (self.cmd_sock, self.state_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self.cmd_sock = None
        self.state_sock = None

    def _state_loop(self) -> None:
        assert self.state_sock is not None
        buffer = b""
        while self.running:
            try:
                chunk = self.state_sock.recv(4096)
                if not chunk:
                    self.last_error = "状态连接关闭"
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        state = RobotState.from_json(line.decode("utf-8"))
                    except Exception as exc:
                        self.last_error = f"状态包解析失败: {exc}"
                        continue
                    with self.state_lock:
                        self.latest_state = state
            except socket.timeout:
                continue
            except Exception as exc:
                if self.running:
                    self.last_error = f"状态线程错误: {exc}"
                break
        self.running = False

    def send_command(self, cmd: str) -> dict:
        if self.cmd_sock is None:
            raise RuntimeError("命令连接未建立")
        payload = (cmd.strip() + "\n").encode("utf-8")
        with self.cmd_lock:
            self.cmd_sock.sendall(payload)
            buf = b""
            while b"\n" not in buf:
                chunk = self.cmd_sock.recv(4096)
                if not chunk:
                    raise RuntimeError("命令连接关闭")
                buf += chunk
        reply = json.loads(buf.split(b"\n", 1)[0].decode("utf-8"))
        if not reply.get("ok", False):
            self.last_error = reply.get("error", "未知错误")
        return reply

    def set_joint(self, values: List[float]) -> dict:
        cmd = "set_joint " + " ".join(f"{v:.6f}" for v in values)
        return self.send_command(cmd)

    def get_state(self) -> Optional[RobotState]:
        with self.state_lock:
            return self.latest_state


class SimRunner:
    def __init__(self, model_path: str, qpos_start: int, hz: float = 60.0, alpha: float = 0.18):
        self.model_path = model_path
        self.qpos_start = qpos_start
        self.hz = hz
        self.alpha = alpha
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.target = np.zeros(12, dtype=float)
        self.sim_joint = np.zeros(12, dtype=float)
        self.last_error = ""

    def start(self) -> None:
        if not HAS_MUJOCO:
            raise RuntimeError("未安装 mujoco Python 包")
        if not Path(self.model_path).exists():
            raise FileNotFoundError(f"MuJoCo 模型不存在: {self.model_path}")
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def set_target(self, values: List[float]) -> None:
        with self.lock:
            self.target[:] = np.array(values, dtype=float)

    def get_joint(self) -> List[float]:
        with self.lock:
            return self.sim_joint.tolist()

    def _loop(self) -> None:
        try:
            model = mujoco.MjModel.from_xml_path(self.model_path)
            data = mujoco.MjData(model)
            ref_qpos = np.array(data.qpos.copy(), dtype=float)
            end = self.qpos_start + 12
            if end > model.nq:
                raise RuntimeError(
                    f"qpos 索引越界: start={self.qpos_start}, end={end}, model.nq={model.nq}"
                )

            with mujoco.viewer.launch_passive(model, data) as viewer:
                dt = 1.0 / max(1.0, self.hz)
                while self.running and viewer.is_running():
                    with self.lock:
                        tgt = self.target.copy()
                        self.sim_joint[:] = self.sim_joint + self.alpha * (tgt - self.sim_joint)
                        q = ref_qpos.copy()
                        q[self.qpos_start:end] = ref_qpos[self.qpos_start:end] + self.sim_joint
                    data.qpos[:] = q
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    time.sleep(dt)
        except Exception as exc:
            self.last_error = str(exc)
        finally:
            self.running = False


class TwinConsoleApp:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.remote = TwinRemoteClient(args.host, args.cmd_port, args.state_port)
        self.sim = SimRunner(args.model, args.sim_qpos_start, hz=args.sim_hz, alpha=args.sim_alpha)
        self.running = True
        self.target_lock = threading.Lock()
        self.desired_targets = [0.0] * 12
        self.last_sent_targets: Optional[List[float]] = None
        self.last_send_time = 0.0

        self.root = tk.Tk()
        self.root.title("Twin Local Console")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status_var = tk.StringVar(value="未连接")
        self.mode_var = tk.StringVar(value="mode: ?")
        self.error_var = tk.StringVar(value="")
        self.info_vars = []
        self.real_vars = []
        self.sim_vars = []
        self.diff_vars = []
        self.slider_vars = []

        self._build_ui()

        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, textvariable=self.status_var, width=40).grid(row=0, column=0, sticky="w")
        ttk.Label(top, textvariable=self.mode_var, width=25).grid(row=0, column=1, sticky="w")
        ttk.Label(top, textvariable=self.error_var, foreground="red").grid(row=1, column=0, columnspan=4, sticky="w")

        btns = ttk.Frame(self.root, padding=8)
        btns.pack(fill=tk.X)
        ttk.Button(btns, text="连接并启动", command=self.connect_all).grid(row=0, column=0, padx=2)
        ttk.Button(btns, text="初始化到 Offset", command=self.init_to_offset).grid(row=0, column=1, padx=2)
        ttk.Button(btns, text="Enable", command=lambda: self.send_simple("enable")).grid(row=0, column=2, padx=2)
        ttk.Button(btns, text="Hold Offset", command=lambda: self.send_simple("hold")).grid(row=0, column=3, padx=2)
        ttk.Button(btns, text="Disable", command=lambda: self.send_simple("disable")).grid(row=0, column=4, padx=2)
        ttk.Button(btns, text="滑杆清零", command=self.zero_sliders).grid(row=0, column=5, padx=2)
        ttk.Button(btns, text="滑杆同步真实狗", command=self.copy_real_to_sliders).grid(row=0, column=6, padx=2)

        body = ttk.Frame(self.root, padding=8)
        body.pack(fill=tk.BOTH, expand=True)

        ttk.Label(body, text="Joint").grid(row=0, column=0, sticky="w")
        ttk.Label(body, text="Slider Target(rad)").grid(row=0, column=1, sticky="w")
        ttk.Label(body, text="Real(rad)").grid(row=0, column=2, sticky="w")
        ttk.Label(body, text="Sim(rad)").grid(row=0, column=3, sticky="w")
        ttk.Label(body, text="Diff(real-sim)").grid(row=0, column=4, sticky="w")

        for i, name in enumerate(JOINT_NAMES):
            ttk.Label(body, text=name, width=10).grid(row=i + 1, column=0, sticky="w")
            var = tk.DoubleVar(value=0.0)
            self.slider_vars.append(var)
            scale = tk.Scale(
                body,
                variable=var,
                from_=JOINT_LIMITS[i][1],
                to=JOINT_LIMITS[i][0],
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=320,
                command=lambda _v, idx=i, v=var: self._on_slider(idx, v.get()),
            )
            scale.grid(row=i + 1, column=1, sticky="we")

            real_var = tk.StringVar(value="0.000")
            sim_var = tk.StringVar(value="0.000")
            diff_var = tk.StringVar(value="0.000")
            self.real_vars.append(real_var)
            self.sim_vars.append(sim_var)
            self.diff_vars.append(diff_var)
            ttk.Label(body, textvariable=real_var, width=10).grid(row=i + 1, column=2, sticky="w")
            ttk.Label(body, textvariable=sim_var, width=10).grid(row=i + 1, column=3, sticky="w")
            ttk.Label(body, textvariable=diff_var, width=10).grid(row=i + 1, column=4, sticky="w")

        body.columnconfigure(1, weight=1)

    def _on_slider(self, idx: int, value: float) -> None:
        with self.target_lock:
            self.desired_targets[idx] = float(value)

    def zero_sliders(self) -> None:
        for var in self.slider_vars:
            var.set(0.0)
        with self.target_lock:
            self.desired_targets = [0.0] * 12

    def copy_real_to_sliders(self) -> None:
        state = self.remote.get_state()
        if state is None:
            return
        for i in range(12):
            self.slider_vars[i].set(state.joint_positions[i])
        with self.target_lock:
            self.desired_targets = list(state.joint_positions)

    def connect_all(self) -> None:
        try:
            self.remote.connect()
            self.sim.start()
            self.control_thread.start()
            self.status_var.set(f"已连接: {self.args.host} | cmd={self.args.cmd_port} state={self.args.state_port}")
            if self.args.auto_init_seconds > 0.0:
                reply = self.remote.send_command(f"init {self.args.auto_init_seconds:.3f}")
                self.error_var.set(reply.get("msg", ""))
        except RuntimeError as exc:
            messagebox.showerror("启动失败", str(exc))
        except Exception as exc:
            messagebox.showerror("启动失败", str(exc))

    def init_to_offset(self) -> None:
        try:
            reply = self.remote.send_command(f"init {self.args.init_seconds:.3f}")
            self.error_var.set(reply.get("msg", ""))
            self.zero_sliders()
        except Exception as exc:
            self.error_var.set(str(exc))

    def send_simple(self, cmd: str) -> None:
        try:
            reply = self.remote.send_command(cmd)
            self.error_var.set(reply.get("msg", reply.get("error", "")))
        except Exception as exc:
            self.error_var.set(str(exc))

    def _control_loop(self) -> None:
        period = 1.0 / max(1.0, self.args.control_hz)
        while self.running:
            with self.target_lock:
                desired = list(self.desired_targets)
            self.sim.set_target(desired)

            now = time.time()
            need_send = (
                self.last_sent_targets is None
                or any(abs(a - b) > 1e-4 for a, b in zip(desired, self.last_sent_targets))
                or (now - self.last_send_time) >= 0.2
            )
            if need_send and self.remote.running:
                try:
                    reply = self.remote.set_joint(desired)
                    if not reply.get("ok", False):
                        self.error_var.set(reply.get("error", "set_joint failed"))
                    self.last_sent_targets = desired
                    self.last_send_time = now
                except Exception as exc:
                    self.error_var.set(str(exc))
            time.sleep(period)

    def _refresh_ui(self) -> None:
        state = self.remote.get_state()
        sim_joint = self.sim.get_joint() if self.sim.running else [0.0] * 12

        if state is not None:
            self.mode_var.set(
                f"mode={state.mode} online={state.motors_online}/12 age={state.feedback_age_ms}ms seq={state.seq}"
            )
            real_joint = state.joint_positions
            for i in range(12):
                self.real_vars[i].set(f"{real_joint[i]:+.3f}")
                self.sim_vars[i].set(f"{sim_joint[i]:+.3f}")
                self.diff_vars[i].set(f"{(real_joint[i] - sim_joint[i]):+.3f}")
        else:
            for i in range(12):
                self.real_vars[i].set("---")
                self.sim_vars[i].set(f"{sim_joint[i]:+.3f}")
                self.diff_vars[i].set("---")

        err_parts = []
        if self.remote.last_error:
            err_parts.append(f"remote: {self.remote.last_error}")
        if self.sim.last_error:
            err_parts.append(f"sim: {self.sim.last_error}")
        self.error_var.set(" | ".join(err_parts))

        if self.running:
            self.root.after(80, self._refresh_ui)

    def on_close(self) -> None:
        self.running = False
        self.remote.close()
        self.sim.stop()
        self.root.destroy()

    def run(self) -> None:
        self.root.after(80, self._refresh_ui)
        self.root.mainloop()


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="本地一键双端控制台")
    ap.add_argument("--host", default="100.109.192.69", help="Jetson 上 twin_agent 的 IP")
    ap.add_argument("--cmd-port", type=int, default=47001)
    ap.add_argument("--state-port", type=int, default=47002)
    ap.add_argument("--model", default="/home/wufy/github_respository/dog_project/dog/deploy_robot_v2/twin_panel/local/robot_fixed.xml", help="本地 MuJoCo XML/MJCF 路径")
    ap.add_argument("--sim-qpos-start", type=int, default=0, help="12 个关节在 qpos 里的起始索引")
    ap.add_argument("--control-hz", type=float, default=20.0)
    ap.add_argument("--sim-hz", type=float, default=60.0)
    ap.add_argument("--sim-alpha", type=float, default=0.18, help="仿真关节向滑杆目标靠拢的平滑系数")
    ap.add_argument("--init-seconds", type=float, default=2.5, help="点击初始化按钮时的缓动秒数")
    ap.add_argument("--auto-init-seconds", type=float, default=0.0, help="启动后自动执行 init 的秒数，0 表示不自动")
    return ap


def main() -> int:
    if not HAS_MUJOCO:
        raise SystemExit("请先安装 mujoco: pip install mujoco numpy")
    args = build_argparser().parse_args()
    app = TwinConsoleApp(args)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

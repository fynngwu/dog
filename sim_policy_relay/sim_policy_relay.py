from __future__ import annotations

import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "simulation"))

from joystick_input import OptionalGamepad
from relay_engine import EngineConfig, JOINT_NAMES, SimPolicyRelayEngine

try:
    from ros2_interface import ROS2_AVAILABLE
    if not ROS2_AVAILABLE:
        raise ImportError
    from ros2_interface import ROS2Interface
except (ImportError, Exception):
    ROS2_AVAILABLE = False
    ROS2Interface = None  # type: ignore[assignment,misc]


class SimPolicyRelayApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("sim_policy_relay")
        self.root.geometry("1220x760")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.engine = SimPolicyRelayEngine()
        self.gamepad = OptionalGamepad()
        self.gamepad.start()

        self.ros2 = None
        self.ros2_last_active = 0.0
        self.ros2_deadman_ms = 200

        self.host_var = tk.StringVar(value="127.0.0.1")
        self.cmd_port_var = tk.StringVar(value="47001")
        self.state_port_var = tk.StringVar(value="47002")
        self.xml_path_var = tk.StringVar(value="")
        self.onnx_path_var = tk.StringVar(value="")
        self.viewer_var = tk.BooleanVar(value=True)
        self.sim_only_var = tk.BooleanVar(value=False)
        self.use_gamepad_var = tk.BooleanVar(value=False)
        self.use_ros2_var = tk.BooleanVar(value=False)
        self.auto_stop_var = tk.BooleanVar(value=False)
        self.div_thresh_var = tk.StringVar(value="0.35")

        self.cmd_x_var = tk.DoubleVar(value=0.0)
        self.cmd_y_var = tk.DoubleVar(value=0.0)
        self.cmd_yaw_var = tk.DoubleVar(value=0.0)

        self.status_line_var = tk.StringVar(value="Disconnected")
        self.stats_line_var = tk.StringVar(value="motors_online=0 | feedback_age_ms=-1 | policy_fps=0.0 | sim_fps=0.0")
        self.error_line_var = tk.StringVar(value="")

        self.button_refs = {}
        self.table_labels = []

        self._build_ui()
        self._bind_keyboard()
        self._refresh_loop()

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)

        top = ttk.LabelFrame(outer, text="Connection settings", padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Jetson IP").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.host_entry = ttk.Entry(top, textvariable=self.host_var, width=18)
        self.host_entry.grid(row=0, column=1, sticky="we", padx=4, pady=4)
        ttk.Label(top, text="cmd_port").grid(row=0, column=2, sticky="w", padx=4, pady=4)
        self.cmd_port_entry = ttk.Entry(top, textvariable=self.cmd_port_var, width=8)
        self.cmd_port_entry.grid(row=0, column=3, sticky="we", padx=4, pady=4)
        ttk.Label(top, text="state_port").grid(row=0, column=4, sticky="w", padx=4, pady=4)
        self.state_port_entry = ttk.Entry(top, textvariable=self.state_port_var, width=8)
        self.state_port_entry.grid(row=0, column=5, sticky="we", padx=4, pady=4)
        ttk.Checkbutton(top, text="Sim Only (no robot)", variable=self.sim_only_var, command=self._on_sim_only_toggled).grid(row=0, column=6, sticky="w", padx=8)
        ttk.Checkbutton(top, text="Viewer", variable=self.viewer_var).grid(row=0, column=7, sticky="w", padx=8)

        ttk.Label(top, text="MuJoCo XML").grid(row=1, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(top, textvariable=self.xml_path_var, width=72).grid(row=1, column=1, columnspan=5, sticky="we", padx=4, pady=4)
        ttk.Button(top, text="Browse", command=self.pick_xml).grid(row=1, column=6, sticky="we", padx=4, pady=4)

        ttk.Label(top, text="ONNX Policy").grid(row=2, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(top, textvariable=self.onnx_path_var, width=72).grid(row=2, column=1, columnspan=5, sticky="we", padx=4, pady=4)
        ttk.Button(top, text="Browse", command=self.pick_onnx).grid(row=2, column=6, sticky="we", padx=4, pady=4)

        ttk.Label(top, text="Diff threshold (rad)").grid(row=3, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(top, textvariable=self.div_thresh_var, width=8).grid(row=3, column=1, sticky="w", padx=4, pady=4)
        ttk.Checkbutton(top, text="Auto-stop on divergence", variable=self.auto_stop_var).grid(row=3, column=2, columnspan=2, sticky="w", padx=4, pady=4)
        ttk.Checkbutton(top, text="Use gamepad if available", variable=self.use_gamepad_var).grid(row=3, column=4, sticky="w", padx=4, pady=4)
        ros2_label = "Use ROS2 cmd_vel" + ("" if ROS2_AVAILABLE else " (unavailable)")
        self.ros2_check = ttk.Checkbutton(top, text=ros2_label, variable=self.use_ros2_var, command=self._on_ros2_toggled, state=tk.NORMAL if ROS2_AVAILABLE else tk.DISABLED)
        self.ros2_check.grid(row=3, column=5, sticky="w", padx=4, pady=4)
        self.button_refs["connect"] = ttk.Button(top, text="Connect", command=self.on_connect)
        self.button_refs["connect"].grid(row=3, column=6, sticky="we", padx=4, pady=4)

        buttons = ttk.LabelFrame(outer, text="Control", padding=8)
        buttons.pack(fill=tk.X, pady=(8, 0))
        self.button_refs["init"] = ttk.Button(buttons, text="Init Robot", command=self.on_init)
        self.button_refs["start"] = ttk.Button(buttons, text="Start Policy", command=self.on_start_policy)
        self.button_refs["stop"] = ttk.Button(buttons, text="Stop Policy", command=self.on_stop_policy)
        self.button_refs["hold"] = ttk.Button(buttons, text="Hold", command=self.on_hold)
        self.button_refs["disable"] = ttk.Button(buttons, text="Disable", command=self.on_disable)
        self.button_refs["reset"] = ttk.Button(buttons, text="Reset Sim", command=self.on_reset_sim)

        self.button_refs["init"].grid(row=0, column=0, padx=4, pady=4)
        self.button_refs["start"].grid(row=0, column=1, padx=4, pady=4)
        self.button_refs["stop"].grid(row=0, column=2, padx=4, pady=4)
        self.button_refs["hold"].grid(row=0, column=3, padx=4, pady=4)
        self.button_refs["disable"].grid(row=0, column=4, padx=4, pady=4)
        self.button_refs["reset"].grid(row=0, column=5, padx=4, pady=4)
        self.button_refs["estop"] = tk.Button(buttons, text="E-Stop", bg="#c62828", fg="white", font=("TkDefaultFont", 11, "bold"), command=self.on_estop)
        self.button_refs["estop"].grid(row=0, column=6, padx=10, pady=4, ipadx=18, ipady=8)

        cmd_frame = ttk.LabelFrame(outer, text="Velocity commands fed into policy observation", padding=8)
        cmd_frame.pack(fill=tk.X, pady=(8, 0))
        self._make_cmd_slider(cmd_frame, 0, "cmd_x", self.cmd_x_var, -2.0, 2.0)
        self._make_cmd_slider(cmd_frame, 1, "cmd_y", self.cmd_y_var, -1.0, 1.0)
        self._make_cmd_slider(cmd_frame, 2, "cmd_yaw", self.cmd_yaw_var, -1.5, 1.5)
        ttk.Button(cmd_frame, text="Zero Command", command=self.zero_command).grid(row=0, column=3, rowspan=3, sticky="ns", padx=10)

        table_wrap = ttk.LabelFrame(outer, text="Joint monitor", padding=8)
        table_wrap.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        headers = ["Joint", "Real(rad)", "Sim(rad)", "Policy Action", "Diff(real-sim)"]
        for col, title in enumerate(headers):
            tk.Label(table_wrap, text=title, relief=tk.GROOVE, borderwidth=1, bg="#ececec", width=18).grid(row=0, column=col, sticky="nsew")

        for row, name in enumerate(JOINT_NAMES, start=1):
            row_widgets = []
            vals = [name, "0.0000", "0.0000", "0.0000", "0.0000"]
            widths = [18, 18, 18, 18, 18]
            for col, text in enumerate(vals):
                label = tk.Label(table_wrap, text=text, relief=tk.GROOVE, borderwidth=1, width=widths[col], bg="white")
                label.grid(row=row, column=col, sticky="nsew")
                row_widgets.append(label)
            self.table_labels.append(row_widgets)

        status = ttk.Frame(outer, padding=(0, 8, 0, 0))
        status.pack(fill=tk.X)
        ttk.Label(status, textvariable=self.status_line_var).pack(anchor="w")
        ttk.Label(status, textvariable=self.stats_line_var).pack(anchor="w")
        ttk.Label(status, textvariable=self.error_line_var, foreground="#b00020").pack(anchor="w")

        self._set_button_states(connected=False, initialized=False, policy_running=False)

    def _make_cmd_slider(self, parent, row, label_text, variable, min_v, max_v):
        ttk.Label(parent, text=label_text).grid(row=row, column=0, sticky="w", padx=4, pady=4)
        scale = tk.Scale(parent, variable=variable, from_=max_v, to=min_v, resolution=0.01, orient=tk.HORIZONTAL, length=520, command=self._on_command_changed)
        scale.grid(row=row, column=1, sticky="we", padx=4, pady=4)
        ttk.Entry(parent, textvariable=variable, width=8).grid(row=row, column=2, sticky="w", padx=4, pady=4)
        parent.columnconfigure(1, weight=1)

    def _bind_keyboard(self) -> None:
        self.root.bind("<w>", lambda _e: self._nudge(self.cmd_x_var, +0.05))
        self.root.bind("<s>", lambda _e: self._nudge(self.cmd_x_var, -0.05))
        self.root.bind("<a>", lambda _e: self._nudge(self.cmd_y_var, +0.05))
        self.root.bind("<d>", lambda _e: self._nudge(self.cmd_y_var, -0.05))
        self.root.bind("<q>", lambda _e: self._nudge(self.cmd_yaw_var, +0.05))
        self.root.bind("<e>", lambda _e: self._nudge(self.cmd_yaw_var, -0.05))
        self.root.bind("<space>", lambda _e: self.zero_command())

    def _nudge(self, var: tk.DoubleVar, delta: float) -> None:
        var.set(var.get() + delta)
        self._on_command_changed()

    def pick_xml(self) -> None:
        path = filedialog.askopenfilename(title="Select MuJoCo XML", filetypes=[("MuJoCo XML", "*.xml"), ("All files", "*")])
        if path:
            self.xml_path_var.set(path)

    def pick_onnx(self) -> None:
        path = filedialog.askopenfilename(title="Select ONNX policy", filetypes=[("ONNX", "*.onnx"), ("All files", "*")])
        if path:
            self.onnx_path_var.set(path)

    def _on_command_changed(self, *_args) -> None:
        self.engine.update_command(self.cmd_x_var.get(), self.cmd_y_var.get(), self.cmd_yaw_var.get())

    def _on_sim_only_toggled(self) -> None:
        sim_only = self.sim_only_var.get()
        state = tk.DISABLED if sim_only else tk.NORMAL
        self.host_entry.configure(state=state)
        self.cmd_port_entry.configure(state=state)
        self.state_port_entry.configure(state=state)
        if sim_only:
            self.viewer_var.set(True)

    def _on_ros2_toggled(self) -> None:
        if self.use_ros2_var.get():
            if self.ros2 is None and ROS2Interface is not None:
                try:
                    self.ros2 = ROS2Interface(max_v_x=2.0, max_v_y=1.0, max_omega=1.5)
                    self.ros2_last_active = time.monotonic()
                    self.error_line_var.set("ROS2 cmd_vel active")
                except Exception as exc:
                    messagebox.showerror("ROS2 failed", str(exc))
                    self.use_ros2_var.set(False)
            if self.use_gamepad_var.get():
                self.use_gamepad_var.set(False)
                self.gamepad.set_enabled(False)
        else:
            if self.ros2 is not None:
                self.ros2.stop()
                self.ros2 = None
            self.cmd_x_var.set(0.0)
            self.cmd_y_var.set(0.0)
            self.cmd_yaw_var.set(0.0)
            self._on_command_changed()

    def zero_command(self) -> None:
        self.cmd_x_var.set(0.0)
        self.cmd_y_var.set(0.0)
        self.cmd_yaw_var.set(0.0)
        self._on_command_changed()

    def on_connect(self) -> None:
        try:
            cfg = EngineConfig(
                sim_only=self.sim_only_var.get(),
                host=self.host_var.get().strip(),
                cmd_port=int(self.cmd_port_var.get()),
                state_port=int(self.state_port_var.get()),
                xml_path=self.xml_path_var.get().strip(),
                onnx_path=self.onnx_path_var.get().strip(),
                viewer_enabled=bool(self.viewer_var.get()),
                divergence_threshold_rad=float(self.div_thresh_var.get()),
                auto_stop_on_divergence=bool(self.auto_stop_var.get()),
            )
            self.engine.connect_and_load(cfg)
            self.engine.update_command(self.cmd_x_var.get(), self.cmd_y_var.get(), self.cmd_yaw_var.get())
            if cfg.sim_only:
                self.status_line_var.set("Connected (sim-only mode)")
            else:
                self.status_line_var.set(f"Connected to {cfg.host}:{cfg.cmd_port}/{cfg.state_port}")
            self.error_line_var.set("")
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))

    def on_init(self) -> None:
        try:
            reply = self.engine.init_robot(2.5)
            self.error_line_var.set(reply.get("msg", "Robot initialized"))
        except Exception as exc:
            messagebox.showerror("Init failed", str(exc))

    def on_start_policy(self) -> None:
        try:
            self.engine.update_safety_settings(float(self.div_thresh_var.get()), bool(self.auto_stop_var.get()))
            self.engine.start_policy()
        except Exception as exc:
            messagebox.showerror("Start failed", str(exc))

    def on_stop_policy(self) -> None:
        self.engine.stop_policy()

    def on_hold(self) -> None:
        try:
            reply = self.engine.hold()
            self.error_line_var.set(reply.get("msg", "hold"))
        except Exception as exc:
            messagebox.showerror("Hold failed", str(exc))

    def on_disable(self) -> None:
        try:
            reply = self.engine.disable()
            self.error_line_var.set(reply.get("msg", "disabled"))
        except Exception as exc:
            messagebox.showerror("Disable failed", str(exc))

    def on_estop(self) -> None:
        self.engine.emergency_stop("manual E-stop from GUI")

    def on_reset_sim(self) -> None:
        self.engine.reset_sim()

    def _set_button_states(self, connected: bool, initialized: bool, policy_running: bool) -> None:
        self.button_refs["connect"].configure(state=tk.DISABLED if connected else tk.NORMAL)
        self.button_refs["init"].configure(state=tk.NORMAL if connected else tk.DISABLED)
        self.button_refs["start"].configure(state=tk.NORMAL if (connected and initialized and not policy_running) else tk.DISABLED)
        self.button_refs["stop"].configure(state=tk.NORMAL if policy_running else tk.DISABLED)
        for key in ("hold", "disable", "reset"):
            self.button_refs[key].configure(state=tk.NORMAL if connected else tk.DISABLED)
        self.button_refs["estop"].configure(state=tk.NORMAL if connected else tk.DISABLED)

    def _refresh_loop(self) -> None:
        # ROS2 cmd_vel input (with deadman timeout)
        if self.use_ros2_var.get() and self.ros2 is not None:
            cx, cy, cyaw = self.ros2.get_command()
            if any(abs(v) > 1e-6 for v in (cx, cy, cyaw)):
                self.ros2_last_active = time.monotonic()
            elif (time.monotonic() - self.ros2_last_active) * 1000.0 > self.ros2_deadman_ms:
                cx, cy, cyaw = 0.0, 0.0, 0.0
            self.cmd_x_var.set(cx)
            self.cmd_y_var.set(cy)
            self.cmd_yaw_var.set(cyaw)
            self._on_command_changed()
        elif self.use_gamepad_var.get() and self.gamepad.available:
            gx, gy, gyaw = self.gamepad.get_command()
            self.cmd_x_var.set(gx)
            self.cmd_y_var.set(gy)
            self.cmd_yaw_var.set(gyaw)
            self._on_command_changed()

        snap = self.engine.get_snapshot()
        self._set_button_states(snap.connected, snap.initialized and not snap.estopped, snap.policy_running)

        self.status_line_var.set(
            f"status={snap.status_text} | connected={snap.connected} | initialized={snap.initialized} | estopped={snap.estopped} | viewer={snap.viewer_enabled}"
        )
        self.stats_line_var.set(
            f"motors_online={snap.motors_online}/12 | feedback_age_ms={snap.feedback_age_ms} | state_age_ms={snap.state_stream_age_ms:.0f} | policy_fps={snap.policy_fps:.1f} | sim_fps={snap.sim_fps:.1f}"
        )
        err = snap.last_error or snap.last_warning
        parts = []
        if self.use_ros2_var.get() and self.ros2 is not None:
            cx, cy, cyaw = self.ros2.get_command()
            parts.append(f"ros2_cmd=[{cx:+.2f},{cy:+.2f},{cyaw:+.2f}]")
        if self.gamepad.available:
            parts.append(f"gamepad_ready={self.gamepad.available}")
        if parts:
            err = (err + " | " if err else "") + " | ".join(parts)
        self.error_line_var.set(err)

        threshold = max(1e-6, float(self.div_thresh_var.get() or 0.35))
        for i, row in enumerate(self.table_labels):
            real_v = snap.real_joint_positions[i]
            sim_v = snap.sim_joint_positions[i]
            act_v = snap.scaled_action[i]
            diff_v = snap.diff_real_minus_sim[i]
            row[0].configure(text=JOINT_NAMES[i])
            row[1].configure(text=f"{real_v:+.4f}")
            row[2].configure(text=f"{sim_v:+.4f}")
            row[3].configure(text=f"{act_v:+.4f}")
            row[4].configure(text=f"{diff_v:+.4f}")

            abs_diff = abs(diff_v)
            if abs_diff < 0.5 * threshold:
                bg = "#e8f5e9"
            elif abs_diff < threshold:
                bg = "#fff9c4"
            else:
                bg = "#ffebee"
            for widget in row[1:]:
                widget.configure(bg=bg)

        self.root.after(100, self._refresh_loop)

    def on_close(self) -> None:
        try:
            self.engine.shutdown()
        finally:
            if self.ros2 is not None:
                self.ros2.stop()
            self.gamepad.stop()
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    # Helpful defaults for local testing in the unpacked zip directory.
    app = SimPolicyRelayApp()
    local_xml = Path(__file__).with_name("leggedrobot_flat.xml")
    if local_xml.exists() and not app.xml_path_var.get():
        app.xml_path_var.set(str(local_xml))
    local_onnx = Path(__file__).with_name("policy.onnx")
    if local_onnx.exists() and not app.onnx_path_var.get():
        app.onnx_path_var.set(str(local_onnx))
    app.run()

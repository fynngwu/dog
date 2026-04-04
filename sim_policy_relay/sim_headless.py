#!/usr/bin/env python3
"""
sim_headless.py — Run MuJoCo sim + ONNX policy with ROS2 cmd_vel input, no real robot.

Usage:
    python sim_headless.py [--viewer] [--no-viewer] [--xml path] [--onnx path]
                           [--cmd-scale 0.5] [--deadman-ms 200]

If ROS2 is not available, runs with zero command (policy will stand in place).
"""

import argparse
import signal
import sys
import time
from pathlib import Path

# Allow importing ros2_interface from the sibling simulation/ directory
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "simulation"))

from relay_engine import EngineConfig, SimPolicyRelayEngine

# Scaling convention matches deploy_robot_v2/v3 (linear.x * 0.5 → cmd_x)
DEFAULT_CMD_SCALE = 1.0
DEFAULT_DEADMAN_MS = 200
BRIDGE_HZ = 50


def main():
    parser = argparse.ArgumentParser(description="Sim-only mode with ROS2 cmd_vel")
    parser.add_argument("--xml", type=str, default="", help="MuJoCo XML path")
    parser.add_argument("--onnx", type=str, default="", help="ONNX policy path")
    parser.add_argument("--no-viewer", action="store_true", help="Disable MuJoCo viewer")
    parser.add_argument("--cmd-scale", type=float, default=DEFAULT_CMD_SCALE,
                        help="Command scaling factor (default: 0.5)")
    parser.add_argument("--deadman-ms", type=int, default=DEFAULT_DEADMAN_MS,
                        help="Deadman timeout in ms, zero cmd if no message (default: 200)")
    args = parser.parse_args()

    # Auto-detect paths next to script
    base = Path(__file__).resolve().parent
    xml_path = args.xml or str(base / "leggedrobot_flat.xml")
    onnx_path = args.onnx or str(base / "policy.onnx")

    # Import ROS2 interface (optional dependency)
    ros2 = None
    try:
        from ros2_interface import ROS2_AVAILABLE
        if not ROS2_AVAILABLE:
            raise ImportError
        from ros2_interface import ROS2Interface
        ros2 = ROS2Interface(max_v_x=2.0, max_v_y=1.0, max_omega=1.5)
        print("[ROS2] Subscribed to /cmd_vel")
    except (ImportError, Exception) as e:
        print(f"[WARN] ROS2 not available ({e}); running with zero command")

    # Create engine in sim-only mode
    engine = SimPolicyRelayEngine()
    cfg = EngineConfig(
        sim_only=True,
        xml_path=xml_path,
        onnx_path=onnx_path,
        viewer_enabled=not args.no_viewer,
    )
    engine.connect_and_load(cfg)
    engine.init_robot()
    engine.start_policy()
    viewer_label = "viewer ON" if cfg.viewer_enabled else "no viewer"
    print(f"[Engine] Sim-only mode started ({viewer_label})")

    running = True
    last_active = time.monotonic()

    def on_signal(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while running:
            t0 = time.monotonic()

            # Bridge ROS2 cmd_vel → engine command
            if ros2 is not None:
                cx, cy, cyaw = ros2.get_command()
                if any(abs(v) > 1e-6 for v in (cx, cy, cyaw)):
                    last_active = time.monotonic()
                elif (time.monotonic() - last_active) * 1000.0 > args.deadman_ms:
                    cx, cy, cyaw = 0.0, 0.0, 0.0
                engine.update_command(cx * args.cmd_scale, cy * args.cmd_scale, cyaw * args.cmd_scale)

            # Print status every ~1s
            snap = engine.get_snapshot()
            if snap.sim_fps > 0:
                print(
                    f"\r[sim] policy_fps={snap.policy_fps:.0f} sim_fps={snap.sim_fps:.0f} "
                    f"cmd=[{snap.cmd[0]:+.2f}, {snap.cmd[1]:+.2f}, {snap.cmd[2]:+.2f}]  ",
                    end="", flush=True,
                )

            elapsed = time.monotonic() - t0
            sleep_time = (1.0 / BRIDGE_HZ) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        print("\n[Engine] Shutting down...")
        engine.shutdown()
        if ros2 is not None:
            ros2.stop()
        print("[Engine] Done.")


if __name__ == "__main__":
    main()

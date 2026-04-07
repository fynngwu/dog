"""Virtual dog daemon for frontend testing.

Simulates the C++ twin_agent daemon with two TCP servers:
  - Command port (47001): request/reply, one JSON line per command
  - State port (47002): push stream, one JSON line every 50ms

Usage:
    python mock_daemon.py [--cmd-port 47001] [--state-port 47002]
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
from typing import Any, Dict, List, Optional


NUM_MOTORS = 12
DEFAULT_POS = [0.0] * NUM_MOTORS


def make_state(
    seq: int,
    mode: str = "disabled",
    joint_positions: Optional[List[float]] = None,
    motion_active: bool = False,
    motion_name: str = "",
    fault_active: bool = False,
    fault_code: int = 0,
    fault_msg: str = "",
    replay_loaded: bool = False,
    replay_cursor: int = 0,
    replay_total: int = 0,
    replay_status: str = "idle",
) -> Dict[str, Any]:
    pos = joint_positions or list(DEFAULT_POS)
    return {
        "ok": True,
        "mode": mode,
        "seq": seq,
        "state": {
            "joint_positions": pos,
            "joint_velocities": [0.0] * NUM_MOTORS,
            "joint_torques": [0.0] * NUM_MOTORS,
            "target_joint_positions": list(pos),
            "offline_motors": [],
        },
        "motion": {
            "active": motion_active,
            "name": motion_name,
            "last_error": "",
        },
        "replay": {
            "loaded": replay_loaded,
            "csv_path": "/tmp/mock_replay.csv",
            "cursor": replay_cursor,
            "total_frames": replay_total,
            "speed_factor": 1.0,
            "status": replay_status,
        },
    }


def handle_command(cmd: str, shared: dict) -> Dict[str, Any]:
    """Process a single text command and return a JSON-serializable reply."""
    parts = cmd.strip().split()
    action = parts[0] if parts else ""

    if action == "ping":
        return {"ok": True, "msg": "pong"}

    elif action == "get_state":
        return make_state(
            seq=shared["seq"],
            mode=shared["mode"],
            joint_positions=list(shared["positions"]),
            motion_active=shared["motion_active"],
            motion_name=shared["motion_name"],
            fault_active=shared["fault_active"],
            fault_code=shared["fault_code"],
            fault_msg=shared["fault_msg"],
            replay_loaded=shared["replay_loaded"],
            replay_cursor=shared["replay_cursor"],
            replay_total=shared["replay_total"],
            replay_status=shared["replay_status"],
        )

    elif action == "enable":
        shared["mode"] = "enabled"
        return {"ok": True, "msg": "Robot enabled"}

    elif action == "disable":
        shared["mode"] = "disabled"
        shared["motion_active"] = False
        shared["motion_name"] = ""
        return {"ok": True, "msg": "Robot disabled"}

    elif action == "init":
        duration_s = float(parts[1]) if len(parts) > 1 else 2.5
        shared["mode"] = "enabled"
        shared["positions"] = list(DEFAULT_POS)
        shared["motion_active"] = False
        shared["motion_name"] = ""
        shared["fault_active"] = False
        time.sleep(min(duration_s, 1.0))  # don't actually wait full duration in mock
        return {"ok": True, "msg": f"Robot initialized in {duration_s:.1f}s"}

    elif action == "set_joint":
        values = [float(x) for x in parts[1:]]
        if len(values) != NUM_MOTORS:
            return {"ok": False, "error": {"message": f"expected {NUM_MOTORS} values, got {len(values)}"}}
        shared["positions"] = values
        return {"ok": True, "msg": "joint targets set"}

    elif action == "joint_test":
        if len(parts) < 3:
            return {"ok": False, "error": {"message": "usage: joint_test <indices> <target_rad>"}}
        indices = [int(x) for x in parts[1].split(",")]
        target = float(parts[2])
        for i in indices:
            if 0 <= i < NUM_MOTORS:
                shared["positions"][i] = target
        shared["motion_active"] = True
        shared["motion_name"] = "joint_test"
        # auto-clear after a moment
        t = threading.Timer(0.5, _clear_motion, args=(shared,))
        t.daemon = True
        t.start()
        return {"ok": True, "msg": f"joint_test sent to {indices}"}

    elif action == "joint_sine":
        if len(parts) < 5:
            return {"ok": False, "error": {"message": "usage: joint_sine <indices> <amp> <freq> <duration>"}}
        indices = [int(x) for x in parts[1].split(",")]
        amp = float(parts[2])
        freq = float(parts[3])
        duration = float(parts[4])
        shared["motion_active"] = True
        shared["motion_name"] = "joint_sine"
        shared["_sine"] = {"indices": indices, "amp": amp, "freq": freq, "end": time.time() + duration}
        return {"ok": True, "msg": f"joint_sine sent to {indices}"}

    elif action == "set_mit_param":
        if len(parts) < 5:
            return {"ok": False, "error": {"message": "usage: set_mit_param <kp> <kd> <vel_limit> <torque_limit>"}}
        return {"ok": True, "msg": "mit params set"}

    elif action == "load_replay_csv":
        path = parts[1] if len(parts) > 1 else "/tmp/mock_replay.csv"
        shared["replay_loaded"] = True
        shared["replay_total"] = 1000
        shared["replay_cursor"] = 0
        shared["replay_status"] = "idle"
        return {"ok": True, "msg": f"loaded {path}", "detail": {"total_frames": 1000}}

    elif action == "replay_start":
        speed = float(parts[1]) if len(parts) > 1 else 1.0
        shared["replay_status"] = "playing"
        shared["replay_cursor"] = 0
        shared["motion_active"] = True
        shared["motion_name"] = "replay"
        return {"ok": True, "msg": f"replay started at {speed:.1f}x"}

    elif action == "replay_stop":
        shared["replay_status"] = "idle"
        shared["motion_active"] = False
        shared["motion_name"] = ""
        return {"ok": True, "msg": "replay stopped"}

    elif action == "replay_step":
        if shared["replay_loaded"]:
            shared["replay_cursor"] = min(shared["replay_cursor"] + 1, shared["replay_total"] - 1)
        return {"ok": True, "msg": f"replay_step -> frame {shared['replay_cursor']}"}

    elif action == "replay_prev":
        if shared["replay_loaded"]:
            shared["replay_cursor"] = max(shared["replay_cursor"] - 1, 0)
        return {"ok": True, "msg": f"replay_prev -> frame {shared['replay_cursor']}"}

    elif action == "replay_seek":
        if len(parts) < 2:
            return {"ok": False, "error": {"message": "usage: replay_seek <frame_idx>"}}
        idx = int(parts[1])
        shared["replay_cursor"] = max(0, min(idx, shared["replay_total"] - 1))
        return {"ok": True, "msg": f"replay_seek -> frame {shared['replay_cursor']}"}

    elif action == "replay_status":
        return {
            "ok": True,
            "msg": "replay status",
            "detail": {
                "loaded": shared["replay_loaded"],
                "cursor": shared["replay_cursor"],
                "total_frames": shared["replay_total"],
                "speed_factor": 1.0,
                "status": shared["replay_status"],
            },
        }

    elif action == "replay":
        if len(parts) < 3:
            return {"ok": False, "error": {"message": "usage: replay <csv_path> <speed_factor>"}}
        shared["replay_loaded"] = True
        shared["replay_total"] = 1000
        shared["replay_cursor"] = 0
        shared["replay_status"] = "playing"
        shared["motion_active"] = True
        shared["motion_name"] = "replay"
        return {"ok": True, "msg": "replay started"}

    else:
        return {"ok": False, "error": {"message": f"unknown command: {action}"}}


def _clear_motion(shared: dict) -> None:
    shared["motion_active"] = False
    shared["motion_name"] = ""


def _cmd_server(host: str, port: int, shared: dict) -> None:
    """Handle command connections (one request/reply per connection or persistent)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"[mock] Command server listening on {host}:{port}")

    while True:
        try:
            conn, addr = srv.accept()
            conn.settimeout(30.0)
        except OSError:
            break
        print(f"[mock] Command connection from {addr}")
        threading.Thread(target=_cmd_handler, args=(conn, shared), daemon=True).start()


def _cmd_handler(conn: socket.socket, shared: dict) -> None:
    """Process commands on a single connection (persistent, one per line)."""
    buf = b""
    try:
        while True:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode("utf-8", errors="replace").strip()
                if not cmd:
                    continue
                print(f"[mock] cmd: {cmd}")
                reply = handle_command(cmd, shared)
                payload = (json.dumps(reply, separators=(",", ":")) + "\n").encode("utf-8")
                conn.sendall(payload)
                print(f"[mock] reply: {reply.get('msg', reply.get('error', {}).get('message', '?'))}")
    except (OSError, ConnectionResetError):
        pass
    finally:
        conn.close()
        print("[mock] Command connection closed")


def _state_server(host: str, port: int, shared: dict) -> None:
    """Push state stream to connected clients at ~20Hz."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"[mock] State server listening on {host}:{port}")

    clients: list[socket.socket] = []
    lock = threading.Lock()

    def accept_loop() -> None:
        while True:
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with lock:
                clients.append(conn)
            print(f"[mock] State client connected from {addr}")

    threading.Thread(target=accept_loop, daemon=True).start()

    seq = 0
    while True:
        # Update sine motion if active
        sine_cfg = shared.get("_sine")
        if sine_cfg and time.time() < sine_cfg["end"]:
            t = time.time()
            for i in sine_cfg["indices"]:
                if 0 <= i < NUM_MOTORS:
                    shared["positions"][i] = sine_cfg["amp"] * math.sin(2 * math.pi * sine_cfg["freq"] * t)
        elif sine_cfg and time.time() >= sine_cfg["end"]:
            shared.pop("_sine", None)
            _clear_motion(shared)

        # Advance replay cursor if playing
        if shared["replay_status"] == "playing" and shared["replay_loaded"]:
            shared["replay_cursor"] = (shared["replay_cursor"] + 1) % shared["replay_total"]

        seq += 1
        shared["seq"] = seq

        state = make_state(
            seq=seq,
            mode=shared["mode"],
            joint_positions=list(shared["positions"]),
            motion_active=shared["motion_active"],
            motion_name=shared["motion_name"],
            fault_active=shared["fault_active"],
            fault_code=shared["fault_code"],
            fault_msg=shared["fault_msg"],
            replay_loaded=shared["replay_loaded"],
            replay_cursor=shared["replay_cursor"],
            replay_total=shared["replay_total"],
            replay_status=shared["replay_status"],
        )
        payload = (json.dumps(state, separators=(",", ":")) + "\n").encode("utf-8")

        with lock:
            dead = []
            for c in clients:
                try:
                    c.sendall(payload)
                except OSError:
                    dead.append(c)
            for c in dead:
                clients.remove(c)
                try:
                    c.close()
                except OSError:
                    pass
            if dead:
                print(f"[mock] {len(dead)} state client(s) disconnected")

        time.sleep(0.05)  # ~20Hz


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock dog daemon for frontend testing")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--cmd-port", type=int, default=47001, help="Command port (default: 47001)")
    parser.add_argument("--state-port", type=int, default=47002, help="State stream port (default: 47002)")
    args = parser.parse_args()

    shared = {
        "seq": 0,
        "mode": "disabled",
        "positions": list(DEFAULT_POS),
        "motion_active": False,
        "motion_name": "",
        "fault_active": False,
        "fault_code": 0,
        "fault_msg": "",
        "replay_loaded": False,
        "replay_cursor": 0,
        "replay_total": 0,
        "replay_status": "idle",
    }

    t1 = threading.Thread(target=_cmd_server, args=(args.host, args.cmd_port, shared), daemon=True)
    t2 = threading.Thread(target=_state_server, args=(args.host, args.state_port, shared), daemon=True)
    t1.start()
    t2.start()

    print("[mock] Running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[mock] Shutting down.")


if __name__ == "__main__":
    main()

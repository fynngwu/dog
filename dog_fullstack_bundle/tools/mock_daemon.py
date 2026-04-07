#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import socket
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

CMD_PORT = 47001
STATE_PORT = 47002

JOINT_NAMES = [
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
]


def ok(msg: str = "ok", extra: Optional[Dict] = None) -> bytes:
    payload = {"ok": True, "msg": msg}
    if extra:
        payload.update(extra)
    return (json.dumps(payload) + "\n").encode()


def err(message: str, code: str = "bad_command", detail: str = "") -> bytes:
    payload = {"ok": False, "msg": message, "error": {"code": code, "message": message}}
    if detail:
        payload["error"]["detail"] = detail
    return (json.dumps(payload) + "\n").encode()


@dataclass
class SimState:
    enabled: bool = False
    seq: int = 0
    joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    joint_velocities: List[float] = field(default_factory=lambda: [0.0] * 12)
    joint_torques: List[float] = field(default_factory=lambda: [0.0] * 12)
    target_joint_positions: List[float] = field(default_factory=lambda: [0.0] * 12)
    offline_motors: List[str] = field(default_factory=list)
    motion_active: bool = False
    motion_name: str = ""
    motion_last_error: str = ""
    replay_loaded: bool = False
    replay_csv_path: str = ""
    replay_cursor: int = 0
    replay_total: int = 0
    replay_speed: float = 1.0
    replay_status: str = "idle"
    replay_samples: List[List[float]] = field(default_factory=list)
    sine_thread: Optional[threading.Thread] = None
    stop_motion: threading.Event = field(default_factory=threading.Event)

    def snapshot(self) -> Dict:
        payload = {
            "ok": True,
            "mode": "enabled" if self.enabled else "disabled",
            "seq": self.seq,
            "state": {
                "joint_positions": self.joint_positions,
                "joint_velocities": self.joint_velocities,
                "joint_torques": self.joint_torques,
                "target_joint_positions": self.target_joint_positions,
                "offline_motors": self.offline_motors,
            },
            "motion": {
                "active": self.motion_active,
                "name": self.motion_name,
                "last_error": self.motion_last_error,
            },
            "replay": {
                "loaded": self.replay_loaded,
                "csv_path": self.replay_csv_path,
                "cursor": self.replay_cursor,
                "total_frames": self.replay_total,
                "speed_factor": self.replay_speed,
                "status": self.replay_status,
            },
        }
        return payload


class MockDaemon:
    def __init__(self, host: str = "127.0.0.1", cmd_port: int = CMD_PORT, state_port: int = STATE_PORT):
        self.host = host
        self.cmd_port = cmd_port
        self.state_port = state_port
        self.state = SimState()
        self._lock = threading.Lock()
        self._state_clients: List[socket.socket] = []
        self._running = False

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._state_publisher, daemon=True).start()
        threading.Thread(target=self._serve_state, daemon=True).start()
        self._serve_cmd()

    def _serve_cmd(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.cmd_port))
        server.listen()
        print(f"[mock] cmd server listening on {self.cmd_port}")
        try:
            while self._running:
                conn, _ = server.accept()
                threading.Thread(target=self._handle_cmd_client, args=(conn,), daemon=True).start()
        finally:
            server.close()

    def _serve_state(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.state_port))
        server.listen()
        print(f"[mock] state server listening on {self.state_port}")
        try:
            while self._running:
                conn, _ = server.accept()
                conn.setblocking(True)
                with self._lock:
                    self._state_clients.append(conn)
        finally:
            server.close()

    def _handle_cmd_client(self, conn: socket.socket) -> None:
        fp = conn.makefile('r', encoding='utf-8')
        try:
            for line in fp:
                raw = line.strip()
                if not raw:
                    continue
                resp = self._process(raw)
                conn.sendall(resp)
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _process(self, raw: str) -> bytes:
        toks = raw.split()
        op = toks[0]
        with self._lock:
            if op == 'ping':
                return ok('pong')
            if op == 'get_state':
                return (json.dumps(self.state.snapshot()) + "\n").encode()
            if op == 'enable':
                self.state.enabled = True
                return ok('enabled')
            if op == 'disable':
                self.state.enabled = False
                self.state.motion_active = False
                self.state.motion_name = ''
                self.state.replay_status = 'idle'
                self.state.stop_motion.set()
                return ok('disabled')
            if op == 'init':
                self.state.enabled = True
                self.state.target_joint_positions = [0.0] * 12
                self.state.joint_positions = [0.0] * 12
                self.state.joint_velocities = [0.0] * 12
                return ok('initialized')
            if op == 'set_mit_param':
                return ok('mit params set')
            if op == 'joint_test':
                if len(toks) != 3:
                    return err('joint_test expects: joint_test <idx[,idx...]> <target_rad>')
                idxs = [int(x) for x in toks[1].split(',')]
                target = float(toks[2])
                for idx in idxs:
                    self.state.target_joint_positions[idx] = target
                    self.state.joint_positions[idx] = target
                    self.state.joint_velocities[idx] = 0.0
                    self.state.joint_torques[idx] = 0.1
                self.state.motion_active = False
                self.state.motion_name = 'joint_test'
                return ok('joint_test accepted')
            if op == 'joint_sine':
                if len(toks) != 5:
                    return err('joint_sine expects: joint_sine <idx[,idx...]> <amp_rad> <freq_hz> <duration_sec>')
                idxs = [int(x) for x in toks[1].split(',')]
                amp = float(toks[2]); freq = float(toks[3]); dur = float(toks[4])
                self._start_sine(idxs, amp, freq, dur)
                return ok('joint_sine started')
            if op == 'load_replay_csv':
                if len(toks) != 2:
                    return err('load_replay_csv expects: load_replay_csv <csv_path>')
                path = Path(toks[1])
                if not path.exists():
                    return err('csv not found', 'not_found', str(path))
                samples = []
                with path.open('r', encoding='utf-8') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        sample = [float(row[f'scaled_action_{i}']) for i in range(12)]
                        samples.append(sample)
                self.state.replay_samples = samples
                self.state.replay_loaded = True
                self.state.replay_csv_path = str(path)
                self.state.replay_total = len(samples)
                self.state.replay_cursor = 0
                self.state.replay_status = 'idle'
                return ok('replay csv loaded')
            if op == 'replay_start':
                self.state.replay_status = 'playing'
                self.state.motion_active = True
                self.state.motion_name = 'replay'
                return ok('replay started')
            if op == 'replay_stop':
                self.state.replay_status = 'idle'
                self.state.motion_active = False
                self.state.motion_name = ''
                return ok('replay stopped')
            if op == 'replay_step':
                return self._step_replay(forward=True)
            if op == 'replay_prev':
                return self._step_replay(forward=False)
            if op == 'replay_seek':
                if len(toks) != 2:
                    return err('replay_seek expects: replay_seek <frame_idx>')
                idx = max(0, min(int(toks[1]), max(0, self.state.replay_total - 1)))
                self.state.replay_cursor = idx
                return ok('replay cursor updated')
            if op == 'replay_status':
                return ok('replay status', {'replay': self.state.snapshot()['replay']})
        return err(f'unknown command: {op}', 'bad_command')

    def _start_sine(self, idxs: List[int], amp: float, freq: float, dur: float) -> None:
        self.state.stop_motion.set()
        self.state.stop_motion = threading.Event()
        self.state.motion_active = True
        self.state.motion_name = 'joint_sine'

        def worker():
            start = time.time()
            last = start
            while time.time() - start < dur and not self.state.stop_motion.is_set():
                now = time.time()
                t = now - start
                with self._lock:
                    for idx in idxs:
                        pos = amp * math.sin(2 * math.pi * freq * t)
                        vel = 2 * math.pi * freq * amp * math.cos(2 * math.pi * freq * t)
                        self.state.target_joint_positions[idx] = pos
                        self.state.joint_positions[idx] = pos
                        self.state.joint_velocities[idx] = vel
                        self.state.joint_torques[idx] = 0.15
                time.sleep(max(0.0, 0.02 - (time.time() - last)))
                last = time.time()
            with self._lock:
                self.state.motion_active = False
                self.state.motion_name = ''
        threading.Thread(target=worker, daemon=True).start()

    def _step_replay(self, forward: bool) -> bytes:
        if not self.state.replay_loaded:
            return err('no replay loaded', 'not_loaded')
        if self.state.replay_total == 0:
            return err('replay is empty', 'empty_replay')
        if forward:
            self.state.replay_cursor = min(self.state.replay_cursor + 1, self.state.replay_total - 1)
        else:
            self.state.replay_cursor = max(self.state.replay_cursor - 1, 0)
        sample = self.state.replay_samples[self.state.replay_cursor]
        self.state.target_joint_positions = list(sample)
        self.state.joint_positions = list(sample)
        self.state.joint_velocities = [0.0] * 12
        self.state.joint_torques = [0.2] * 12
        return ok('replay step applied')

    def _state_publisher(self) -> None:
        while self._running:
            with self._lock:
                self.state.seq += 1
                payload = (json.dumps(self.state.snapshot()) + '\n').encode()
                dead = []
                for sock in self._state_clients:
                    try:
                        sock.sendall(payload)
                    except OSError:
                        dead.append(sock)
                for sock in dead:
                    self._state_clients.remove(sock)
            time.sleep(0.05)


if __name__ == '__main__':
    MockDaemon().start()

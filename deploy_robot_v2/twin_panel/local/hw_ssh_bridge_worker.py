from __future__ import annotations

import json
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass

from PyQt5 import QtCore

from twin_topics import NUM_JOINTS


@dataclass
class HwSshConfig:
    ssh_target: str
    remote_cli: str = '~/.local/bin/hw_action_cli'
    sample_delay_ms: int = 15
    min_tx_interval_s: float = 0.20
    ssh_timeout_s: float = 4.0
    auto_report: bool = True
    low_gain: bool = False


class HwSshBridgeWorker(QtCore.QThread):
    status = QtCore.pyqtSignal(str)
    state_ready = QtCore.pyqtSignal(dict)

    def __init__(self, config: HwSshConfig, parent=None) -> None:
        super().__init__(parent)
        self._config = config
        self._running = True
        self._lock = threading.Lock()
        self._latest_action = [0.0] * NUM_JOINTS
        self._action_dirty = False
        self._control_queue: deque = deque()
        self._last_tx = 0.0

    def stop(self) -> None:
        self._running = False

    def set_action(self, values: list) -> None:
        with self._lock:
            if len(values) >= NUM_JOINTS:
                self._latest_action = list(values[:NUM_JOINTS])
                self._action_dirty = True

    def send_command(self, text: str) -> None:
        with self._lock:
            self._control_queue.append(text.strip())

    def _pop_next(self):
        with self._lock:
            if self._control_queue:
                return self._control_queue.popleft(), None
            if self._action_dirty:
                self._action_dirty = False
                return None, list(self._latest_action)
        return None, None

    def _run_ssh(self, remote_args: list) -> dict:
        base = ['ssh', self._config.ssh_target, self._config.remote_cli] + remote_args
        started = time.time()
        try:
            result = subprocess.run(base, text=True, capture_output=True, timeout=self._config.ssh_timeout_s)
            stdout = result.stdout.strip().splitlines()
            stderr = result.stderr.strip()
            if result.returncode != 0:
                error_msg = f'ssh exit={result.returncode}'
                if stderr:
                    error_msg += f' | stderr: {stderr}'
                return {'ok': False, 'error': error_msg, 'rtt_ms': int((time.time() - started) * 1000)}
            if not stdout:
                return {'ok': False, 'error': 'empty stdout', 'rtt_ms': int((time.time() - started) * 1000)}
            payload = json.loads(stdout[-1])
            payload.setdefault('rtt_ms', int((time.time() - started) * 1000))
            return payload
        except Exception as exc:
            return {'ok': False, 'error': str(exc), 'rtt_ms': int((time.time() - started) * 1000)}

    def _action_command(self, action: list) -> list:
        csv = ','.join(f'{x:.6f}' for x in action[:NUM_JOINTS])
        args = ['set', '--action', csv, '--sample-delay-ms', str(self._config.sample_delay_ms), '--json']
        if self._config.auto_report:
            args.append('--auto-report')
        if self._config.low_gain:
            args.append('--low-gain')
        return args

    def _control_command(self, text: str) -> list:
        parts = text.split()
        if not parts:
            return ['ping', '--json']
        cmd = parts[0]
        if cmd == 'enable':
            args = ['enable', '--json']
            if self._config.auto_report:
                args.append('--auto-report')
            if self._config.low_gain:
                args.append('--low-gain')
            return args
        if cmd == 'goto_offset':
            seconds = parts[1] if len(parts) > 1 else '2.0'
            args = ['goto_offset', '--seconds', seconds, '--json']
            if self._config.auto_report:
                args.append('--auto-report')
            if self._config.low_gain:
                args.append('--low-gain')
            return args
        if cmd == 'disable':
            return ['disable', '--json']
        if cmd == 'hold':
            return ['hold', '--sample-delay-ms', str(self._config.sample_delay_ms), '--json']
        if cmd == 'get_state':
            return ['get_state', '--sample-delay-ms', str(self._config.sample_delay_ms), '--json']
        if cmd == 'enable_auto_report':
            return ['enable_auto_report', '--json']
        if cmd == 'disable_auto_report':
            return ['disable_auto_report', '--json']
        return ['ping', '--json']

    def run(self) -> None:
        self.status.emit('SSH worker ready')
        while self._running:
            control_text, action = self._pop_next()
            now = time.time()
            if control_text is not None:
                payload = self._run_ssh(self._control_command(control_text))
                self.state_ready.emit(payload)
                self._last_tx = now
            elif action is not None and (now - self._last_tx) >= self._config.min_tx_interval_s:
                payload = self._run_ssh(self._action_command(action))
                self.state_ready.emit(payload)
                self._last_tx = now
            time.sleep(0.01)
#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BACKEND_PY = ROOT / 'backend' / 'python'
if str(BACKEND_PY) not in sys.path:
    sys.path.insert(0, str(BACKEND_PY))

from experiment_manager import ExperimentManager
from replay_controller import ReplayController
from robot_backend import RobotBackend


def main() -> int:
    mock = subprocess.Popen([sys.executable, str(ROOT / 'tools' / 'mock_daemon.py')])
    try:
        for _ in range(20):
            try:
                mgr = ExperimentManager(host='127.0.0.1', cmd_port=47001, state_port=47002)
                mgr.connect()
                break
            except OSError:
                time.sleep(0.2)
        else:
            raise RuntimeError('mock daemon did not open command port in time')
        assert mgr.backend.client.ping() is True
        print('PING OK')
        print(mgr.init_robot())
        print(mgr.joint_test([0, 1], 0.2))
        print(mgr.joint_sine([2], 0.15, 0.5, 0.5))
        time.sleep(0.2)
        state = mgr.get_state()
        print('STATE KEYS', sorted(state.keys()))
        rb = RobotBackend(host='127.0.0.1', cmd_port=47001, state_port=47002)
        rb.connect()
        rc = ReplayController(rb)
        csv_text = (ROOT / 'tests' / 'sample_replay.csv').read_text(encoding='utf-8')
        print(rc.stage_and_load_csv('sample_replay.csv', csv_text))
        print(rc.step())
        print(rc.prev())
        print(rc.seek(3))
        print(rc.status())
        latest = []
        mgr.start_state_stream(lambda s: latest.append(s))
        time.sleep(0.2)
        mgr.stop_state_stream()
        assert latest, 'no state stream received'
        print('STATE STREAM OK', json.dumps(latest[-1]['replay']))
        mgr.disconnect()
        rb.disconnect()
        print('ALL MOCK INTEGRATION CHECKS PASSED')
        return 0
    finally:
        mock.terminate()
        try:
            mock.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            mock.kill()


if __name__ == '__main__':
    raise SystemExit(main())

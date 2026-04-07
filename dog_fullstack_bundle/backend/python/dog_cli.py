"""Command-line and REPL client for the robot debug daemon."""

from __future__ import annotations

import argparse
import json
import shlex
import sys
from typing import Any, Dict, Iterable, List, Optional

from robot_client import RobotClient


class Color:
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    CYAN = "\033[36m"
    RESET = "\033[0m"
    BOLD = "\033[1m"


def colorize(text: str, color: str) -> str:
    return f"{color}{text}{Color.RESET}"


class DogCLI:
    def __init__(self, client: RobotClient):
        self.client = client

    def run_tokens(self, tokens: List[str]) -> Dict[str, Any]:
        if not tokens:
            return {"ok": True, "msg": "no command"}
        op = tokens[0]

        if op == "ping":
            return self.client.send_command("ping")
        if op == "get_state":
            return self.client.get_state_once()
        if op == "init":
            seconds = float(tokens[1]) if len(tokens) > 1 else 2.5
            return self.client.init(seconds)
        if op in {"enable", "disable", "replay_stop", "replay_step", "replay_prev", "replay_status"}:
            return self.client.send_command(op)
        if op == "set_joint":
            if len(tokens) != 13:
                raise ValueError("set_joint expects 12 floats")
            return self.client.send_command(" ".join(tokens))
        if op == "joint_test":
            if len(tokens) != 3:
                raise ValueError("joint_test expects: joint_test <idx[,idx...]> <target_rad>")
            return self.client.send_command(" ".join(tokens))
        if op == "joint_sine":
            if len(tokens) != 5:
                raise ValueError("joint_sine expects: joint_sine <idx[,idx...]> <amp_rad> <freq_hz> <duration_sec>")
            return self.client.send_command(" ".join(tokens))
        if op == "load_replay_csv":
            if len(tokens) != 2:
                raise ValueError("load_replay_csv expects: load_replay_csv <csv_path>")
            return self.client.send_command(" ".join(tokens))
        if op == "replay_start":
            if len(tokens) not in {1, 2}:
                raise ValueError("replay_start expects: replay_start [speed_factor]")
            return self.client.send_command(" ".join(tokens))
        if op == "replay_seek":
            if len(tokens) != 2:
                raise ValueError("replay_seek expects: replay_seek <frame_idx>")
            return self.client.send_command(" ".join(tokens))
        if op == "replay":
            if len(tokens) not in {2, 3}:
                raise ValueError("replay expects: replay <csv_path> [speed_factor]")
            return self.client.send_command(" ".join(tokens))
        if op == "set_mit_param":
            if len(tokens) != 5:
                raise ValueError("set_mit_param expects: set_mit_param <kp> <kd> <vel_limit> <torque_limit>")
            return self.client.send_command(" ".join(tokens))
        raise ValueError(f"unknown command: {op}")


def print_reply(reply: Dict[str, Any]) -> None:
    ok = bool(reply.get("ok", False))
    banner = colorize("OK", Color.GREEN if ok else Color.RED)
    print(f"{Color.BOLD}{banner}{Color.RESET}")
    print(json.dumps(reply, indent=2, sort_keys=True))


def repl(cli: DogCLI) -> int:
    print(colorize("Connected. Type 'help' for examples, 'quit' to exit.", Color.CYAN))
    while True:
        try:
            line = input(colorize("dog> ", Color.YELLOW))
        except EOFError:
            print()
            return 0
        except KeyboardInterrupt:
            print()
            return 130
        line = line.strip()
        if not line:
            continue
        if line in {"quit", "exit"}:
            return 0
        if line == "help":
            print(
                "Commands:\n"
                "  ping\n"
                "  init [duration]\n"
                "  enable / disable\n"
                "  set_joint <12 floats>\n"
                "  joint_test <idx[,idx...]> <target_rad>\n"
                "  joint_sine <idx[,idx...]> <amp_rad> <freq_hz> <duration_sec>\n"
                "  set_mit_param <kp> <kd> <vel_limit> <torque_limit>\n"
                "  load_replay_csv <csv_path>\n"
                "  replay_start [speed]\n"
                "  replay_stop / replay_step / replay_prev / replay_seek <idx> / replay_status\n"
                "  replay <csv_path> [speed]\n"
                "  get_state\n"
            )
            continue
        try:
            reply = cli.run_tokens(shlex.split(line))
        except Exception as exc:
            print(colorize(f"Error: {exc}", Color.RED), file=sys.stderr)
            continue
        print_reply(reply)


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Robot dog debug CLI")
    ap.add_argument("--host", required=True, help="Daemon host or IP")
    ap.add_argument("--cmd-port", type=int, default=47001, help="Command port")
    ap.add_argument("--state-port", type=int, default=47002, help="State push port")
    ap.add_argument("--timeout", type=float, default=10.0, help="Socket timeout in seconds")
    ap.add_argument("command", nargs=argparse.REMAINDER, help="Command and its arguments")
    return ap


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_argparser().parse_args(list(argv) if argv is not None else None)
    client = RobotClient(args.host, args.cmd_port, args.state_port, args.timeout)
    try:
        client.connect()
        cli = DogCLI(client)
        if args.command:
            reply = cli.run_tokens(args.command)
            print_reply(reply)
            return 0 if reply.get("ok", False) else 1
        return repl(cli)
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
import argparse
import json
import socket
import sys
from typing import List


def send_command(host: str, port: int, cmd: str) -> dict:
    with socket.create_connection((host, port), timeout=3.0) as sock:
        sock.sendall((cmd.strip() + "\n").encode("utf-8"))
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError("连接已关闭")
            buf += chunk
        return json.loads(buf.split(b"\n", 1)[0].decode("utf-8"))


def format_joint_cmd(values: List[float]) -> str:
    if len(values) != 12:
        raise ValueError("需要 12 个关节值")
    return "set_joint " + " ".join(f"{v:.6f}" for v in values)


def main() -> int:
    ap = argparse.ArgumentParser(description="TwinAgent 简单客户端")
    ap.add_argument("--host", required=True)
    ap.add_argument("--cmd-port", type=int, default=47001)
    ap.add_argument("--cmd", default="get_state", help="直接发送的命令")
    ap.add_argument("--joint", nargs=12, type=float, help="发送 set_joint 12 维目标")
    args = ap.parse_args()

    try:
        cmd = format_joint_cmd(args.joint) if args.joint is not None else args.cmd
        result = send_command(args.host, args.cmd_port, cmd)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

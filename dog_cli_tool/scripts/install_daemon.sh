#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT_DIR/build/dog_debug_daemon"
if [[ ! -x "$BIN" ]]; then
  echo "Binary not found: $BIN" >&2
  echo "Run ./build.sh first." >&2
  exit 1
fi
install -m 0755 "$BIN" /usr/local/bin/dog_debug_daemon

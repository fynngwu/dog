#!/usr/bin/env bash
set -euo pipefail
HOST="${1:-127.0.0.1}"
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

cat <<MSG
Example session against host: $HOST
MSG

python "$ROOT_DIR/python/dog_cli.py" --host "$HOST" ping
python "$ROOT_DIR/python/dog_cli.py" --host "$HOST" init 2.5
python "$ROOT_DIR/python/dog_cli.py" --host "$HOST" get_state
python "$ROOT_DIR/python/dog_cli.py" --host "$HOST" set_mit_param 40 0.5 44 17

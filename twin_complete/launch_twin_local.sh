#!/usr/bin/env bash
set -e

# 写死配置
HOST="100.109.192.69"
MODEL="/home/wufy/github_respository/dog_project/dog/deploy_robot_v2/twin_panel/local/robot_fixed.xml"

cd /home/wufy/github_respository/dog_project/dog
uv run python /home/wufy/github_respository/dog_project/dog/twin_complete/twin_local_console.py --host "$HOST" --model "$MODEL"
# dog_cli.py

## 角色

给调试人员的 CLI / REPL。  
它不是前端核心依赖，但对联调非常有价值。

## 作用

- 单次发命令
- 进入 REPL
- 把 JSON 回复格式化打印出来

## 支持命令

- `ping`
- `get_state`
- `init`
- `enable`
- `disable`
- `set_joint`
- `joint_test`
- `joint_sine`
- `set_mit_param`
- `load_replay_csv`
- `replay_start`
- `replay_stop`
- `replay_step`
- `replay_prev`
- `replay_seek`
- `replay_status`
- `replay`

## 你什么时候读它

- 想快速验证协议是否通
- 想知道最原始命令应该怎么拼
- 想在 GUI 前先手工打通后端

# twin_protocol.hpp

## 角色

纯协议工具头文件。  
负责：

- trim
- 空白分词
- JSON escape
- JSON 数组拼装
- 成功/失败回复拼装
- 固定数量浮点参数解析

## 主要函数

- `Trim`
- `SplitWS`
- `JsonEscape`
- `JsonArray`
- `JsonStringArray`
- `OkReply`
- `ErrorReply`
- `ParseFloatList`

## 价值

这个文件把 `TwinAgent` 里的协议杂项都抽出来了，已经是一个比较合理的“轻工具层”。

## 维护建议

如果以后协议继续扩展，可以把：
- 数字数组
- 对象拼装
- error code 常量
继续从 `TwinAgent` 往这里收。

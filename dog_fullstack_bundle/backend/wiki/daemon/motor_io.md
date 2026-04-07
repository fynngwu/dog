# motor_io.cpp / motor_io.hpp

## 角色

`MotorIO` 是“关节空间抽象层”。

它把下层电机控制包装成上层更稳定的接口：
- joint offset
- direction
- clamp
- relative target
- smooth move
- feedback freshness
- offline motor 检查

## 关键常量

- `kNumMotors = 12`
- `kKneeRatio = 1.667`
- `kActionScale = 0.25`
- 默认 MIT 参数：
  - `kp = 40`
  - `kd = 0.5`
  - `vel_limit = 44`
  - `torque_limit = 17`

### 标定相关
- `kMotorIds`
- `kJointDirection`
- `kJointOffsets`
- `kXmlMin`
- `kXmlMax`
- `kCanIds`

这些目前是**硬编码**的。

---

## 对上层暴露的主接口

## 初始化与使能
- `Initialize()`
- `EnableAll()`
- `DisableAll()`
- `EnableAllAutoReport()`
- `EnableJoint(int)`

## 参数
- `SetMITConfig(...)`

## 运动
- `MoveToOffset(duration_sec)`
- `SendActions(actions, action_scale)`
- `SendJointRelativeTargets(rel_targets)`
- `SendSingleMotorRelativeTarget(motor_index, rel_target)`

## 状态
- `GetJointObs()`
- `GetMotorStates(...)`
- `CheckFeedbackFresh(max_age_ms)`
- `GetOfflineJoints(max_age_ms)`

## 工具
- `ClampRelativeTargetRad(...)`
- `ClampAbsoluteTargetRad(...)`

---

## 语义重点

### relative target
当前项目里的“目标角”是**相对 offset 的关节空间角度**。  
所以前端输入的 `joint_test` / `set_joint` / replay CSV 都是在这个语义下工作。

### clamp
所有目标在真正发下去前都会过 clamp。  
这保证前端不用自己做极限裁剪。

### smooth move
`MoveToOffset()` 用于 `init` 的安全站立过程。

---

## 维护建议

如果以后要做更强的可配置化，最优先改这里：
- 把硬编码标定常量外置
- 把 12 关节表述整理成统一结构体数组

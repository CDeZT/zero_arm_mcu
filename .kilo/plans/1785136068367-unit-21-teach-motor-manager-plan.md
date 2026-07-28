# 单元 21 实施计划：拖动示教接入 motor_manager

## 目标与边界

按 Code Reference 第 16 节单元 21 与 Design 10.1，在 `motor_manager.c/.h` 中完成：

- `TEACH_START` 服务：停止选定轴 → 丢弃待发目标 → 启动 20 ms `S_CPOS/0x36` 定时返回 → 失能选定轴 → Robot 切到 `ROBOT_STATE_TEACHING`。
- `TEACH_STOP` 服务：停止 0x36 定时返回 → 把最终 `actual_joint_urad` 同步为新的运动参考目标 → Robot 切到 `ROBOT_STATE_READY`，保持失能。
- `0x36` 电机实时位置反馈：解析有符号 `0.1°` → `motor_urad` → `motor_to_joint_position()` → 写入 Robot `actual_joint_urad`。

本轮不实现：PC 协议命令字、TeachingTask、自动重力补偿、轨迹保存、板载限位/Homing、重新使能后追目标的状态机处理。

## 关键设计决策

1. **`robot_sync_reference_to_actual()` 行为（已确认选项 B）**
   - 把 `robot_state.actual_joint_urad[6]` 复制到 `robot_state.target_joint_urad[6]`。
   - 同时写入 `robot.c` 的 `g_latest_joint_target.joint_urad[6]`，恢复 `s_joint_target_valid = true`，递增 `g_joint_target_generation`。
   - `duration_ms` 与 `gripper_u16` 重置为 0（这两个量不属于 actual state，不能从电机读出）。
   - 这样 TEACH_STOP 后重新 ENABLE 再发目标时，会以当前实际位置为参考，避免追赶旧目标。

2. **`0x36` 转换更新时机**
   - 任何有效的 `0x36` 数据帧都立即转换并写入 `robot_state.actual_joint_urad`。
   - 实际场景下 `0x36` 只在 TEACH_START 后由电机定时返回；普通运动不请求该参数，因此不会引入额外开销。
   - `motor_to_joint_position()` 溢出时跳过该轴 actual 更新，不置 fault。

3. **服务处理错误策略**
   - `X_V2_Auto_Return_Sys_Params_Timed` / `X_V2_En_Control` 失败不中断同组其余轴的处理，与现有 `motor_manager_enable_mask`/`stop_mask` 风格一致。
   - 不检查 `robot_set_run_state` 返回值（现有 `robot.c` 请求层也不检查）。

## 需要修改的文件

### 源文件

- `Robot/Inc/robot.h`
  - 新增：`bool robot_sync_reference_to_actual(void);`
- `Robot/Src/robot.c`
  - 实现 `robot_sync_reference_to_actual()`。
- `Motor/Inc/motor_manager.h`
  - 无新增公开函数；如有内部扩展需要可保持原接口不变。
- `Motor/Src/motor_manager.c`
  - 新增 `#include "joint_transform.h"` 与 `#include "robot_state.h"`。
  - `motor_process_all_services()` 增加 `ROBOT_SERVICE_TEACH_START`、`ROBOT_SERVICE_TEACH_STOP` 分支。
  - `motor_manager_on_can_frame()` 在 `MOTOR_FUNC_POSITION` 分支内增加 `motor_to_joint_position()` 与 `robot_set_actual_joint()` 调用。

### 测试文件

- `Tests/test_motor_manager.c`
  - 增加 `X_V2_Auto_Return_Sys_Params_Timed` 桩函数及调用记录。
  - 增加 `robot_set_actual_joint` 桩函数（记录 joint_index 与 actual_urad）。
  - 增加 `robot_sync_reference_to_actual` 桩函数。
  - 增加 `robot_set_run_state` 桩函数（记录状态）。
  - 新增测试用例：
    1. `TEACH_START` 服务：验证 stop、discard、每个选定轴的 `S_CPOS 20 ms`、disable、状态切到 `TEACHING`。
    2. `TEACH_STOP` 服务：验证每个轴的 `S_CPOS 0 ms`、调用 `robot_sync_reference_to_actual`、状态切到 `READY`。
    3. `0x36` 帧解析同时更新 Robot actual（验证 `robot_set_actual_joint` 收到正确 joint/urad）。
    4. 非选定轴在 `TEACH_START` 中不被 stop/disable/auto-return。
- `Tests/test_robot.c`
  - 新增 `robot_sync_reference_to_actual()` 用例：先通过 `robot_set_actual_joint` 写入 actual，调用 sync 后验证 `robot_get_state` 的 target 与 actual 一致，且 `robot_get_latest_target` 返回有效目标。

### CMake

- `Tests/CMakeLists.txt`
  - `test_motor_manager` 目标加入 `../Motion/Src/joint_transform.c`。
- `CMakeLists.txt`（根）
  - 已包含 `joint_transform.c`，无需改动；可顺带确认 `test_motor_manager` 自定义目标存在（当前已存在）。

## 详细数据流

### TEACH_START

```text
PC -> robot_request_teach_start(mask)
  -> robot_invalidate_motion_target()  [已在 robot.c]
  -> queue ROBOT_SERVICE_TEACH_START, mask, prio=200
  -> MotorTask motor_process_all_services()
       -> motor_manager_stop_mask(mask)
       -> motor_discard_pending_target()
       -> for joint in mask:
              X_V2_Auto_Return_Sys_Params_Timed(motor_id, S_CPOS, 20)
              X_V2_En_Control(motor_id, false, false)
       -> robot_invalidate_motion_target()  [冗余但安全]
       -> robot_set_run_state(ROBOT_STATE_TEACHING)
```

### 0x36 实时位置

```text
FDCAN RX -> motor_manager_on_can_frame()
  -> MOTOR_FUNC_POSITION (length == 7, data[1] ∈ {0,1})
  -> magnitude = data[2..5]; negative = data[1]
  -> motor_urad = motor_x_position_to_urad(magnitude, negative)
  -> feedback->position_urad = motor_urad
  -> joint_index = motor_find_feedback_index(motor_id)
  -> if motor_to_joint_position(joint_index, motor_urad, &joint_urad):
         robot_set_actual_joint(joint_index, joint_urad)
```

### TEACH_STOP

```text
PC -> robot_request_teach_stop()
  -> queue ROBOT_SERVICE_TEACH_STOP, mask=0x3F, prio=200
  -> MotorTask motor_process_all_services()
       -> for joint in mask:
              X_V2_Auto_Return_Sys_Params_Timed(motor_id, S_CPOS, 0)
       -> robot_sync_reference_to_actual()
       -> robot_set_run_state(ROBOT_STATE_READY)
```

## 实现细节与边界条件

- `motor_x_position_to_urad` 已按 X 固件 `0.1°` 实现，无需改动。
- `motor_find_feedback_index` 返回的是 joint index（按 motor_id 查找），可直接用于 `robot_set_actual_joint` 与 `motor_to_joint_position`。
- TEACH_START/STOP 服务处理使用现有 `motor_for_each_masked_joint` 辅助函数。
- `robot_sync_reference_to_actual()` 实现顺序：
  1. `robot_get_state(&state)` 获取 actual。
  2. 加 `s_joint_target_mutex`。
  3. 复制 actual 到 `g_latest_joint_target.joint_urad`；`duration_ms = 0`；`gripper_u16 = 0`；`s_joint_target_valid = true`；`g_joint_target_generation++`。
  4. 释放 mutex。
  5. `robot_update_target_state(&g_latest_joint_target)` 写入 state target。
- 若任一 mutex 获取失败，返回 `false`，但已修改的部分不回滚（与现有错误处理风格一致）。

## 验证步骤

1. **主机普通测试**
   ```sh
   cmake -S Tests -B build/host-tests -DCMAKE_BUILD_TYPE=Debug
   cmake --build build/host-tests --parallel
   ctest --test-dir build/host-tests --output-on-failure
   ```
   - 预期全部通过，新增 `test_motor_manager` 与 `test_robot` 用例。

2. **ASan/UBSan**
   ```sh
   cmake -S Tests -B build/host-tests-sanitize \
     -DCMAKE_BUILD_TYPE=Debug \
     '-DCMAKE_C_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
     '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
   cmake --build build/host-tests-sanitize --parallel
   ctest --test-dir build/host-tests-sanitize --output-on-failure
   ```

3. **STM32 Debug 交叉编译**
   ```sh
   cmake --preset Debug
   cmake --build --preset Debug --parallel
   ```
   - 固件路径：`build/Debug/zero_arm_mcu.elf`。

4. **板测（安全限制）**
   - 只允许：刷写、verify、HELLO、GET_STATE、多轮只读通信稳定性。
   - **禁止**：TEACH_START、ENABLE、非零 SET_JOINT_TARGET 或任何可能松轴/运动的命令。
   - 刷写命令：
     ```sh
     /opt/homebrew/bin/openocd \
       -f interface/stlink.cfg \
       -f target/stm32g4x.cfg \
       -c "program build/Debug/zero_arm_mcu.elf verify reset exit"
     ```
   - 板测后报告 FLASH/RAM 占用量。

## 已知风险与未决问题

- 失能后电机是否仍持续发送 `0x36` 需在有可靠机械支撑的台架上验证，不在本轮主机/编译测试中覆盖。
- J2/J3 等重力轴失能可能快速下落；板测阶段不得发送 TEACH_START。
- `robot_sync_reference_to_actual()` 不会同步 duration/gripper；PC 在 TEACH_STOP 后首次回放应重新发送完整目标。
- 当前 `motor_manager_on_can_frame` 的 `0x36` 转换不检查 Robot 当前运行状态，依赖“普通运动不请求 0x36”这一前提。

## 轮次停止点

完成上述 `.c/.h`、CMake、主机测试、ASan/UBSan、STM32 编译及安全板测后：

1. 审查 diff（不提交、不清理工作区）。
2. 按 AGENTS.md 第 7 节交付格式报告：改动文件、接口/数据流、测试结果、FLASH/RAM、风险、板测是否发送过运动/使能/松轴命令。
3. 停止，等待用户回复“继续”。

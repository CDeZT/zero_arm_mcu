# ZEROARM MCU 后续 Agent 执行说明 V2

更新时间：2026-07-28

## 1. 任务目标

后续 Agent 的目标不是只让工程“能编译”，而是完成以下闭环：

```text
发现缺陷
 -> 固定复现
 -> 最小修复
 -> 专项测试
 -> 全量回归
 -> Sanitizer
 -> STM32 双构建
 -> 安全板测（适用时）
 -> diff 审查
 -> 一个缺陷一个提交
 -> 更新机器状态
```

所有当前可以由源码、Mock、故障注入、Sanitizer、交叉编译或只读板测确定的软件
问题都应处理。依赖机械装配、支撑、标定或真实电机的事实必须建立测试卡，不能
伪报为通过。

## 2. 开始时必须现场确认

每次开始执行：

```powershell
git status --short
git log --oneline -15
cmake --list-presets
```

然后读取：

1. `ZEROARM_MCU_HANDOFF_V2.md`
2. `ZEROARM_MCU_HANDOFF_STATUS_V2.yaml`
3. `docx/Reference_plan/ZEROARM_MCU_AUDIT_REMEDIATION_MASTER_PLAN_V2.md`
4. `docx/Reference_plan/ZEROARM_MCU_AUDIT_MATRIX_V2.yaml`
5. Design 和 Code Reference 中当前模块章节
6. `docx/Reference_project/` 中对应参考源码
7. 当前生产源码和测试，而不是仅依赖旧总结

容易变化的测试数量、固件大小、ST-Link、COM 端口和工作区状态必须重新探测。

## 3. Dirty 工作区规则

交接时已知不属于当前修复提交的状态：

```text
M  .idea/editor.xml
D  AGENTS.md
M  KNOWN_ISSUES.md
?? Tests/test_hardware_readonly.ps1
?? tmp/
```

规则：

- 不恢复或提交根 `AGENTS.md`，除非用户明确要求。
- 不提交 `.idea/editor.xml` 和 `tmp/`。
- `KNOWN_ISSUES.md` 含多轮混合更新，提交前必须拆分审查。
- 硬件脚本必须先审核命令白名单，再决定归档。
- 禁止使用 `git reset --hard`、`git checkout --`、`git clean` 或删除方式制造干净状态。
- 同一文件有多个问题时按 hunk 精确暂存。

## 4. 缺陷处理规范

每个缺陷先建立记录：

```text
id
severity
trigger
old_result
expected_result
root_cause
invariants
production_files
test_files
red_evidence
focused_result
full_result
sanitizer_result
debug_size
release_size
board_commands
commit
remaining_risk
```

专项测试必须命中生产实现。以下内容不能作为修复证据：

- 调用测试桩再断言测试桩返回值。
- 只打印 `FIX-VERIFIED` 而没有断言。
- 没有重新编译的旧可执行文件。
- 只运行一个测试而未运行全量。
- 只通过编译推断状态机或并发正确。

## 5. 主机测试

普通测试：

```powershell
cmake -S Tests -B build/host-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

需要证明不存在旧产物依赖时：

```powershell
cmake --build build/host-tests --target clean
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

严格 ASan/UBSan：

```powershell
& Tests/run_sanitizers.ps1
```

该脚本使用 `.tools/llvm-mingw-20260616-ucrt-x86_64`，从 CTest JSON 动态发现全部
测试，逐个直接运行并检查退出码。新增测试后输出数量应自动增加。

## 6. STM32 构建

Debug：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
arm-none-eabi-size build/Debug/zero_arm_mcu.elf
```

Release：

```powershell
cmake --preset Release
cmake --build --preset Release --parallel
arm-none-eabi-size build/Release/zero_arm_mcu.elf
```

固件正确路径：

```text
build/Debug/zero_arm_mcu.elf
build/Release/zero_arm_mcu.elf
```

根 `build` 下的主机测试可执行文件不是固件。

## 7. 故障和状态语义

已提交故障位：

| Bit | 名称 | 语义 |
|---:|---|---|
| 0 | TARGET_RANGE | 目标越界或目标校验失败 |
| 1 | HOST_TX | UART TX 启动失败 |
| 2 | INTERNAL_STATE | RTOS 锁、状态不变量或内部操作失败 |
| 3 | MOTOR_TX | 电机命令或同步广播发送失败 |
| 4 | MOTOR_FEEDBACK | 电机反馈格式或转换失败 |
| 5 | UART_RX_OVERFLOW | UART 软件字节环丢字节 |
| 6 | CAN_RX_DROP | CAN RX 帧未进入 MotorTask |
| 7 | SERVICE_PARTIAL | 多轴服务只影响部分轴 |
| 8 | FEEDBACK_STALE | 必需反馈超过新鲜度阈值 |
| 9 | STARTUP | 资源、任务、外设或启动状态失败 |

bit0/bit1 保持 V1 兼容。当前仅“定义故障位”不代表检测器全部接入；以状态 YAML
和源码为准。

后续不得把以下概念混为一谈：

```text
requested
accepted
queued
dispatched
motor acknowledged
completed
```

V1 目前只能可靠表达部分阶段；缺失确认必须报告 Unknown，不能伪造成功。

## 8. RTOS 和并发审查

每个关键 CMSIS 调用点至少注入：

```text
osOK
osErrorResource
osErrorTimeout
osErrorParameter
osErrorISR（适用时）
```

检查：

- mutex acquire/release 都处理。
- queue put 成功而 event set 失败时没有永久幽灵消息。
- event wait 最高位错误不能进入普通事件分支。
- ISR 不等待、不解析协议、不调用阻塞接口。
- delay 失败不能产生无退避高速循环。
- STOP/服务优先级高于旧运动目标。
- 缓冲区、队列和循环均有显式上界。

## 9. 协议规则

当前线上格式：

```text
AA LEN CMD PAYLOAD CRC8 55
```

- `LEN` 包含 CMD 和 PAYLOAD。
- CRC 覆盖 CMD 和 PAYLOAD。
- V1 GET_STATE 当前为 60 字节本地小端结构布局。
- 上位机必须显式按偏移解码，不能使用 native ABI。
- V2 未实施前不得改变 V1 命令 ID 或现有响应。

协议修改前先形成审批包，至少包括：

- 字段表、宽度、端序、单位、版本。
- capability/schema 协商。
- sequence、device time、sample sequence。
- accepted/completed 语义。
- V1/V2 回退。
- 黄金帧、fuzz、断线和长稳结果。

## 10. Git 原子提交

每个缺陷验证完成后：

```powershell
git diff --check
git diff -- <本缺陷文件>
git add -- <本缺陷文件>
git diff --cached --check
git diff --cached --name-status
git diff --cached
git commit -m "<type>(<scope>): <单一结果>"
```

提交前必须确认 cached diff 没有：

- IDE 文件。
- 根 `AGENTS.md` 删除。
- `tmp/`。
- 其他缺陷的半成品。
- 用户自己的无关改动。

不得自动 push、rebase、release 或清理工作区。

## 11. 实板与机械安全

默认允许：

- ST-Link 探测。
- 固件刷写和 verify。
- HELLO。
- GET_STATE。
- 坏 CRC、噪声、截断/坏帧后的只读恢复。

默认禁止：

- SET_JOINT_TARGET。
- ENABLE。
- DISABLE。
- STOP。
- HOME。
- TEACH_START/STOP。

即使目标为零或仅单轴，也不能在机械臂未支撑、方向/零点/限位未确认时发送。

## 12. 当前 ST-Link 阻塞

现场设备：

```text
Tool: STM32CubeProgrammer 2.23.0
ST-Link: V2J46M31
SN: 066CFF494986485067181236
Voltage: 3.27 V
MCU: STM32G47x/G48x/G414, Device ID 0x469
VCP: COM3
```

已观察：

1. Normal/Under Reset 连接返回 `DEV_TARGET_HELD_UNDER_RESET`。
2. HOTPLUG 可以读取芯片并暂停/恢复 core。
3. HOTPLUG 擦除扇区失败。
4. Option Bytes 显示 `RDP=0x0`，工具解释为 Level 1。
5. COM3 正常 HELLO 超时。
6. 未修改 RDP。
7. 诊断结束已执行 `Core run`。

注意：目标持续复位时 Option Bytes 读数可能需要再次确认。正确恢复顺序：

```text
检查 Reset 按钮
 -> 测量 NRST
 -> 检查 ST-Link NRST 是否误接 GND/持续拉低
 -> 暂时断开 NRST 后重新 Normal/HOTPLUG 探测
 -> 再显示 Option Bytes
 -> 仅在确认真实 RDP Level 1 后决定是否降为 Level 0
```

RDP Level 1 降到 Level 0 会整片擦除 Flash并降低保护级别，不能作为普通连接参数
随意尝试。若执行，必须立即重刷经过验证的 ELF、verify、reset，再做只读串口测试。

板上固件目前可能没有完整运行，不能把此前板测结果当作当前固件已刷入。

## 13. 每个单元交付

至少报告：

1. 结果和缺陷 ID。
2. 实际修改文件。
3. 根因、接口和数据流。
4. 专项、全量、Sanitizer、Debug/Release 结果。
5. RAM/FLASH。
6. 板测命令审计。
7. 剩余风险和下一个入口。
8. 独立提交哈希。

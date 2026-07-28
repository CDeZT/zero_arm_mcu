# ZEROARM MCU Agent 交接说明

## 1. 用户的协作习惯

1. 默认使用中文交流，结论优先，更新保持简洁。
2. 必须仔细参考以下设计文档及其中已经跑通的参考源码结构：
   - `docx/Reference_plan/ZEROARM_MCU_FIRMWARE_DESIGN_V1.md`
   - `docx/Reference_plan/ZEROARM_MCU_FIRMWARE_CODE_REFERENCE_V1.md`
   - `docx/Reference_project/`
3. 严格按照 Code Reference 第 16 节的审查单元逐轮构建。
4. 一次只完成一个审查单元。完成后必须停止，列出改动、测试结果、风险和未决问题，等待用户回复“继续”。
5. 对用户而言，`.c/.h` 实现、CMake 集成和成功编译属于同一组操作，不能拆开交付。
6. 每轮都要检查 diff；不要自动提交、暂存、推送或清理工作区。
7. 当前工作区包含用户和此前 Agent 的大量未提交、未跟踪改动。不得使用 `git reset --hard`、`git checkout --` 或删除这些改动。
8. 用户重视实际验证，但机械安全高于进度。没有确认支撑、标定和限位前，不得发送可能造成机械臂运动或重力轴松脱的命令。

推荐每轮流程：

```text
阅读本单元设计文档和参考源码
 -> 说明本轮准备修改的文件和边界
 -> 完成 .h/.c 与 CMake
 -> 主机测试
 -> ASan/UBSan
 -> STM32 交叉编译
 -> 审查 diff
 -> 必要时刷写并执行安全板测
 -> 停止等待人工审查
```

## 2. 当前项目完成状态

Code Reference 审查单元 `0～20` 已完成。

主要已完成能力：

- 应用资源创建以及 HostTask、MotorTask、MotionTask。
- UART Circular DMA 接收、私有字节流、TX DMA 队列。
- 主机协议组帧、解析、CRC8、HELLO、GET_STATE。
- ENABLE、DISABLE、STOP、HOME 未配置回复和六轴目标解码。
- Robot 最新目标快照、服务队列、STOP 目标失效。
- X_V2 CAN 命令打包及发送结果传播。
- FDCAN 扩展 ID、过滤、RX 队列、分包和发送失败上报。
- 六轴电机管理、反馈解析、关节配置校验。
- 关节与电机位置双向转换。
- 20 ms 在线限速轨迹和整组六轴范围检查。
- PC 六轴目标到 MotionTask、MotorTask 的数据链路。
- 六轴位置帧全部发送成功后才广播同步运动；任一轴失败时禁止同步广播。
- 单元 20 新增 `ROBOT_STATE_TEACHING`、`TEACH_START/STOP` 服务和请求接口。
- `TEACH_START` 会先让当前运动目标失效；服务优先级为：
  - STOP：255
  - TEACH_START/STOP：200
  - 普通服务：0

单元 20 的主要文件：

- `Robot/Inc/robot_types.h`
- `Robot/Inc/robot.h`
- `Robot/Src/robot.c`
- `Tests/test_robot.c`
- `Tests/CMakeLists.txt`
- 根目录 `CMakeLists.txt`

单元 20 验证结果：

- 普通主机测试：8/8 通过。
- AppleClang ASan/UBSan：8/8 通过。
- 根 CMake `test_robot` 自定义目标通过。
- STM32 Debug 交叉编译通过。
- RAM：22968 B / 128 KiB，17.52%。
- FLASH：51984 B / 512 KiB，9.92%。
- 已通过 OpenOCD 刷写并校验。
- 连续 5 轮板上 HELLO/GET_STATE 均返回：
  - `ZEROARM/1.0`
  - `ROBOT_STATE_READY`
  - `fault_flags=0`
- 板卡当前运行单元 20 固件。
- 单元 20 板测没有发送目标、使能、失能、STOP 或 TEACH 命令，没有触发运动或松轴。

## 3. 下一步：只做审查单元 21

下一个 Agent 收到用户“继续”后，只实现 Code Reference 第 16 节的单元 21：

```text
主要文件：motor_manager.c/.h 及对应测试
目标：接入 TEACH 服务的 STOP、20 ms 实时位置返回、选定轴失能和最终位置同步
审查门：只有在机械臂可靠支撑的条件下，才验证失能后位置反馈
```

必须重点阅读：

- Design `10.1 拖动示教可行性`
- Code Reference `11.7 拖动示教骨架`
- M_Project 的：
  - `App/Src/X_V2.c`
  - `App/Src/state_machine.c`

单元 21 预期数据流：

```text
TEACH_START
 -> 停止选定轴
 -> 丢弃 Motor 待发送目标
 -> 每个选定轴启动 20 ms S_CPOS/0x36 返回
 -> 失能选定轴
 -> Robot 状态切换为 TEACHING

0x36 有符号电机实时位置
 -> motor_urad
 -> motor_to_joint_position()
 -> Robot actual_joint_urad

TEACH_STOP
 -> 停止定时位置返回
 -> 将最终 actual 同步为新的参考目标
 -> Robot 状态切换为 READY
 -> 保持电机失能，不自动锁轴
```

不要原样复制参考 `state_machine.c` 的 `0x36` 处理。必须解析手册规定的正负号字节，再按 X 固件默认的 `0.1°` 单位转换。

## 4. 尚未完成和已知风险

- 六轴真实电机地址唯一性尚未做完整实机确认。
- 电机方向、机械零点、减速比、软限位仍是保守配置，尚未完成装机标定。
- GET_STATE 当前发送原始 C 结构体，存在 padding、端序和协议版本兼容风险。
- 六轴 actual state 尚未完整接入关节反馈；这是单元 21 的一部分。
- 无完整 Homing；限位未配置时必须继续明确返回 `ROBOT_ERR_NOT_CONFIGURED`。
- 无 Cartesian、速度、力矩控制接口。
- 面向未来 PC 具身模型，目前只有基础关节目标、STOP 和状态查询链路；缺少时间戳、命令序号回显、速度/电流/在线状态、能力查询和稳定的版本化状态协议。
- 参考项目的 TD3 模型只连接 MuJoCo/Stable Baselines3，没有可直接复用的真实机械臂串口或 MQTT observation/action 协议。
- J2/J3 等重力轴失能后可能快速下落。未确认机械支撑、配重或制动前，禁止执行真实 TEACH_START。
- 手册没有明确保证失能后 `0x36` 仍持续更新，这必须在安全台架条件下验证。
- TEACH_STOP 后不得自动使能；还需验证重新使能不会追赶旧目标。
- 当前无完整故障恢复/清错状态机。非法目标会置 `ROBOT_FAULT_TARGET_RANGE`。

## 5. 构建和验证命令

主机普通测试：

```sh
cmake -S Tests -B build/host-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

主机 ASan/UBSan：

```sh
cmake -S Tests -B build/host-tests-sanitize \
  -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_C_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
cmake --build build/host-tests-sanitize --parallel
ctest --test-dir build/host-tests-sanitize --output-on-failure
```

STM32 Debug 交叉编译必须使用项目预设：

```sh
cmake --preset Debug
cmake --build --preset Debug --parallel
```

正确固件路径：

```text
build/Debug/zero_arm_mcu.elf
```

不要把根目录的 `build` 当作嵌入式构建目录；它可能是 CLion/macOS 主机编译配置。嵌入式结果以 `build/Debug` 为准。

板卡连接信息：

```text
ST-Link V2
VCP: /dev/cu.usbmodem1303
OpenOCD: /opt/homebrew/bin/openocd
```

刷写命令：

```sh
/opt/homebrew/bin/openocd \
  -f interface/stlink.cfg \
  -f target/stm32g4x.cfg \
  -c "program build/Debug/zero_arm_mcu.elf verify reset exit"
```

OpenOCD 和串口访问可能需要用户批准提升权限。

## 6. 板测安全要求

可默认执行的安全测试：

- HELLO
- GET_STATE
- 多轮只读通信稳定性
- 固件刷写和 verify

未经明确安全确认不得执行：

- 非零 SET_JOINT_TARGET
- ENABLE
- TEACH_START
- 任何可能造成运动、松轴或重力轴下落的命令

即使用户说“板子已经插上”，也只代表允许连接测试，不代表机械臂已经获得可靠支撑或允许松轴。

## 7. 每轮交付格式

完成单元后至少报告：

1. 本轮结果和停止审查点。
2. 实际修改文件。
3. 主要接口和数据流。
4. 普通测试、消毒器测试、STM32 编译和板测结果。
5. FLASH/RAM 使用量。
6. 文档偏差、风险和未决问题。
7. 明确说明板测是否发送过运动、使能或松轴命令。

随后停止，不要自动进入下一单元。

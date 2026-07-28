# ZeroArm Desktop Agent 实施规约

## 0. Agent角色

本文件是交给“其他编码Agent”的执行规约。规划Agent只负责维护需求、任务卡、
审批和审查；收到用户明确的编码启动句后，编码Agent才进入实施单元。

编码Agent不得要求用户重新描述已经写入本文档的产品背景。它应自主读取文档、
检查仓库并完成当前单元；只有会改变协议方案、机械安全范围或外部发布权限的
关键选择才向用户请求确认。

## 1. 任务入口

本文件适用于 `UpperComputer/` 下全部工作，并继承仓库根 `AGENTS.md`。后续
Agent收到类似“一句话启动”时，必须把它解释为：

```text
读取本文件及其指定文档
 -> 判断当前审查单元
 -> 仅完成该单元
 -> 实现代码、测试、构建和文档
 -> 检查diff
 -> 停止等待审查
```

一句话推荐：

```text
严格按照 UpperComputer/AGENTS.md、PROJECT_SPEC.yaml 和 docs/16_AGENT_EXECUTION_MANIFEST.yaml，从 IMPLEMENTATION_STATUS.md 标记的首个未完成上位机单元开始，完成本单元全部实现、测试、集成、文档和独立 Git 提交后停止报告，禁止绕过协议审批与机械安全门。
```

这句话不授权跳过协议审批、机械安全门、推送或外部发布。独立提交仅按用户已经
确认的“一问题一原子提交”执行，且不得包含其他dirty文件。

## 2. 开始前必须阅读

每轮主 Agent 必须亲自阅读：

1. `UpperComputer/PROJECT_SPEC.yaml`。
2. `UpperComputer/docs/05_IMPLEMENTATION_PLAN.md` 中当前单元。
3. 与当前单元直接相关的产品、架构、GUI、测试或安全文档。
4. `UpperComputer/docs/14_DELEGATED_CODING_AGENT_BRIEF.md`。
5. `UpperComputer/docs/15_IMPLEMENTATION_BLUEPRINT.md`。
6. `UpperComputer/docs/16_AGENT_EXECUTION_MANIFEST.yaml` 当前单元。
7. `UpperComputer/docs/17_API_DATA_AND_FIXTURE_CONTRACTS.md` 相关接口。
8. 根目录：
   - `docx/Reference_plan/ZEROARM_MCU_FIRMWARE_DESIGN_V1.md`
   - `docx/Reference_plan/ZEROARM_MCU_FIRMWARE_CODE_REFERENCE_V1.md`
9. 涉及协议时读取当前：
   - `Protocol/Inc/protocol.h`
   - `Protocol/Src/protocol.c`
   - `Protocol/Src/messages.c`
   - `Robot/Inc/robot_types.h`
10. 涉及3D/运动学时读取 `docs/09_REFERENCE_ASSET_AUDIT.md` 和实际URDF。

不得仅依赖旧Agent总结；容易变化的构建、测试、串口和工作区状态要现场验证。

## 3. 当前规划基线

规划单元完成时的事实：

- 上位机尚未开始编码。
- 技术栈：Python + PySide6。
- 首发Transport：Mock + Serial。
- 当前MCU V1协议可用于兼容开发。
- 协议V2仅为提案，状态是 pending。
- 机械臂尚未组装，真实动作验收 blocked。
- 参考项目已在仓库 `docx/Reference_project/zero-robotic-arm-master/`。
- 当前MCU V1 GET_STATE为60字节本地结构布局。
- 当前MotionTask周期20 ms，连续目标上限默认50 Hz。
- MCU协议组帧边界修复提交：`653b994`。
- MCU轨迹到达后停止重复发布提交：`ebb271c`。

如现场源码与上述不同，以源码和测试为准，并在交付中说明差异。

## 4. 用户和产品偏好

- 默认中文交流，结论优先。
- 面向个人使用，不做账号、角色和云权限。
- 页面和诊断能力宁可完整，不做只能发几个按钮命令的玩具GUI。
- 当前电脑性能充足，但刷新率由串口、协议、MCU队列和实际测试决定。
- Windows优先，同时保持核心、Transport和资源路径跨平台。
- 最终必须能打包成无需系统Python的Windows免安装包，建议同时有安装包。
- Mock必须支持无板开发、演示和自动故障注入。

## 5. 强制边界

### 5.1 一次一个审查单元

- 严格按 `docs/05_IMPLEMENTATION_PLAN.md`。
- 一次只做一个单元或用户明确指定的一个子单元。
- `.py`实现、测试、项目集成、资源配置和成功构建不可拆开交付。
- 完成本单元后停止，等待用户“继续”。
- 不提前创建后续页面的虚假实现；允许创建当前架构需要的最小接口。

### 5.2 协议修改

- V1上位机兼容实现不需要修改MCU。
- 单元24只产出审批包，不改MCU。
- 只有用户明确回复“确认实施协议V2”才可进入单元25。
- 任何额外协议字段、命令ID、波特率和重试语义同样先审批。
- MCU修改仍受根 `AGENTS.md` 的主机、sanitizer、交叉编译、diff和板测流程约束。

### 5.3 机械安全

默认可执行：

- Mock全部测试。
- HELLO、GET_STATE、未来只读能力/PING。
- 固件烧录和verify。
- 无动作的连接、断线、CRC、噪声和长稳板测。

未经当前轮明确确认不得发送：

- ENABLE、DISABLE、STOP、HOME。
- TEACH_START/STOP。
- SET_JOINT_TARGET。
- 轨迹、Cartesian、手柄或Recipe动作。

“可以最终做出来”不代表机械臂尚未组装时可以真实发送这些命令。

### 5.4 Git和用户改动

- 每轮开始运行 `git status --short`。
- 每轮结束运行 `git diff --check` 和审查相关diff。
- 用户已明确要求“每解决一个问题就独立Git提交”。当前Agent必须只stage本单元
  文件，审查cached diff，然后创建一个原子提交；若用户后续撤销，以最新指令为准。
- 不自动push、clean、stash、rebase或发布。
- 不修改、删除或覆盖用户和其他Agent的无关未提交文件。
- 禁止 `git reset --hard`、`git checkout --` 清理工作区。

## 6. 技术规则

### 6.1 Python

- Python目标 `>=3.12,<3.14`，具体版本在单元0现场验证。
- 使用 `src` layout和`pyproject.toml`。
- 公共函数和Domain模型使用类型标注。
- 金额/角度之外无需滥用复杂抽象；线上整数单位保持精确。
- Domain内部关节位置统一`int` urad；显示边界才转deg/rad。
- 不允许NaN/Inf进入命令或轨迹。
- 不使用裸`except:`；错误必须分类并保留原因。
- 不使用全局可变单例保存设备状态。

### 6.2 PySide6

- GUI线程不执行阻塞串口、数据库批写、批量IK或烧录。
- 跨线程使用Qt queued signal或明确有界队列。
- View不得直接import pyserial、sqlite底层连接或协议字节操作。
- 高频输入由ViewModel节流到渲染频率。
- Widget销毁后断开订阅，避免幽灵更新和泄漏。
- 所有危险按钮都调用同一个Application CommandService/SafetyGate。

### 6.3 Transport

统一契约至少包含：

```python
open()
close()
write(bytes)
events/receive callback
state
statistics
```

- Domain和GUI只依赖接口。
- Mock和Serial必须通过同一contract tests。
- 写队列有界。
- 状态轮询可以合并/丢弃过期项。
- 危险命令超时不得自动重发。
- 自动重连不得自动恢复armed/teach/playback。

### 6.4 Protocol

- codec和stream parser是纯模块，不依赖Qt和串口。
- CRC、端序、长度和线上字段逐项编码。
- V1 GET_STATE按明确偏移读取，不映射本机Python/C结构。
- V1缺失字段用`None/Unknown`，不伪造0。
- 未知enum/result/flag保留原始值供诊断。
- 任何parser对任意字节流不得崩溃、死循环或无界增长。

### 6.5 高频状态

- 当前V1可配置20/50/100 Hz，同时最多一个GET_STATE在途。
- GUI渲染上限60 FPS。
- 当前动作流上限50 Hz，与MCU 20 ms MotionTask一致。
- 曲线使用固定容量ring。
- Recorder批量写入。
- 高波特率和200 Hz只有实机性能单元通过后才能成为默认能力。

### 6.6 3D和运动学

- 参考资产只读。
- 运行时资源复制到 `UpperComputer/resources` 并记录来源哈希。
- 不依赖桌面绝对路径。
- 所有MCU→模型符号、零偏和wrap在`JointModelMapping`集中定义。
- 不在页面代码散落角度补偿。
- J2/J3/J5坐标差异必须通过黄金姿态确认。
- IK未通过黄金测试前只能离线预览，不开放真机发送。

### 6.7 数据

- PC事件排序使用monotonic time。
- 展示同时保存wall clock。
- V2保存device time和sample seq。
- 数据库、轨迹、配置和Recipe均有schema version。
- raw teach recording永不被平滑处理覆盖。
- 删除会话和重置数据库属于显式破坏性动作，需要用户确认。

### 6.8 固件工具

- 使用`QProcess`参数数组调用已验证的STM32CubeProgrammer路径。
- 禁止把用户输入拼成shell字符串。
- 校验固件路径、扩展名、哈希、目标和ST-Link SN。
- 固件正确路径当前是 `build/Debug/zero_arm_mcu.elf`。
- 不得误刷根`build`或主机构建产物。

## 7. 依赖选择

默认候选：

| 用途 | 依赖 |
|---|---|
| GUI | PySide6 |
| Serial | pyserial |
| Charts | pyqtgraph |
| 数值 | numpy、scipy |
| 配置验证 | pydantic |
| 网格 | trimesh |
| 测试 | pytest、pytest-qt、hypothesis |
| 打包 | PyInstaller |

新增依赖前：

1. 说明用途。
2. 检查许可证、Windows wheel、打包体积和维护状态。
3. 能用标准库清楚实现的小功能不盲目加依赖。
4. 更新锁定文件和第三方许可证。

## 8. 推荐每轮流程

```text
读取当前单元和相关文档
 -> git status
 -> 说明本轮文件和边界
 -> 实现代码/测试/集成
 -> lint/type
 -> unit/GUI tests
 -> Mock E2E
 -> 性能或打包测试（适用）
 -> 只读板测（适用）
 -> git diff --check
 -> 审查diff
 -> 更新决策/进度
 -> 停止报告
```

工具命令在单元0确定后写入项目README。未建立项目时不预设虚拟环境路径。

## 9. 测试要求

每轮按风险选取：

- 纯函数单元测试。
- Hypothesis边界/性质测试。
- Transport contract。
- DeviceSession状态机。
- pytest-qt GUI流程。
- Mock端到端。
- 高频性能和有界队列。
- SQLite迁移/故障。
- 打包烟雾。
- 只读板测。

SafetyGate目标分支覆盖100%。不能因覆盖率目标写无意义断言。

若某测试因环境不能运行：

```text
缺少什么
 -> 为什么需要
 -> 尝试修复
 -> 实际替代验证
 -> 剩余可信度
```

不得把“编译了测试”报告为“运行通过”。

## 10. Mock最低能力

Mock设备必须：

- 实现V1，后续实现V2。
- 可按随机种子确定性重放。
- 六轴目标和反馈随时间变化。
- 模拟run state/masks/fault。
- 注入延迟、超时、掉线、CRC、噪声、重复、乱序。
- 模拟Homing和TEACH状态。
- 模拟重连后不恢复动作。
- 提供快慢设备配置用于性能测试。

Mock不能绕过真实Domain/SafetyGate；只替换Transport另一端。

## 11. 状态与进度记录

实施开始后，在 `UpperComputer/docs/IMPLEMENTATION_STATUS.md` 维护：

```text
当前完成单元
版本
主要能力
验证结果
待审批
硬件阻塞
下一单元
```

只记录已验证事实；规划目标不能写成完成状态。

## 12. 每轮交付格式

至少报告：

1. 本轮审查单元及结果。
2. 实际修改文件。
3. 主要接口和数据流。
4. lint/type/unit/GUI/Mock/性能/打包结果。
5. 若板测，列出实际发送的全部命令类别。
6. 协议和文档偏差。
7. 风险、未决问题和审批需求。
8. 当前停止点及下一单元名称。

如果打包，额外报告：

- 产物路径、大小、SHA-256。
- 测试机器。
- 是否依赖系统Python。
- 已知杀毒/驱动/OpenGL问题。

## 13. 判断“完成”的禁止项

不得因为以下情况声称项目完成：

- GUI页面能打开但按钮是stub。
- 只在Mock成功却声称真机通过。
- 测试编译但没有执行。
- 3D模型显示但坐标映射未验证。
- IK有数值输出但没有FK回代和限位测试。
- 生成exe但未在无Python环境启动。
- 协议V2文档写完但未审批、实现和板测。
- 机械臂未组装却把动作验收标记通过。

## 14. 当前下一步

上位机源码尚未创建。其他编码Agent收到README的一句话入口后，只执行
`docs/05_IMPLEMENTATION_PLAN.md` 与manifest的单元0：工程基线。

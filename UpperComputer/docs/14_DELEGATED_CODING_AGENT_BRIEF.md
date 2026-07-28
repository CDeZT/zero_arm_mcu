# 交给编码 Agent 的完整任务书

## 1. 委托目标

你是 ZeroArm Desktop 的编码执行Agent，不是重新做产品访谈的规划Agent。你的
目标是按审查单元，将本目录中的需求最终变成：

具体创建哪些模块、接口签名和fixture，以以下三份实施合同为准：

- `15_IMPLEMENTATION_BLUEPRINT.md`
- `16_AGENT_EXECUTION_MANIFEST.yaml`
- `17_API_DATA_AND_FIXTURE_CONTRACTS.md`

```text
可维护源码
 + 自动测试
 + Mock完整演示
 + 当前V1/未来V2真板通信
 + 3D和运动学
 + 安全控制、轨迹和示教
 + 诊断、日志和固件升级
 + Windows portable和installer
 + 可复现发布证据
```

用户已经确认的产品决策位于`PROJECT_SPEC.yaml`，不得再次询问Windows还是
跨平台、PySide6还是其他栈、是否需要Mock、哪些页面等已决问题。

## 2. 权威来源顺序

遇到冲突按以下顺序处理：

1. 当前用户明确指令。
2. 仓库根`AGENTS.md`。
3. `UpperComputer/AGENTS.md`。
4. `PROJECT_SPEC.yaml`及`10_DECISIONS_APPROVALS.md`。
5. 产品/安全需求。
6. 技术架构和GUI规格。
7. 实施计划。
8. 参考项目源码。

参考项目是输入证据，不是规范。旧文本协议、F407/EMM行为和continuous URDF
限位不得覆盖当前G474/X_V2事实。

## 3. 第一次接手检查

编码前输出简短说明并执行只读检查：

```text
git status --short
rg --files UpperComputer
读取IMPLEMENTATION_STATUS
确认当前单元
确认Python/CMake/Qt环境
确认无关dirty文件
```

若工作区已有其他改动：

- 保留。
- 仅修改当前单元文件。
- 重叠时先读diff，能兼容则继续。
- 不能安全区分时才向用户报告。

不得清理`tmp/`、IDE文件、未跟踪模型或其他Agent改动。

## 4. 工程级交付标准

每个实现都必须同时具备：

- 类型清晰的接口。
- 正常路径。
- 输入边界。
- 错误传播。
- 取消/关闭路径。
- 测试。
- 日志但不泄漏任意原始隐私路径。
- 与项目构建/打包集成。

禁止：

- `pass`、`TODO`或总返回成功作为交付。
- 页面按钮只弹“未实现”却标记单元完成。
- 为通过测试在生产代码加入测试专用分支。
- 测试中重新实现一份生产逻辑并只测试副本。
- 只用Mock截图替代自动断言。
- 把未知状态转成0或READY。

## 5. 核心接口契约

具体语法可在单元实现时微调，但职责不得混淆。

### Transport

输入：串口/Mock配置和bytes。

输出事件：

- opened。
- bytes_received。
- write_completed或write_failed。
- closed。
- transport_error。
- statistics。

Transport不知道HELLO、RobotSnapshot或JointTarget。

### FrameCodec/StreamParser

输入：纯bytes或单个byte/chunk。

输出：

- 完整frame。
- 可分类frame error。
- parser统计。

不得调用Qt、串口、数据库或GUI。

### DeviceSession

输入：

- Transport事件。
- Application request。
- monotonic clock。

输出：

- ConnectionState。
- RobotSnapshot。
- request result/unknown outcome。
- diagnostics event。

Session不拥有Widget。

### CommandService

输入Domain command，通过SafetyGate，交给DeviceSession。它是GUI、终端、手柄、
轨迹和Recipe的唯一动作入口。

### SnapshotBus/TelemetryBuffer

- 发布不可变快照。
- latest value用于数值和3D。
- 固定容量ring用于曲线。
- Recorder得到独立有界流。
- 消费者慢不能反压Communication Worker到无界增长。

### ResourceLocator

统一处理开发环境、portable和installer资源路径。页面不得写相对`../../STL`。

## 6. 单元0详细任务卡

### 输入

- 规划文档已完成。
- 上位机源码尚不存在。

### 必须创建

```text
UpperComputer/pyproject.toml
UpperComputer/src/zeroarm_desktop/__init__.py
UpperComputer/src/zeroarm_desktop/__main__.py
UpperComputer/src/zeroarm_desktop/bootstrap.py
UpperComputer/src/zeroarm_desktop/version.py
UpperComputer/src/zeroarm_desktop/gui/main_window.py
UpperComputer/tests/test_app_smoke.py
UpperComputer/tests/conftest.py
UpperComputer/README_DEVELOPMENT.md
```

依赖锁定文件采用实施时确认的工具，但必须提交可复现结果。不得把本机venv提交。

### 最小GUI

- QApplication由bootstrap创建。
- MainWindow有应用名、版本和“尚未连接”状态。
- 支持`python -m zeroarm_desktop`。
- 关闭时正常退出，无后台线程。
- 不创建虚假连接和控制按钮。

### 工具

- ruff：format + lint。
- pytest + pytest-qt。
- 类型检查选择mypy或pyright，并说明理由。
- 项目元数据、Python范围、入口点和Windows标识。

### 验收

```text
从UpperComputer目录安装开发依赖
ruff format --check
ruff check
type check
pytest
应用启动烟雾
git diff --check
```

完成后创建/更新`IMPLEMENTATION_STATUS.md`，停止；不得进入协议单元。

## 7. 单元1～6通信核心任务卡

### 单元1

产物：

- `protocol/crc8.py`。
- `protocol/frame_codec.py`。
- `protocol/stream_parser.py`。
- `tests/protocol/test_frame_codec.py`。
- `tests/protocol/test_stream_parser.py`。
- Hypothesis测试。

关键断言：

- `AA 01 00 00 55`为合法HELLO请求。
- 任意chunk拆分结果一致。
- parser缓存有严格上限。
- CRC错误不触发业务frame。
- 任意bytes不抛未分类异常。

### 单元2

产物：

- V1命令/结果enum（允许Unknown wrapper）。
- `V1Codec`。
- `RobotSnapshot`基础模型。
- 60字节GET_STATE黄金fixture。

关键断言：

- 六轴i32状态按V1小端明确解析。
- 六轴目标按BE编码为28字节。
- payload长度为1的GET_STATE可能是错误result，60才是成功状态。
- 不把V1缺失online/current/velocity设为0。

### 单元3

产物：

- Transport抽象。
- MockTransport。
- MockDevice。
- FakeClock。
- 故障场景模型和contract test。

MockDevice必须真正吃入V1 bytes并返回frame，不允许直接把RobotSnapshot塞给GUI。

### 单元4

产物：

- SerialTransport worker。
- SerialConfig。
- PortDescriptor/discovery。
- fake serial backend。

验证：

- 打开失败。
- read取消。
- close期间事件。
- unplug。
- write queue full。
- 重复open/close。

### 单元5

产物：

- DeviceSession状态机。
- Handshake/Negotiation。
- V1 RequestTracker。
- TelemetryScheduler。
- ReconnectPolicy。

验证所有状态迁移和危险命令超时不重发。

### 单元6

产物：

- pydantic配置。
- SQLite migrations。
- repositories/recorder。
- CSV/JSON exporter。

验证数据库损坏、磁盘写失败、迁移、批量性能和导出回读。

## 8. GUI单元任务卡

| 单元 | 必须交付 | 不得遗漏 |
|---:|---|---|
| 7 | MainWindow、导航、状态栏、主题 | 固定STOP区域只接安全stub，不发Transport |
| 8 | 连接页和握手时间线 | Mock/Serial同流程 |
| 9 | 总览、六轴表、曲线 | Unknown显示、100 Hz性能 |
| 12 | 3D工作区 | ghost/actual、资源路径、映射debug |
| 14 | 手动关节 | hold-to-run、Arm、Mock E2E |
| 15 | 轨迹编辑 | undo/redo、raw不覆盖、联动 |
| 17 | 示教页面 | 安全清单、recording/review状态机 |
| 18 | Cartesian | 多解和奇异性，默认预览 |
| 19 | 标定/Homing | 逐轴，不盲改六轴 |
| 20 | 诊断/终端 | 默认只读，HEX与解析树 |
| 21 | 手柄/Recipe | 统一SafetyGate |
| 22 | 数据集 | Episode和时间源 |
| 23 | 固件升级 | 固定参数QProcess、哈希、verify |

每个页面至少有：

- ViewModel测试。
- pytest-qt关键流程。
- 禁用/错误/断线状态。
- 高频更新不泄漏订阅。
- 键盘和窗口关闭路径。

## 9. 3D与运动学任务卡

### 资产

不要从桌面复制外层目录。使用仓库参考路径并运行复制工具，只纳入：

- 规范URDF。
- 7个运行时STL。
- 来源许可证。
- asset manifest/hash。

### 映射

建立表：

```text
joint index
MCU sign
MCU zero
model sign
model zero
wrap policy
MCU limit
URDF limit
source/revision
```

J2/J3/J5不能凭视觉猜。先生成可调映射和黄金姿态报告，未确认项明确pending。

### IK

- 可借鉴MATLAB公式，不能逐行无验证移植。
- 输入输出坐标和角度偏置显式。
- 每个解做FK回代。
- 过滤limit。
- 返回不可达/奇异而不是NaN。
- 真机Cartesian仍等待审批。

## 10. 危险动作实现规则

动作代码可以先在Mock实现。对真实Serial：

- Action按钮调用CommandService。
- SafetyGate检查。
- UI显示最终wire意图。
- 用户当前轮审批。
- Command audit落库。
- 结果区分Denied/Accepted/Completed/Rejected/UnknownOutcome。

测试协议终端不得提供绕过SafetyGate的`serial.write`按钮。

## 11. 协议V2协作

到单元24时，编码Agent只能写审批包。它应以
`13_MCU_REMEDIATION_PLAN.md`为问题基线，但需要现场复核源码。

用户未批准时：

- 上位机保持V1。
- V2代码可以有纯fixture/设计试验，但不能修改MCU或标记完成。
- 不擅自占用命令ID。

用户批准后，MCU修复和上位机V2仍分别作为独立审查单元交付。

## 12. 打包任务卡

portable至少验证：

- 无系统Python。
- 从含空格和中文目录启动。
- Mock完整演示。
- Qt平台/OpenGL插件。
- 7个STL。
- 配置和数据库可写。
- `--safe-mode`。
- 固件工具缺失时可诊断。

生成：

- ZIP。
- SHA-256。
- 第三方许可证。
- 构建日志。
- 测试证据。

不得自动上传。

## 13. Handoff要求

每个编码Agent结束时，除了用户报告，还应更新状态文件，使下一个Agent无需读
聊天记录即可继续：

```text
last_completed_unit
files
interfaces
commands_run
test_counts
performance
hardware_commands_sent
approvals_pending
known_issues
next_unit
```

结论必须区分：

- 自动测试通过。
- Mock通过。
- 只读真板通过。
- 动作真机未测试/通过。

## 14. 最终完成标准

最终“完成上位机”必须同时拥有：

1. 功能需求实现或明确硬件延期。
2. 自动测试证据。
3. Mock完整流程。
4. 真实板只读长稳。
5. 3D/FK/IK黄金验证。
6. 经批准的动作测试。
7. portable和installer。
8. 用户手册、诊断和已知限制。
9. 需求追踪矩阵。
10. 不依赖规划Agent聊天上下文。

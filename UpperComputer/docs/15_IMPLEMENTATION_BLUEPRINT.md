# ZeroArm Desktop 精确实现蓝图

## 1. 文档地位

本文把产品需求转换为可直接创建代码的工程蓝图。编码Agent必须同时遵守：

1. `UpperComputer/AGENTS.md`的执行、安全和审查规则。
2. `PROJECT_SPEC.yaml`的固定产品决策。
3. `05_IMPLEMENTATION_PLAN.md`的单元顺序。
4. `16_AGENT_EXECUTION_MANIFEST.yaml`的每单元文件和验证清单。
5. `17_API_DATA_AND_FIXTURE_CONTRACTS.md`的接口、模型和fixture合同。

发生冲突时优先级为：现场源码事实与根安全规则、用户当前明确指令、
`UpperComputer/AGENTS.md`、`PROJECT_SPEC.yaml`、本文、其他说明文档。

## 2. 最终交付形态

最终不是单个脚本，而是一个可测试、可打包、无系统Python依赖的桌面应用：

```text
ZeroArmDesktop.exe
 -> GUI/Application层
 -> DeviceSession
 -> V1/V2 Protocol Codec
 -> SerialTransport或MockTransport
 -> ZeroArm MCU
```

无板卡时必须能以Mock模式演示全部页面和安全流程；有板卡但机械臂未组装时，
必须能完成连接、HELLO、GET_STATE、诊断、记录和固件刷新，不发送动作命令。

## 3. 目标源码树

编码Agent按当前单元渐进创建，不得一次生成空壳：

```text
UpperComputer/
├── pyproject.toml
├── README.md
├── AGENTS.md
├── PROJECT_SPEC.yaml
├── src/zeroarm_desktop/
│   ├── __init__.py
│   ├── __main__.py
│   ├── bootstrap.py
│   ├── version.py
│   ├── domain/
│   │   ├── models.py
│   │   ├── enums.py
│   │   ├── errors.py
│   │   ├── units.py
│   │   ├── trajectory.py
│   │   ├── safety.py
│   │   └── kinematics.py
│   ├── protocol/
│   │   ├── crc8.py
│   │   ├── frame.py
│   │   ├── stream_parser.py
│   │   ├── v1_codec.py
│   │   ├── v2_codec.py
│   │   └── fixtures.py
│   ├── transport/
│   │   ├── base.py
│   │   ├── mock.py
│   │   ├── serial.py
│   │   ├── discovery.py
│   │   └── statistics.py
│   ├── application/
│   │   ├── device_session.py
│   │   ├── command_service.py
│   │   ├── safety_gate.py
│   │   ├── playback.py
│   │   ├── teach.py
│   │   ├── recorder.py
│   │   ├── firmware_service.py
│   │   └── event_bus.py
│   ├── infrastructure/
│   │   ├── config.py
│   │   ├── database.py
│   │   ├── migrations.py
│   │   ├── export.py
│   │   ├── paths.py
│   │   └── logging.py
│   ├── model3d/
│   │   ├── assets.py
│   │   ├── urdf_loader.py
│   │   ├── joint_mapping.py
│   │   ├── fk.py
│   │   ├── ik.py
│   │   └── scene.py
│   └── gui/
│       ├── app.py
│       ├── main_window.py
│       ├── navigation.py
│       ├── theme.py
│       ├── dialogs.py
│       ├── viewmodels/
│       ├── widgets/
│       └── pages/
├── tests/
│   ├── unit/
│   ├── property/
│   ├── contract/
│   ├── integration/
│   ├── gui/
│   ├── e2e/
│   ├── performance/
│   ├── hardware_readonly/
│   └── fixtures/
├── resources/
│   ├── icons/
│   ├── themes/
│   ├── robot/
│   └── licenses/
├── tools/
│   ├── copy_reference_assets.py
│   ├── verify_assets.py
│   ├── board_readonly_test.py
│   └── build_portable.ps1
└── packaging/
    ├── zeroarm_desktop.spec
    └── zeroarm_desktop.iss
```

依赖方向强制为：

```text
domain <- protocol/transport/application/model3d <- gui
domain <- infrastructure adapters
```

`domain`不得导入Qt、pyserial、sqlite连接、STM32工具或页面模块。

## 4. 应用组合根

只有`bootstrap.py`可以组装具体实现。推荐入口：

```python
def create_runtime(
    *,
    mode: AppMode,
    data_root: Path | None = None,
    transport_factory: TransportFactory | None = None,
) -> Runtime: ...

def create_qt_application(
    argv: Sequence[str],
    runtime: Runtime,
) -> tuple[QApplication, MainWindow]: ...

def main(argv: Sequence[str] | None = None) -> int: ...
```

`Runtime`持有session、command service、safety gate、recorder、配置和事件总线，
并提供有序`start()`/`shutdown()`。窗口不得自行创建串口或数据库。

关闭顺序：

```text
取消playback/teach/hold-to-run
 -> SafetyGate撤销Arm上下文
 -> Session停止轮询并断开
 -> Transport worker退出并join
 -> Recorder flush/commit
 -> 数据库关闭
 -> Qt窗口销毁
```

任何阶段失败都继续尝试后续清理，最后聚合错误写日志。

## 5. 并发和线程模型

### GUI线程

只执行：

- Widget绘制和交互。
- ViewModel接收不可变快照。
- 轻量输入校验和命令意图创建。
- 60 FPS以内渲染节流。

禁止阻塞串口、等待响应、数据库批写、批量IK、网格加载和固件进程等待。

### I/O worker

负责：

- 串口open/read/write/close。
- 增量parser喂入。
- 有界写队列。
- 连接错误和统计发布。

同一Transport实例最多一个读循环和一个有序写通道。close必须能取消读并在
规定超时内结束worker。

### Session调度

负责握手、V1单在途请求、超时、轮询、重连和快照合并。可使用Qt worker thread
或标准线程，但接口不得泄漏线程实现。

### Recorder worker

使用有界队列和批量事务。控制/STOP事件优先于普通遥测；遥测过载可以按明确策略
丢样并计数，禁止无限增长。

### CPU任务池

批量IK、轨迹处理和资产验证使用可取消任务。结果回GUI前必须检查请求generation，
过期结果直接丢弃。

## 6. 核心数据流

### 只读状态

```text
Session定时器
 -> GET_STATE request
 -> Protocol encoder
 -> Transport.write
 -> stream bytes
 -> parser完整Frame
 -> V1 decoder
 -> RobotSnapshot
 -> EventBus
 -> Recorder + ViewModels
 -> 60 FPS合并渲染
```

当前V1最多一个请求在途。新轮询tick遇到在途请求时跳过并增加
`poll_coalesced_count`，不得排队追赶。

### 危险命令

```text
Widget意图
 -> ViewModel规范化单位
 -> CommandService
 -> SafetyGate.evaluate
 -> Preview显示wire意图
 -> 当前Arm/确认上下文
 -> Protocol encoder
 -> Session发送
 -> Accepted/Rejected/Timeout
 -> 后续状态观察
 -> CommandOutcome
 -> 审计落库
```

协议终端、手柄、Recipe和测试工具不能绕开此路径。

## 7. GUI页面实现合同

每个页面采用`PageWidget + PageViewModel`，Widget只依赖ViewModel。页面必须具有
稳定`objectName`供pytest-qt定位。

| 页面 | objectName | ViewModel最低输入 | 主要输出 |
|---|---|---|---|
| 启动/连接 | `page_connection` | ports/session/link stats | connect/disconnect/select transport |
| 总览 | `page_dashboard` | RobotSnapshot/faults | navigation/readonly refresh |
| 六轴监控 | `page_joint_monitor` | snapshot/sample ring | plot range/export marker |
| 手动关节 | `page_manual_joint` | snapshot/calibration/safety | preview/arm/hold/send/stop |
| Cartesian | `page_cartesian` | mapping/FK/IK result | offline target preview |
| 示教 | `page_teach` | teach state/snapshot | select mask/start/stop/save |
| 轨迹编辑 | `page_trajectory` | versioned trajectory | edit/resample/validate |
| 标定/Homing | `page_calibration` | config/capability | wizard intents/config diff |
| 诊断 | `page_diagnostics` | counters/errors/events | diagnostic bundle |
| 协议终端 | `page_protocol_console` | parsed/raw frames | safe form request |
| 日志导出 | `page_logs` | sessions/query | export/annotation/delete confirm |
| 固件升级 | `page_firmware` | probes/process events | select/verify/flash |
| 设置 | `page_settings` | validated config | apply/reset/export |
| 3D工作区 | `page_workspace_3d` | scene snapshot/ghost | view controls/pick preview |
| 手柄 | `page_gamepad` | device axes/safety | hold-to-run intent |
| 数据集 | `page_dataset` | episode records | label/export |
| Recipe | `page_recipes` | recipe schema/run state | validate/arm/run/abort |

固定全局组件：

- `connection_badge`
- `firmware_badge`
- `snapshot_age_badge`
- `fault_badge`
- `mode_selector`
- `global_stop_button`
- `notification_center`

`global_stop_button`在GUI Shell阶段可以存在，但只有CommandService和SafetyGate完成后
才能接入真命令；未接入时必须明确显示“尚未启用”，不能伪装成功。

## 8. ViewModel更新规则

ViewModel对外发布不可变`ViewState`。高频Snapshot写入latest slot；Qt渲染timer
以最多60 FPS读取。页面隐藏或销毁后停止timer并解除订阅。

通用状态字段：

```python
@dataclass(frozen=True, slots=True)
class PageStatus:
    busy: bool
    can_interact: bool
    headline: str
    detail: str | None
    last_error: AppError | None
```

异步操作必须有operation id。只允许当前operation id更新完成状态，防止旧连接、
旧IK或旧导出结果覆盖新请求。

## 9. 连接与Session状态机

状态：

```text
DISCONNECTED
 -> OPENING
 -> HANDSHAKING
 -> READONLY_READY
 -> OPERATOR_READY
 -> RECONNECT_WAIT
 -> CLOSING
 -> DISCONNECTED
```

任意活动状态可进入`FAULTED`，但用户disconnect仍应最终进入DISCONNECTED。

关键规则：

- 握手先HELLO，再GET_STATE。
- V1固件默认READONLY_READY。
- OPERATOR_READY要求显式切换模式、状态新鲜、SafetyGate可用。
- 自动重连只恢复端口和只读轮询。
- 重连后mode回Observer，Arm上下文、hold、teach和playback全部撤销。
- V1响应无法用seq关联，因此只能有一个请求在途。
- 超时后的危险命令结果为UnknownOutcome，不自动重发。

## 10. 安全状态

Arm上下文不是永久设置：

```python
@dataclass(frozen=True, slots=True)
class ArmContext:
    token: UUID
    command_family: CommandFamily
    allowed_joint_mask: int
    created_monotonic_ns: int
    expires_monotonic_ns: int
    support_confirmed: bool
    calibration_hash: str
    snapshot_generation: int
```

连接变化、FAULT、状态过期、模式切换、窗口失焦、设备睡眠、校准变化或到期均撤销。
STOP也属于危险硬件命令，但应在软件优先级上高于普通动作；不能称作物理急停。

## 11. 性能和背压

有界容量在配置中集中定义，初始建议：

| 队列/缓存 | 初始容量 | 满载策略 |
|---|---:|---|
| Transport写队列 | 64帧 | STOP保留槽；普通命令拒绝 |
| Parser输入chunk | 64 KiB上限 | 分块消费，不累计无限buffer |
| GUI latest snapshot | 1 | 覆盖旧值 |
| Plot ring | 每轴60秒×100 Hz | 覆盖最旧 |
| Recorder事件队列 | 20,000 | 普通遥测降采样并计数 |
| Protocol frame history | 10,000 | 覆盖最旧 |
| Notification history | 1,000 | 覆盖最旧 |

容量必须能配置并有边界测试。不得用“电脑性能好”作为无界队列理由。

## 12. 错误分类

最低分类：

```text
ConfigurationError
TransportOpenError
TransportPermissionError
TransportDisconnected
TransportQueueFull
ProtocolLengthError
ProtocolCrcError
ProtocolFramingError
ProtocolDecodeError
RequestTimeout
UnknownCommandOutcome
SafetyDenied
CalibrationMissing
SnapshotStale
CapabilityMissing
DatabaseError
AssetError
KinematicsError
FirmwareToolError
Cancelled
InternalError
```

错误对象包含稳定code、用户消息、技术detail、cause、recoverable、context和时间。
GUI可以折叠技术细节，但诊断包必须保留。

## 13. 配置层级

```text
代码安全默认值
 <- 安装包默认配置
 <- 用户配置文件
 <- 当前会话临时覆盖
```

用户配置放`platformdirs`确定的位置，不能写portable程序目录。配置至少包含
schema_version、transport、轮询Hz、GUI FPS、plot窗口、数据保留、主题、路径、
SafetyGate阈值和日志级别。

以下内容不得持久化为true：operator armed、support confirmed、teach active、
playback active、hold-to-run active。

## 14. 代码质量门

每单元结束至少执行：

```powershell
python -m ruff check UpperComputer
python -m ruff format --check UpperComputer
python -m mypy UpperComputer/src
python -m pytest UpperComputer/tests -q
```

项目创建后由单元0把实际命令写入README和manifest。若选择pyright替代mypy，
只能在单元0记录一次决定并统一使用，不能两套规则漂移。

额外门按单元启用：

- 协议：Hypothesis和黄金fixture。
- GUI：pytest-qt与offscreen测试。
- 性能：输出机器、P50/P95/P99、内存和丢样计数。
- 打包：无Python环境烟雾测试。
- 板测：完整命令类别审计。

## 15. 禁止的“快速实现”

- 一个`main.py`同时包含UI、串口和协议。
- 页面直接`serial.write()`。
- `struct.unpack("@...")`读取GET_STATE。
- 用0替代V1不存在或未接入的状态。
- 用`time.time()`调度轨迹。
- 串口线程直接更新Widget。
- 自动重连后恢复运动。
- 危险命令超时自动重试。
- 只创建页面标题和按钮就标记功能完成。
- 将参考项目、`node_modules`或桌面外层目录整体复制。
- 打包后不在无Python环境启动。

## 16. 编码Agent启动算法

收到README中的一句话后：

```text
1. 读根AGENTS和UpperComputer/AGENTS
2. 读PROJECT_SPEC和IMPLEMENTATION_STATUS
3. 从manifest找第一个status=pending且依赖完成的单元
4. 读该单元在05计划和相关专题文档
5. 现场检查环境、源码和dirty工作区
6. 声明本轮文件、边界和不会执行的危险动作
7. 写测试并证明旧实现缺失/失败
8. 实现、集成、运行适用验证
9. 更新状态与追踪
10. 审查diff
11. 若当前用户已明确要求独立提交，仅提交本单元文件
12. 停止并报告
```

若无法判断当前单元，以`IMPLEMENTATION_STATUS.md`为准；状态文件与源码冲突时，
以源码和实际测试为准并先修正文档，不从头重做已完成单元。

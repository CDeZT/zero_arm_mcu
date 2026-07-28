# 技术架构

## 1. 总体结构

采用可测试的分层桌面架构：

```text
PySide6 Views
    |
ViewModels / Controllers
    |
Application Services
    |
Domain Model + SafetyGate
    |
DeviceSession
    |
Protocol Codec
    |
Transport Interface
    +-- SerialTransport
    +-- MockTransport
    +-- TCPTransport future
    +-- MQTTTransport future
```

旁路服务：

```text
TelemetryBuffer -> Plot/View/3D
SessionRecorder -> SQLite -> CSV/JSON
FirmwareService -> STM32CubeProgrammer
AssetPipeline -> URDF/STL -> RobotModel
```

禁止 View、3D、手柄或脚本直接访问 `pyserial`。

## 2. 建议包结构

```text
src/zeroarm_desktop/
├── __main__.py
├── bootstrap.py
├── version.py
├── domain/
│   ├── models.py
│   ├── commands.py
│   ├── capabilities.py
│   ├── faults.py
│   ├── trajectory.py
│   └── safety.py
├── protocol/
│   ├── crc8.py
│   ├── frame_codec.py
│   ├── stream_parser.py
│   ├── v1_codec.py
│   ├── v2_codec.py
│   └── golden_vectors.py
├── transport/
│   ├── base.py
│   ├── serial_transport.py
│   ├── mock_transport.py
│   └── discovery.py
├── session/
│   ├── device_session.py
│   ├── request_tracker.py
│   ├── telemetry_scheduler.py
│   └── reconnect_policy.py
├── services/
│   ├── command_service.py
│   ├── trajectory_service.py
│   ├── teach_service.py
│   ├── firmware_service.py
│   ├── export_service.py
│   └── diagnostics_service.py
├── kinematics/
│   ├── robot_model.py
│   ├── mapping.py
│   ├── fk.py
│   ├── ik.py
│   └── validation.py
├── storage/
│   ├── database.py
│   ├── migrations/
│   ├── repositories.py
│   └── recorder.py
├── gui/
│   ├── main_window.py
│   ├── navigation.py
│   ├── status_bar.py
│   ├── pages/
│   ├── widgets/
│   ├── viewmodels/
│   └── themes/
├── input/
│   └── gamepad.py
├── automation/
│   ├── recipe_schema.py
│   ├── validator.py
│   └── runner.py
└── resources/
    └── locator.py
```

## 3. 核心Domain模型

### DeviceCapabilities

- protocol major/minor。
- state schema。
- firmware/build/hardware version。
- joint count。
- feature bits。
- supported telemetry rates。
- limits/config read-write能力。

### RobotSnapshot

- PC monotonic timestamp。
- PC wall clock。
- device timestamp或None。
- sample sequence或None。
- run state。
- target/actual六轴，内部统一urad。
- enabled/homed/moving/online masks，未知允许None。
- fault flags。
- freshness和source（V1/V2/Mock）。

模型不可变，跨线程传递完整快照，不共享可变数组。

### CommandEnvelope

- command ID/type。
- request seq。
- creation/deadline time。
- payload。
- danger classification。
- accepted/completed语义。
- retry policy。

### Trajectory

- schema version。
- metadata。
- time vector，严格递增。
- 六轴位置。
- 可选速度/加速度。
- source（manual/teach/import）。
- raw parent ID和processing history。

## 4. 线程模型

### GUI主线程

- Qt事件。
- 页面渲染、轻量ViewModel更新。
- 60 FPS以内3D与曲线。
- 不做阻塞IO。

### Communication Worker

- 独立QThread。
- 串口读写、stream parser、请求超时。
- 将完整事件通过queued signal发到应用层。
- 写队列有界，STOP具有最高优先级，状态轮询可丢弃过期任务。

### Recorder Worker

- 批量写SQLite。
- 例如每50条或100 ms提交一次。
- 有界缓存；原始帧丢失必须计数并报警，不能静默。

### Firmware Worker

- 使用 `QProcess` 调用固定可执行文件和参数数组。
- 流式捕获stdout/stderr。
- 烧录期间DeviceSession断开且禁止其他动作。

### Kinematics Worker（按需）

普通六轴FK可以在主线程轻量执行；批量轨迹IK/验证进入线程池，结果带任务ID，
取消后的旧结果不得覆盖新任务。

## 5. 通信状态机

```text
DISCONNECTED
 -> DISCOVERING
 -> OPENING
 -> HANDSHAKING
 -> NEGOTIATING
 -> SYNCHRONIZING
 -> READY_READONLY / READY_CONTROL
 -> DEGRADED
 -> RECONNECT_WAIT
 -> DISCONNECTED
```

规则：

- 自动重连只恢复观察，不恢复ENABLE、示教或轨迹回放。
- 连接更换后清空请求跟踪和旧状态缓存。
- V1请求默认一次在途，避免无seq响应错误关联。
- 连续超时采用降频、DEGRADED、重连三级策略。
- 危险命令超时状态为UNKNOWN_OUTCOME，不得自动重发。

## 6. 高频数据策略

| 数据 | 生产频率 | 消费策略 |
|---|---:|---|
| V1 GET_STATE | 20/50/100 Hz可配 | 同时最多一个在途 |
| V2 telemetry | 目标100～200 Hz | sequence检测丢样 |
| GUI数值 | 最多30～60 Hz | latest snapshot |
| 3D | 最多60 FPS | latest snapshot |
| 曲线 | 采样100～200 Hz | 固定容量ring |
| SQLite | 批量10～20次/s | batch |
| 目标命令 | 最多50 Hz | latest target |

采用backpressure：

- 状态展示只保留最新值。
- 曲线保留固定时间窗，例如60秒。
- 录制使用有界批量队列，并公开丢弃统计。
- 轨迹播放按单调时钟选取当前应发送点，不补发错过的历史点。

## 7. SafetyGate

所有危险命令通过一个纯Domain服务：

```text
Command
 + ConnectionState
 + RobotSnapshot
 + Capabilities
 + CalibrationStatus
 + UserArmingContext
 -> Allow / Deny(reason, remediation)
```

最少检查：

- 连接、协议和状态新鲜度。
- 命令能力。
- 数值有限。
- 六轴范围。
- 最大速度/步长。
- fault/run state。
- 机械组装和标定标志。
- 重力轴失能/示教支撑确认。
- 点动hold-to-run。

协议终端、手柄、Recipe和GUI按钮共用SafetyGate，不能各写一套。

## 8. 数据库建议

SQLite表：

```text
schema_migrations
sessions
connection_events
raw_frames
robot_snapshots
commands
command_results
fault_events
trajectories
trajectory_samples
teach_recordings
annotations
diagnostic_counters
```

关键索引：

- `(session_id, monotonic_ns)`。
- `(session_id, sample_seq)`。
- `(command_id, request_seq)`。
- `(trajectory_id, sample_index)`。

数据库写失败不能阻止STOP发送，但必须立即提示并停止“正在录制”的成功状态。

## 9. 配置

配置采用版本化TOML或JSON：

- GUI主题和窗口布局。
- 端口历史和刷新率。
- 显示单位。
- 图表通道。
- 游戏手柄映射。
- 机械模型映射。
- 本地工具路径。

禁止保存：

- 隐式“机械安全已永久确认”。
- 上次运行中的ENABLE/Teach/Playback armed状态。
- 自动重发危险命令。

## 10. 异常模型

异常分层：

- `TransportError`：打开、读写、掉线。
- `FrameError`：长度、CRC、ETX、超时。
- `ProtocolError`：未知响应、schema不兼容。
- `DeviceRejected`：明确ROBOT_ERR。
- `UnknownOutcome`：危险请求已发送但响应超时。
- `SafetyDenied`：本地主动拒绝。
- `StorageError`、`AssetError`、`FirmwareToolError`。

GUI展示可执行的恢复建议，同时日志保留完整堆栈；不能用一个“通信错误”吞掉
所有原因。

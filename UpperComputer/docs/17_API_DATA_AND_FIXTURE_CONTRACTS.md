# API、数据模型与测试 Fixture 合同

## 1. 原则

本文定义跨模块稳定边界。实现可以使用同步或异步内部机制，但公开语义、单位、
错误和线程归属必须保持一致。所有线上关节位置使用有符号整数微弧度`urad`；
UI边界才转换为度或弧度。

## 2. 基础枚举

```python
class AppMode(Enum):
    OBSERVER = "observer"
    DEBUG = "debug"
    OPERATOR = "operator"

class LinkState(Enum):
    CLOSED = "closed"
    OPENING = "opening"
    OPEN = "open"
    CLOSING = "closing"
    FAILED = "failed"

class SessionState(Enum):
    DISCONNECTED = "disconnected"
    OPENING = "opening"
    HANDSHAKING = "handshaking"
    READONLY_READY = "readonly_ready"
    OPERATOR_READY = "operator_ready"
    RECONNECT_WAIT = "reconnect_wait"
    CLOSING = "closing"
    FAULTED = "faulted"

class Knowledge(Enum):
    UNKNOWN = "unknown"
    COMMANDED = "commanded"
    ACCEPTED = "accepted"
    CONFIRMED = "confirmed"

class OutcomeStatus(Enum):
    DENIED = "denied"
    REJECTED = "rejected"
    ACCEPTED = "accepted"
    COMPLETED = "completed"
    FAILED = "failed"
    TIMEOUT = "timeout"
    UNKNOWN_OUTCOME = "unknown_outcome"
    CANCELLED = "cancelled"
```

未知固件枚举不得强制转换失败。线上枚举模型保存`raw_value`并提供可选known值。

## 3. 不可变领域模型

```python
JointVector = tuple[int, int, int, int, int, int]

@dataclass(frozen=True, slots=True)
class FirmwareIdentity:
    hello_text: str
    protocol_generation: int
    schema_version: int | None
    build_id: str | None
    capabilities: int | None

@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    generation: int
    received_monotonic_ns: int
    received_wall_utc: datetime
    device_time_ms: int | None
    sample_sequence: int | None
    run_state_raw: int
    target_joint_urad: JointVector
    actual_joint_urad: JointVector
    enabled_mask: int | None
    homed_mask: int | None
    moving_mask: int | None
    online_mask: int | None
    teach_mask: int | None
    fault_flags_raw: int
    velocity_urad_s: JointVector | None
    current_ma: JointVector | None

@dataclass(frozen=True, slots=True)
class JointTarget:
    joint_urad: JointVector
    duration_ms: int
    gripper_u16: int

@dataclass(frozen=True, slots=True)
class ProtocolFrame:
    command: int
    payload: bytes
    raw: bytes
    received_monotonic_ns: int

@dataclass(frozen=True, slots=True)
class LinkStatistics:
    bytes_rx: int
    bytes_tx: int
    frames_rx: int
    frames_tx: int
    crc_errors: int
    framing_errors: int
    decode_errors: int
    request_timeouts: int
    reconnects: int
    queue_rejections: int
    poll_coalesced: int
```

V1中mask当前虽然在线上有字节，但MCU尚未完成真实闭环；上位机Domain可保存raw
值，同时通过知识状态标识“reported but not confirmed”，GUI不能把0写成“已确认失能”。

## 4. 当前 V1 线协议

帧：

```text
AA | LEN | CMD | PAYLOAD | CRC8 | 55
LEN = 1 + payload字节数
CRC8覆盖CMD+PAYLOAD
CRC = Dallas/Maxim reflected polynomial 0x8C, initial 0x00
```

当前命令：

| ID | 名称 | 请求payload | 响应payload |
|---:|---|---|---|
| 0x00 | HELLO | 空 | ASCII `ZEROARM/1.0` |
| 0x01 | GET_STATE | 空 | 60字节本地结构布局 |
| 0x02 | ENABLE | joint mask 1B | result 1B |
| 0x03 | DISABLE | joint mask 1B | result 1B |
| 0x04 | STOP | 空 | result 1B |
| 0x05 | SET_JOINT_TARGET | 28B | result 1B |
| 0x06 | HOME | joint mask 1B | result 1B |
| 0x07 | TEACH_START | joint mask 1B | result 1B |
| 0x08 | TEACH_STOP | 空 | result 1B |
| 0x09 | CLEAR_FAULT | 空 | result 1B |

SET_JOINT_TARGET请求：

```text
offset 0..23: J1..J6，每轴BE int32 urad
offset 24..25: BE uint16 duration_ms
offset 26..27: BE uint16 gripper_u16
```

V1 GET_STATE仅对当前STM32G4 GCC构建按以下偏移兼容：

```text
0..3    native little-endian int32 run_state enum
4..27   6 × native little-endian int32 target urad
28..51  6 × native little-endian int32 actual urad
52      enabled_mask
53      homed_mask
54      moving_mask
55      reserved
56..59  native little-endian uint32 fault_flags
```

解码器必须验证payload恰好60字节，使用显式`int.from_bytes(..., "little")`或
标准尺寸`struct.unpack_from("<i")`，不得使用native ABI格式。

当前result：

```text
0 OK
1 ERR_ARGUMENT
2 ERR_STATE
3 ERR_RANGE
4 ERR_NOT_READY
5 ERR_NOT_CONFIGURED
6 ERR_QUEUE_FULL
7 ERR_IO
8 ERR_NOT_IMPLEMENTED
```

响应result仅说明请求函数返回值，不等于电机动作完成或反馈确认。

## 5. Transport 合同

```python
class Transport(Protocol):
    @property
    def state(self) -> LinkState: ...

    @property
    def statistics(self) -> LinkStatistics: ...

    def open(self, settings: TransportSettings) -> None: ...
    def close(self, timeout_s: float = 2.0) -> None: ...
    def write(self, data: bytes, *, priority: WritePriority) -> WriteReceipt: ...
    def subscribe_bytes(self, callback: Callable[[bytes], None]) -> Subscription: ...
    def subscribe_state(
        self, callback: Callable[[LinkStateEvent], None]
    ) -> Subscription: ...
```

合同语义：

- `open`和`close`在各自稳定状态幂等。
- callback不在GUI线程保证执行。
- `write`只表示进入Transport写队列，不表示串口完成或设备接受。
- 输入`bytes`在write返回后由Transport持有不可变副本。
- 队列满返回/抛出稳定`TransportQueueFull`，不能静默丢弃。
- close撤销订阅前先停止新callback，并在超时内结束worker。

MockTransport与SerialTransport必须运行同一组contract tests。

## 6. Protocol 接口

```python
class FrameCodec(Protocol):
    def encode(self, command: int, payload: bytes = b"") -> bytes: ...
    def decode_complete(self, raw: bytes) -> ProtocolFrame: ...

class StreamParser:
    def feed(self, chunk: bytes, *, received_monotonic_ns: int) -> list[ProtocolFrame]: ...
    def reset(self) -> None: ...
    @property
    def statistics(self) -> ParserStatistics: ...

class V1CommandCodec:
    def hello_request(self) -> bytes: ...
    def get_state_request(self) -> bytes: ...
    def decode_hello(self, frame: ProtocolFrame) -> FirmwareIdentity: ...
    def decode_state(self, frame: ProtocolFrame, *, generation: int) -> RobotSnapshot: ...
    def encode_joint_target(self, target: JointTarget) -> bytes: ...
    def encode_joint_mask(self, command: int, mask: int) -> bytes: ...
    def encode_empty_command(self, command: int) -> bytes: ...
    def decode_result(self, frame: ProtocolFrame) -> WireResult: ...
```

`StreamParser.feed`对任意chunk保持有界。非法LEN立即重置；CRC或ETX错误计数后
恢复等待STX。单元1必须用随机分块证明粘包、拆包结果一致。

## 7. DeviceSession 合同

```python
class DeviceSession(Protocol):
    @property
    def state(self) -> SessionState: ...
    @property
    def latest_snapshot(self) -> RobotSnapshot | None: ...
    @property
    def identity(self) -> FirmwareIdentity | None: ...

    def connect(self, settings: ConnectionSettings) -> OperationId: ...
    def disconnect(self) -> OperationId: ...
    def set_poll_rate_hz(self, value: int) -> None: ...
    def request(self, request: DeviceRequest) -> RequestHandle: ...
    def subscribe_snapshots(
        self, callback: Callable[[RobotSnapshot], None]
    ) -> Subscription: ...
    def subscribe_events(
        self, callback: Callable[[SessionEvent], None]
    ) -> Subscription: ...
```

`RequestHandle`最低能力：

```python
request_id
created_monotonic_ns
cancel()
done()
result(timeout=None)
```

V1只允许一个请求在途。轮询不得阻塞危险命令；准备发危险命令时暂停新轮询，
等待当前只读请求完成或超时，再发送一次危险请求。

## 8. SafetyGate 与命令服务

```python
class SafetyGate(Protocol):
    def evaluate(
        self,
        intent: CommandIntent,
        context: SafetyContext,
    ) -> SafetyDecision: ...

class CommandService(Protocol):
    def preview(self, intent: CommandIntent) -> CommandPreview: ...
    def arm(self, request: ArmRequest) -> ArmContext: ...
    def disarm(self, reason: str) -> None: ...
    def execute(
        self,
        preview: CommandPreview,
        arm: ArmContext,
    ) -> CommandHandle: ...
```

`SafetyDecision`包含：

```python
allowed: bool
denials: tuple[SafetyReason, ...]
warnings: tuple[SafetyWarning, ...]
normalized_intent: CommandIntent | None
required_confirmation_text: str | None
```

SafetyGate必须纯计算、确定性、无UI弹窗和无串口副作用。CommandService负责检查
preview哈希、Arm token、snapshot generation和校准哈希仍匹配，再允许编码。

## 9. Recorder 与数据库接口

```python
class Recorder(Protocol):
    def start_session(self, metadata: SessionMetadata) -> SessionId: ...
    def append(self, event: RecordEvent) -> None: ...
    def annotate(self, annotation: Annotation) -> None: ...
    def flush(self, timeout_s: float = 5.0) -> None: ...
    def close(self) -> None: ...

class SessionRepository(Protocol):
    def list_sessions(self, query: SessionQuery) -> Page[SessionSummary]: ...
    def stream_events(self, session_id: SessionId) -> Iterator[StoredEvent]: ...
    def export(self, request: ExportRequest) -> ExportResult: ...
    def delete(self, session_id: SessionId, confirmation: DeleteToken) -> None: ...
```

最低SQLite表：

```text
schema_info(version, applied_utc)
sessions(id, started_utc, ended_utc, app_version, firmware, protocol, metadata_json)
events(id, session_id, monotonic_ns, wall_utc, device_time_ms, sample_seq,
       kind, command, payload_json, raw_blob, severity)
snapshots(event_id, run_state, target_blob, actual_blob, masks_json, fault_flags)
commands(event_id, intent_json, preview_hash, arm_token, outcome, result_raw)
annotations(id, session_id, monotonic_ns, wall_utc, text, tags_json)
drop_counters(session_id, kind, count)
```

迁移必须事务化并备份未知较新schema；不能直接删除用户库重建。

## 10. 轨迹、回放和示教

```python
@dataclass(frozen=True, slots=True)
class TrajectoryPoint:
    time_ns: int
    joint_urad: JointVector
    gripper_u16: int | None

@dataclass(frozen=True, slots=True)
class Trajectory:
    schema_version: int
    name: str
    points: tuple[TrajectoryPoint, ...]
    source: str
    metadata: Mapping[str, JsonValue]

class PlaybackEngine(Protocol):
    def validate(self, trajectory: Trajectory, context: SafetyContext) -> ValidationReport: ...
    def start(self, trajectory: Trajectory, arm: ArmContext) -> PlaybackId: ...
    def pause(self) -> None: ...
    def resume(self) -> None: ...
    def abort(self, reason: str) -> None: ...
```

调度使用monotonic clock；迟到点直接跳过并计数，不以burst追赶。V1最高50 Hz。
示教raw点只追加，不原地平滑；processed轨迹另存并记录父raw哈希。

## 11. 3D 与运动学

```python
class JointModelMapping(Protocol):
    def robot_to_model_rad(self, robot_urad: JointVector) -> tuple[float, ...]: ...
    def model_to_robot_urad(self, model_rad: Sequence[float]) -> JointVector: ...

class Kinematics(Protocol):
    def forward(self, joint_urad: JointVector) -> Pose: ...
    def inverse(self, pose: Pose, seed: JointVector) -> tuple[IkSolution, ...]: ...
```

映射由每轴`sign`、`offset_rad`、wrap和模型joint name组成，集中加载并带schema版本。
IK解必须FK回代、软限位过滤、奇异性标识并按距seed排序。黄金姿态未确认前，
IK结果只能进入ghost。

## 12. 固件工具合同

```python
class FirmwareService(Protocol):
    def probe_tools(self) -> tuple[FirmwareToolProbe, ...]: ...
    def inspect(self, path: Path) -> FirmwareArtifact: ...
    def flash(self, request: FlashRequest) -> ProcessHandle: ...
```

`FlashRequest`必须含artifact绝对路径、SHA-256、ST-Link SN、目标MCU、verify和reset。
进程使用参数数组，无shell。输出按原始bytes保存，并单独解码展示；退出0且出现
verify成功证据后才算刷写成功，reset后必须HELLO复核。

## 13. Qt 信号边界

GUI adapter可以定义：

```python
class QtRuntimeBridge(QObject):
    session_state_changed = Signal(object)
    snapshot_available = Signal(object)
    link_statistics_changed = Signal(object)
    notification_available = Signal(object)
    operation_progress = Signal(object)
    command_outcome = Signal(object)
```

Signal payload为不可变领域对象。不得发送会被worker继续修改的list/dict。

## 14. 标准 Fixture

必须建立以下稳定ID，测试引用ID而非复制魔法字节：

| ID | 内容 |
|---|---|
| `V1-HELLO-REQ` | `AA 01 00 00 55` |
| `V1-HELLO-RSP` | 当前HELLO完整响应 |
| `V1-STATE-READY-ZERO` | 60字节READY、mask0、fault0 |
| `V1-RESULT-OK-CMD02` | ENABLE命令result OK帧 |
| `V1-TARGET-MIXED-SIGNS` | 正负六轴、duration、gripper |
| `V1-BAD-CRC` | 单bit CRC损坏 |
| `V1-BAD-ETX` | ETX损坏 |
| `V1-LEN-ZERO` | LEN=0 |
| `V1-LEN-128` | 当前解析器拒绝边界 |
| `STREAM-NOISE-SPLIT` | 噪声+拆分帧+粘连帧 |
| `MOCK-DISCONNECT-DURING-READ` | 只读请求中断线 |
| `MOCK-DISCONNECT-DURING-ACTION` | 危险命令未知结果 |
| `SAFETY-STALE-SNAPSHOT` | 超过新鲜度阈值 |
| `SAFETY-GRAVITY-UNSUPPORTED` | 重力轴未支撑 |
| `TRAJ-LATE-POINTS` | 回放迟到且禁止追赶 |
| `DB-MIGRATION-N-1` | 上一schema数据库 |

黄金帧由独立参考实现或实际板卡抓取得到，并记录来源、日期、固件commit和SHA-256。

## 15. Contract Test 命名

```text
test_transport_contract__*
test_protocol_property__*
test_session_transition__*
test_safety_gate__*
test_v1_fixture__*
test_recorder_failure__*
test_gui_<page>__*
test_performance_<scenario>__*
test_hardware_readonly__*
```

硬件只读测试必须在收集端拒绝危险command ID，而不只是约定调用者不要发送。

## 16. 完成证据对象

每单元在状态文档记录：

```yaml
unit:
commit:
files:
interfaces_added:
commands_run:
tests:
  passed:
  failed:
performance:
hardware:
  connected:
  commands_sent:
  action_commands_sent: false
approvals_pending:
known_limitations:
next_unit:
```

没有实际运行的测试写`not_run`和原因，不能写pass。

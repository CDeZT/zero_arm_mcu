# 分阶段实施计划

## 1. 执行规则

- 一次只完成一个审查单元。
- `.py`实现、测试、依赖/打包配置和成功构建属于同一单元。
- 每单元结束检查diff并停止。
- MCU协议单元必须先提交方案，用户确认后另开实施单元。
- 危险板测由安全门控制，不因软件完成而自动执行。

## 2. 完成定义

每单元至少满足：

```text
实现完整
静态检查通过
单元/GUI测试通过
Mock端到端通过（适用时）
可启动应用
文档同步
diff审查
风险和未决问题已列出
```

## 3. 审查单元

### 单元 0：工程基线

目标：

- 创建 `pyproject.toml`、`src/`、`tests/`、resources和入口。
- 建立虚拟环境和锁定依赖策略。
- 创建最小PySide6窗口。
- 配置ruff、mypy/pyright选择、pytest、pytest-qt。
- 提供开发启动脚本。

审查：

- 干净环境安装。
- GUI启动/关闭测试。
- 无串口或模型也可启动。

### 单元 1：纯协议基础

- Dallas/Maxim CRC8。
- V1 frame encoder。
- 增量stream parser。
- 长度、CRC、ETX、噪声和分包测试。
- Hypothesis随机字节流测试。
- 官方黄金帧向量。

不得依赖PySide6或pyserial。

### 单元 2：V1命令Codec

- HELLO、GET_STATE、动作result和SET_JOINT_TARGET。
- 当前60字节状态逐偏移解码，不使用Python原生struct ABI。
- 大端目标、小端V1状态差异写入测试。
- 未知enum/result保留原始数值。

### 单元 3：Transport契约与Mock

- `Transport`接口。
- 确定性Mock。
- 内存双工字节通道。
- 正常、延迟、掉线、CRC、重复、噪声和截断场景。
- Transport contract tests。

### 单元 4：Serial Transport和发现

- pyserial worker。
- 端口枚举、VID/PID描述。
- 打开/关闭/取消读。
- 有界写队列和错误传播。
- 使用pseudo/loopback或fake serial测试。

本单元不发送任何真实危险命令。

### 单元 5：DeviceSession

- 完整连接状态机。
- V1握手。
- 请求超时、V1单在途。
- 自动重连只恢复只读。
- RobotSnapshot发布。
- 链路健康统计。

### 单元 6：配置、日志和SQLite

- 配置schema与迁移。
- Session数据库和批量Recorder。
- 原始帧、状态、命令和连接事件。
- CSV/JSON导出。
- 数据库故障注入和恢复。

### 单元 7：GUI Shell

- MainWindow、导航、状态栏、主题和错误展示。
- 页面占位由真实路由管理。
- 固定STOP区域但尚不接真机动作。
- 窗口布局保存（不保存armed状态）。

### 单元 8：连接页

- Mock/Serial选择。
- 端口发现。
- 握手时间线。
- 固件/能力显示。
- 自动重连设置。
- pytest-qt流程测试。

### 单元 9：总览与六轴监控

- 快照ViewModel。
- 六轴数值卡。
- 链路和故障卡。
- pyqtgraph环形数据源。
- 100 Hz输入、60 FPS渲染性能测试。

### 单元 10：参考资产管线

- URDF和STL只读导入。
- package URI解析。
- 网格存在、包围盒、哈希和许可证清单。
- 运行时资源复制脚本。
- 不依赖桌面路径。

### 单元 11：运动学映射与FK

- 集中 `JointModelMapping`。
- MCU urad到模型rad映射。
- FK。
- URDF/MATLAB/MuJoCo黄金姿态。
- 坐标系和末端状态。

J2/J3/J5映射未确认前不得假设。

### 单元 12：3D工作区

- STL渲染。
- 实际和ghost模型。
- 坐标轴、视角、轨迹tail。
- 60 FPS性能基线。
- 截图和headless可测试的数据层。

### 单元 13：SafetyGate

- 危险命令分类。
- 状态新鲜度、能力、限位和速度检查。
- Arm上下文和超时。
- 重力轴警告。
- GUI、手柄、终端、Recipe统一入口。
- 100%分支覆盖目标。

### 单元 14：手动关节控制（Mock）

- 单轴/六轴目标。
- relative/absolute。
- hold-to-run。
- ghost预览。
- STOP快捷键。
- Mock端到端。

真实板卡只读测试，不执行动作。

### 单元 15：轨迹Domain与编辑器

- 版本化Trajectory schema。
- 验证、插值、平滑、重采样、速度缩放。
- undo/redo command。
- 表/图/时间线/3D联动。
- 性质测试和大轨迹性能测试。

### 单元 16：回放引擎（Mock）

- monotonic scheduler。
- 50 Hz最高发送。
- 不补发过期点。
- pause/stop/abort。
- 断线、FAULT、误差过大注入。
- Mock硬实时近似测试。

### 单元 17：拖动示教（Mock）

- 选择掩码、安全清单、状态机。
- 录制device/PC时间。
- raw/processed分离。
- review和另存轨迹。
- TEACH_STOP后保持失能UI。

### 单元 18：Cartesian预览和IK

- 导入或重新实现经验证IK。
- 多解、不可达、奇异性。
- 当前姿态最近解。
- 批量黄金测试。
- 只开放离线预览。

真实Cartesian发送必须另设审查门。

### 单元 19：标定与Homing向导（Mock）

- 逐轴步骤。
- 配置diff、导入导出、审计。
- V1未配置语义。
- Mock限位和超时。

### 单元 20：诊断、协议终端和诊断包

- 原始帧解析树。
- 请求跟踪。
- 错误计数。
- 安全表单式终端。
- 一键诊断包。

### 单元 21：手柄与Recipe（Mock）

- 设备枚举、轴映射、死区、缩放。
- hold-to-run和失焦停止。
- Recipe schema、静态检查、runner。
- 所有动作通过SafetyGate。

### 单元 22：数据集接口

- Episode。
- observation/action/result。
- 时间源字段。
- JSON/CSV/NPZ导出选项。
- 模型动作默认只进入Mock。

### 单元 23：固件升级工具

- 固定参数方式调用STM32CubeProgrammer。
- 文件和目标校验。
- 日志、verify、reset后HELLO。
- 使用测试进程模拟成功/失败输出。

### 单元 24：协议 V2 审批包

只提交：

- 最终字段和命令表。
- MCU改动文件。
- 兼容/回退。
- 黄金帧。
- 资源影响。
- 测试和烧录计划。

停止，等待用户明确确认；本单元不得修改MCU。

### 单元 25：协议 V2 MCU实现

仅在审批后：

- 线上逐字段编码。
- capabilities、seq、device time、fast state。
- parser timeout。
- 主机测试和ASan/UBSan。
- STM32交叉编译。
- 只读刷写验证。

按 MCU 工程交接规则独立停止审查。

### 单元 26：上位机 V2

- 能力协商。
- V1回退。
- seq tracker。
- 主动遥测。
- 诊断字段。
- Mock双版本兼容矩阵。

### 单元 27：高波特率审批与性能

- 提交460800/921600方案。
- 用户批准后改固件。
- 串口误码、长稳、CPU、队列和实际Hz测试。
- 达不到指标即回退115200。

### 单元 28：打包

- PyInstaller spec。
- runtime资源。
- portable ZIP。
- 版本信息和许可证。
- 干净Windows环境烟雾测试。

### 单元 29：安装包

- Inno Setup。
- 安装、升级、卸载。
- 用户数据保留规则。
- 不自动安装未知驱动。

### 单元 30：只读板测

- HELLO、能力、状态。
- 多轮连接/断开。
- CRC、噪声、DMA环绕。
- 长稳运行。
- 确认无动作类命令。

### 单元 31：真实动作验收

仅在机械臂组装、标定、急停和支撑确认后，拆成：

- 31A 单轴低速小角度和STOP。
- 31B 六轴同步目标。
- 31C Homing。
- 31D TEACH失能反馈。
- 31E 轨迹回放。
- 31F Cartesian目标。

每个子单元单独限定轴、角度、速度和停止条件。

### 单元 32：最终发布验收

- 需求追踪矩阵全部结论化。
- 自动测试、性能、只读板测和允许的动作测试。
- portable和installer哈希。
- 用户手册、故障排查、已知限制。
- 最终版本标签/发布需用户另行授权。

## 4. 推荐里程碑

| 里程碑 | 单元 | 可交付 |
|---|---|---|
| M1 通信核心 | 0～6 | CLI/测试可连V1和Mock |
| M2 可视化 | 7～12 | GUI监控、曲线、3D |
| M3 控制与轨迹 | 13～18 | Mock完整操作 |
| M4 工程工具 | 19～23 | 标定、诊断、升级、数据 |
| M5 协议与性能 | 24～27 | 经批准V2和高频遥测 |
| M6 发布 | 28～30 | 可分发应用和只读板测 |
| M7 真机验收 | 31 | 机械安全条件满足后 |
| M8 完成 | 32 | 最终验收包 |

## 5. 工期风险而非承诺

功能范围较大，实施应按里程碑评估，不用“页面数量”代表完成度。最容易低估的
工作是：坐标映射、IK黄金测试、轨迹异常恢复、真实机械安全和干净环境打包。

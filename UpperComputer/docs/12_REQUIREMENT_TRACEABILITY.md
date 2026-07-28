# 需求追踪基线

## 1. 产品功能到实施单元

| 需求组 | 实施单元 | 主要自动测试 |
|---|---|---|
| FR-CON 连接 | 3～5、8 | contract、session state、pytest-qt |
| FR-STA 状态 | 2、5、9、26 | codec、snapshot、GUI高频 |
| FR-JOG 手动控制 | 13～14、31A/B | SafetyGate、Mock E2E、真机限定测试 |
| FR-KIN 运动学 | 11、18、31F | 黄金姿态、IK→FK、真机延期 |
| FR-3D 三维 | 10～12 | asset、mapping、render性能 |
| FR-TCH 示教 | 17、25/26、31D | Mock状态机、协议、支撑真机测试 |
| FR-TRJ 轨迹 | 15～16、31E | property、scheduler、真机延期 |
| FR-CAL 标定/Homing | 19、25/26、31C | schema、Mock limit、真机延期 |
| FR-DIA 诊断 | 1～6、20、26 | parser、统计、导出 |
| FR-DAT 数据 | 6、15、17、22 | migration、roundtrip、load |
| FR-FWU 固件升级 | 23、30 | process fake、刷写verify、HELLO |
| FR-AUT 手柄/Recipe | 21～22 | input、SafetyGate、runner |

## 2. 非功能需求到证据

| ID | 设计 | 证据 |
|---|---|---|
| NFR-001 | Communication/Recorder/Firmware worker | GUI阻塞与线程测试 |
| NFR-002 | latest snapshot + 60 FPS throttle | PERF-001 |
| NFR-003 | 全部队列有界 | 长稳队列指标 |
| NFR-004 | UnknownOutcome、禁止危险重试 | Session/Safety测试 |
| NFR-005 | Domain/codec无GUI依赖 | 纯pytest |
| NFR-006 | PyInstaller onedir | 干净Windows烟雾 |
| NFR-007 | monotonic + wall + device time | 数据roundtrip |
| NFR-008 | schema migrations | 升级测试 |
| NFR-009 | Optional/Unknown字段 | V1 GUI测试 |
| NFR-010 | command audit | SQLite和诊断导出 |

## 3. 安全需求到证据

| 安全事实 | 实现 | 验证 |
|---|---|---|
| STOP非急停 | 固定文案和用户手册 | GUI文本测试/人工审查 |
| 重力轴失能受控 | SafetyGate + 当前轮审批 | 分支测试 + 31D |
| 不自动恢复动作 | DeviceSession | 重连E2E |
| 点动松手停止 | Input controller | release/focus/disconnect测试 |
| 轨迹不追赶 | Playback scheduler | 延迟注入 |
| 危险超时不重发 | RequestTracker | timeout测试 |
| 模型不宣称安全 | 状态标签 | 人工审查 |

## 4. 最终维护方式

实现每个单元时，将本文件中的“设计”替换或补充为实际模块路径和测试名称。
最终发布不得存在只有需求、没有实现/测试/延期结论的required条目。

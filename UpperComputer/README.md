# ZeroArm 上位机工程总览

## 1. 本目录的作用

本目录是 ZeroArm 桌面上位机的唯一实施入口。它不仅描述 GUI，还约束：

- 当前 MCU V1 协议兼容。
- 协议 V2 的提案、审批和迁移。
- 串口、Mock、未来 TCP/MQTT Transport。
- 六轴状态、手动控制、拖动示教、轨迹、标定和诊断。
- URDF/STL 模型导入、运动学映射和 3D 预览。
- 自动化测试、只读板测、危险板测审批。
- Windows 免安装包和安装包。
- 后续数据采集、脚本自动化和具身模型接口。

上位机源码后续统一放在：

```text
UpperComputer/
├── AGENTS.md
├── PROJECT_SPEC.yaml
├── README.md
├── docs/
├── src/                 # 实施单元 1 创建
├── tests/               # 实施单元 1 创建
├── resources/           # 实施单元 1 创建
├── tools/               # 实施单元 1 创建
└── packaging/           # 打包单元创建
```

## 2. 已确认的产品决策

| 项目 | 已确认选择 |
|---|---|
| 主要平台 | Windows，代码保持跨平台 |
| 技术栈 | Python + PySide6 |
| 产品定位 | 个人使用的调试工具 + 操作控制台 |
| 权限系统 | 不实现账号和角色 |
| Transport | Serial + Mock 首发；预留 TCP/MQTT |
| 3D 资产 | 使用仓库内参考项目的 URDF/STL/D-H/MuJoCo |
| 性能 | 当前协议状态采样最高 100 Hz、GUI 60 FPS；V2 高波特率实测后争取 200 Hz |
| 离线开发 | 必须具备确定性 Mock 和故障注入 |
| 数据 | SQLite 会话库，支持 CSV/JSON 导出 |
| 发布 | Windows 免安装包必须有；安装包建议有 |
| MCU 软件修复 | 所有确认问题逐项修复，每项一个独立 Git 提交 |
| MCU 协议 V2 | 允许提出，但最终字段和实施仍需用户确认 |
| 最终目标 | GUI、测试、板测、文档、打包均完成 |

## 3. 文档阅读顺序

1. `AGENTS.md`：Agent 强制执行规则和实施单元。
2. `PROJECT_SPEC.yaml`：机器可读的范围、性能和审批门。
3. `docs/IMPLEMENTATION_STATUS.md`：确定首个未完成单元，禁止从头重做。
4. `docs/01_PRODUCT_REQUIREMENTS.md`：完整产品需求。
5. `docs/02_TECHNICAL_ARCHITECTURE.md`：模块、线程和数据流。
6. `docs/03_GUI_UX_SPECIFICATION.md`：页面和交互细节。
7. `docs/04_PROTOCOL_V2_PROPOSAL.md`：当前协议问题和 V2 提案。
8. `docs/05_IMPLEMENTATION_PLAN.md`：逐单元实现顺序。
9. `docs/06_TEST_AND_ACCEPTANCE.md`：测试矩阵和验收标准。
10. `docs/07_SAFETY_AND_HARDWARE_GATES.md`：危险动作边界。
11. `docs/08_PACKAGING_RELEASE_OPERATIONS.md`：打包与发布。
12. `docs/09_REFERENCE_ASSET_AUDIT.md`：参考资产和坐标差异。
13. `docs/10_DECISIONS_APPROVALS.md`：决策和待审批事项。
14. `docs/11_AGENT_RUNBOOK_AND_PROMPTS.md`：一句话入口和专项提示词。
15. `docs/12_REQUIREMENT_TRACEABILITY.md`：需求到单元和测试的映射。
16. `docs/13_MCU_REMEDIATION_PLAN.md`：MCU问题分级与修复顺序。
17. `docs/14_DELEGATED_CODING_AGENT_BRIEF.md`：交给其他编码Agent的执行任务书。
18. `docs/15_IMPLEMENTATION_BLUEPRINT.md`：精确目录、线程、页面和数据流蓝图。
19. `docs/16_AGENT_EXECUTION_MANIFEST.yaml`：机器可读的 0～32 实施单元和验收门。
20. `docs/17_API_DATA_AND_FIXTURE_CONTRACTS.md`：接口签名、线上偏移、数据库和 fixture 合同。

## 4. 一句话启动

后续可以在仓库根目录对 Agent 说：

```text
严格按照 UpperComputer/AGENTS.md、PROJECT_SPEC.yaml 和 docs/16_AGENT_EXECUTION_MANIFEST.yaml，从 IMPLEMENTATION_STATUS.md 标记的首个未完成上位机单元开始，完成本单元全部实现、测试、集成、文档和独立 Git 提交后停止报告，禁止绕过协议审批与机械安全门。
```

这句话已经给出完整上下文入口，不需要再次解释技术栈和功能范围。但它不会
绕过以下人工审查门：

- 一次只完成一个审查单元。
- MCU 协议变更实施前确认。
- 使能、失能、Homing、示教和真实运动前确认机械安全条件。
- 安装包签名、自动发布或联网服务等外部动作需单独授权。

## 5. “代码框架完成”与“协议固化”的区别

当前 MCU 框架已经完成了可靠的分层、RTOS 资源、UART/FDCAN、六轴目标和基础
示教链路。这说明固件架构可继续扩展，不等于 PC 产品协议已经冻结。

协议冻结至少要求：

1. 线上的每个字段具有固定宽度、端序、单位和版本。
2. 新旧版本能够协商能力，或明确拒绝不兼容客户端。
3. 每条请求有序号，响应可与请求关联。
4. 状态具备设备时间戳、样本序号和必要诊断字段。
5. “请求已入队”和“动作已完成”有不同语义。
6. 主机过载、超时、重试和重复命令的行为已定义。
7. 兼容性、模糊测试、断线恢复和长期压力测试通过。

当前 V1 是可用的最小协议；V2 提案未获用户审批前，上位机必须先兼容 V1，
不得擅自修改固件。

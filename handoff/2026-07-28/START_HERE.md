# ZEROARM 工作交接入口

更新时间：2026-07-28

## 1. 交付范围

本交接包同时覆盖两个后续工作流：

1. ZeroArm Windows 桌面上位机的委托开发。
2. `zero_arm_mcu` 固件的全面审计、修复、优化和后续实机验收。

上位机和 MCU 可以由不同 Agent 并行推进，但必须共享当前 V1 协议事实。协议 V2
仍是提案；任何一侧都不能把提案字段当成已存在能力。

## 2. 上位机入口

上位机资料的唯一入口是：

```text
UpperComputer/
├── README.md
├── AGENTS.md
├── PROJECT_SPEC.yaml
└── docs/
```

执行者必须先阅读：

1. `UpperComputer/README.md`
2. `UpperComputer/AGENTS.md`
3. `UpperComputer/PROJECT_SPEC.yaml`
4. `UpperComputer/docs/IMPLEMENTATION_STATUS.md`
5. `UpperComputer/docs/14_DELEGATED_CODING_AGENT_BRIEF.md`
6. `UpperComputer/docs/15_IMPLEMENTATION_BLUEPRINT.md`
7. `UpperComputer/docs/16_AGENT_EXECUTION_MANIFEST.yaml`
8. `UpperComputer/docs/17_API_DATA_AND_FIXTURE_CONTRACTS.md`

一句话启动上位机编码：

```text
读取 UpperComputer/README.md、AGENTS.md、PROJECT_SPEC.yaml 和 docs 下全部实施文档，以 IMPLEMENTATION_STATUS.md 为现场基线，从执行 Manifest 的首个未完成单元开始，完成代码、测试、集成、文档和独立提交；保持 MCU V1 兼容，不把 V2 提案当作已实现能力，不执行真实机械动作。
```

当前上位机事实：

- 技术栈为 Python 3.12/3.13 + PySide6。
- 首发 Transport 为 Mock + Serial。
- GUI、协议、3D、轨迹、示教、诊断、记录、固件更新和打包均已规划。
- 上位机源码尚未创建；首个实施单元为工程基线。
- Windows 免安装包是必交付，安装包为建议交付。

## 3. MCU 入口

MCU 交接资料：

```text
handoff/2026-07-28/
├── ZEROARM_MCU_HANDOFF_V2.md
├── ZEROARM_MCU_AGENT_INSTRUCTIONS_V2.md
└── ZEROARM_MCU_HANDOFF_STATUS_V2.yaml
```

同时必须阅读：

```text
docx/Reference_plan/ZEROARM_MCU_FIRMWARE_DESIGN_V1.md
docx/Reference_plan/ZEROARM_MCU_FIRMWARE_CODE_REFERENCE_V1.md
docx/Reference_plan/ZEROARM_MCU_AUDIT_REMEDIATION_MASTER_PLAN_V2.md
docx/Reference_plan/ZEROARM_MCU_AUDIT_MATRIX_V2.yaml
```

一句话继续 MCU：

```text
读取 handoff/2026-07-28 和 docx/Reference_plan 中的 MCU 交接、Agent 规则、设计、Code Reference、审计总计划和 YAML 状态，以源码、git status 和现场测试为准，从 next_defect 开始逐缺陷修复；每个缺陷必须有真实回归、全量测试、严格 Sanitizer、STM32 Debug/Release、相关 diff 审查和独立 Git 提交，不夹带 dirty 文件，不执行机械动作。
```

## 4. 参考项目

上位机规划使用 `zero-robotic-arm-master` 中的 URDF、STL、D-H 和 MuJoCo 资产。
参考项目体积约 200 MiB，本交接资料默认不复制大型资产。

现场已发现的候选位置：

```text
C:\Users\Administrator\Downloads\zero-robotic-arm-master
C:\Users\Administrator\CLionProjects\zero_arm_mcu\docx\Reference_project\zero-robotic-arm-master
```

编码 Agent 应选择一个位置作为只读导入源，把经过校验的运行时资产复制到
`UpperComputer/resources/`，记录来源和哈希，禁止依赖桌面绝对路径运行产品。

## 5. 安全结论

- 机械臂尚未完成组装和标定。
- 默认只允许 Mock、编译、刷写 verify、HELLO 和 GET_STATE。
- 不允许真实发送目标、ENABLE、DISABLE、STOP、HOME 或 TEACH。
- ST-Link 当前存在 NRST/RDP 异常，详情见 MCU 交接文档。

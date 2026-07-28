# Agent 操作手册与提示词

## 1. 最短启动句

```text
严格按照 UpperComputer/AGENTS.md、PROJECT_SPEC.yaml 和 docs/16_AGENT_EXECUTION_MANIFEST.yaml，从 IMPLEMENTATION_STATUS.md 标记的首个未完成上位机单元开始，完成本单元全部实现、测试、集成、文档和独立 Git 提交后停止报告，禁止绕过协议审批与机械安全门。
```

## 2. 指定单元

```text
只实现 UpperComputer/docs/05_IMPLEMENTATION_PLAN.md 和 docs/16_AGENT_EXECUTION_MANIFEST.yaml 的审查单元 N，遵守 UpperComputer/AGENTS.md；完成实现、测试、构建、diff审查、状态文档和本单元独立Git提交后停止。
```

## 3. 只做审查

```text
审查当前上位机审查单元，不修改文件；检查需求覆盖、线程安全、协议、安全门、测试可信度和diff，按严重程度报告问题。
```

## 4. 协议审批包

```text
只准备上位机协议V2审批包，不修改MCU；逐字段列出新旧差异、端序、资源影响、兼容回退、黄金帧和测试计划，完成后停止等我确认。
```

## 5. 只读板测

```text
对刷写后的上位机/MCU执行只读板测，只允许HELLO、GET_STATE和已批准的只读能力命令，覆盖连接、CRC、噪声、DMA环绕和压力；列出命令审计，禁止任何运动、使能、失能、STOP、HOME或TEACH命令。
```

## 6. 打包

```text
只完成当前Windows打包审查单元，生成portable产物和哈希，在无系统Python等价环境做启动、Mock、资源和数据库烟雾测试；不要上传或发布。
```

## 7. 真机动作测试

真机动作不能只用一句笼统“测试一下”。应同时给出审批模板：

```text
在机械臂支撑、急停、地址、方向、零点、减速比和软限位均确认的前提下，只测试Jx，从A°到B°，最大V°/s；异常时软件STOP并可立即断电。不得扩展其他轴或命令。
```

Agent仍需复述边界后才能执行。

## 8. 上下文恢复

新Agent应：

1. 读取根和嵌套AGENTS。
2. 读取`IMPLEMENTATION_STATUS.md`（创建后）。
3. 现场运行git status。
4. 验证最近测试，不重复已完成且未变化的大规模工作。
5. 只进入状态文档指定的下一单元。

## 9. 为什么不是“一句话无监督跑到底”

最短启动句可以让Agent不再询问产品范围和技术栈，但不能取消：

- 每单元代码审查。
- MCU协议变更确认。
- 重力轴松脱确认。
- 真实运动范围确认。
- Git发布授权。

这些停点是项目正确性的一部分，不是提示词不够详细。

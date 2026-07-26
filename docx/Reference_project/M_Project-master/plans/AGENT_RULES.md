# Agent 工作硬性约束 (Agent Rules)

> 本文件为 Agent 的永久工作约束。每次恢复上下文或开始新任务前必须**完整阅读**本文件。
> 文件内容为已确认的事实与硬性规则,不是建议,不可绕过。

---

## 一、硬件连接状态（已确认,不要再问）

✅ 电机已上电
✅ CAN 收发器已焊接并正常供电
✅ 120Ω 终端电阻已装
✅ PA11/PA12 已接 CAN_RX/CAN_TX
✅ 串口 USART1（PA9/PA10）已接好
✅ 所有线缆连接**由用户亲自确认**

⚡ **绝对不要再问"请检查硬件连接"、"请确认电机已上电"之类的问题。**
⚡ **如果通信不通,99% 是软件问题,优先在代码层排查。**
⚡ **只有在大量软件诊断穷尽后,附完整日志,才可以提硬件嫌疑。**

---

## 二、烧录方式

### 工具
- **OpenOCD** + **CMSIS-DAP 探针**（不是 ST-Link,不是 pyocd,不是 CubeProgrammer）

### 烧录命令（直接使用,不要改）

⚠️ **Horco CMSIS-DAP v2 必须使用 HID 后端 (v1 模式)，不能用默认 bulk 后端！**

```bash
cd C:\Users\ASUS\Downloads\M_Project-master\M_Project-master
"C:\Tools\OpenOCD-20260121-0.12.0\bin\openocd.exe" -c "adapter driver cmsis-dap; cmsis-dap backend hid; transport select swd; adapter speed 100" -f "C:\Tools\OpenOCD-20260121-0.12.0\share\openocd\scripts\target\stm32g4x.cfg" -c "init; program build/Debug/M_Project.elf verify reset exit"
```

### 编译命令
```bash
cd build/Debug
cmake --build .
```

### 一键编译+烧录
```bash
cd C:\Users\ASUS\Downloads\M_Project-master\M_Project-master\build\Debug && cmake --build . && cd ..\.. && "C:\Tools\OpenOCD-20260121-0.12.0\bin\openocd.exe" -c "adapter driver cmsis-dap; cmsis-dap backend hid; transport select swd; adapter speed 100" -f "C:\Tools\OpenOCD-20260121-0.12.0\share\openocd\scripts\target\stm32g4x.cfg" -c "init; program build/Debug/M_Project.elf verify reset exit"
```

### 如果 OpenOCD 提示端口被占用 / 设备忙
这在反复烧录时很常见,自己解决:
```bash
# Windows: 杀掉已有的 openocd 进程
taskkill /F /IM openocd.exe

# Linux/Mac
pkill openocd
```

---

## 三、串口调试

### 工具
- **Python + pyserial** 已安装
- 串口通常是 `COM8`（CMSIS-DAP 探针的 VCP）或 `COM3/COM5` 等
  不确定就用 `python -m serial.tools.list_ports` 列出

### 常见问题自处理

#### 串口被占用 / 被其他程序打开
自己杀进程,不要问用户:
```bash
# Windows
tasklist | findstr python      # 看看有没有残留的 python 脚本占用
taskkill /F /PID &lt;PID&gt;         # 杀掉对应 PID

# 或者直接杀所有 python
taskkill /F /IM python.exe

# Linux/Mac
lsof /dev/ttyACM0              # 或 /dev/ttyUSB0
kill -9 &lt;PID&gt;
```

#### 串口读不到数据
按顺序自查:
1. 波特率是否是 115200
2. 有没有其他工具占着串口(串口助手、CLion 串口、之前的 python 脚本)
3. 主控是否在运行(是否刚烧录完没复位)
4. TX/RX 是否接反(可以改脚本里的端口试另一个)

#### 串口打印乱码
1. 检查波特率
2. 检查 printf 重定向是否正确配置
3. 检查 USART1 CR1 的 TE 位是否置位(之前的报告提到过)

---

## 四、Requirement.md 第四章的硬性地位

⚡ **Requirement.md 第四章用 ⚡ 标记了"必须实现、不可绕过、不可简化"的核心需求**
⚡ **任何任务的完成标志都是:第四章的三条链路端到端全部跑通**

三条链路简述:
- **链路 A**：上电自动回零 → 主控收到 02/9F → 上位机收到【启动】【完成】
- **链路 B**：上位机发命令 → 主控转 CAN → 收到 02 → 上位机收到【已接收】
- **链路 C**：电机执行 → 收到 9F → 上位机收到【完成】;错误时收到 E2 → 主控本地响应 + 上位机收到【失败】

未三条全部跑通的,**都视为未完成**。

---

## 五、电机侧必须配置的两件事（极易漏）

### 1. Response 模式必须改成 Both
电机出厂默认 Response = Receive,**只回 0x02 不回 0x9F**。
不改这一项,**链路 C 永远不可能跑通**。

改法：发命令 `01 48 D1 ... [Response=03Both] ... 6B`
参考手册 5.8.4 节完整字段。

### 2. 定时返回必须启用
Requirement.md 第四章要求主控转发电机实时数据给上位机。
不启用定时返回,主控没数据可转发。

参考手册 5.5.1 节配置周期性上报。

**这两件事必须在主控初始化阶段完成,不要依赖电机出厂设置。**

---

## 六、工作方式硬性约束

### 必做
- ✅ 以实测串口日志为准,不是"看起来应该对"
- ✅ 每次代码改动都要编译 + 烧录 + 实测
- ✅ 小步快跑,一次改一处
- ✅ 遇到串口/烧录工具问题自己用命令行解决
- ✅ 任务推进过程中把进展追加写入 `plans/can_fix_progress.md`

### 禁止
- ❌ 不问"请检查硬件"、"请确认已上电"等问题
- ❌ 不说"应该可以工作"之类没有日志的推断
- ❌ 不一次改一堆代码然后整体测
- ❌ 不把"本地状态机转通了"当成"链路跑通了"
- ❌ 不跳过实测验证
- ❌ 不在三条链路全部跑通前宣称完成

### 卡住时
尝试 8~10 次仍无进展:
1. 重读手册对应章节
2. 尝试完全不同思路（Loopback 自检 / 完全重写初始化 / 最简命令验证）
3. 对照 STM32G4 官方 FDCAN 例程
4. 实在不行,汇报时必须附完整诊断日志和推理

---

## 七、断点续传

每完成一次关键动作,追加写入 `plans/can_fix_progress.md`:
- 时间
- 做了什么
- 串口日志关键片段
- 结论 / 下一步

这样上下文丢失后,下一次 Agent 读这个文件就能接续。

---

## 八、最终交付格式

任务达成后输出 `plans/can_final_verification.md`:
1. 项目当前状态概述
2. 所有诊断日志
3. 真正的根因分析
4. 修改清单（文件 + 改动 + 原因）
5. 三条链路每条至少 30 行端到端串口日志
6. 对 Requirement.md 第四章三条链路的逐条验证结论
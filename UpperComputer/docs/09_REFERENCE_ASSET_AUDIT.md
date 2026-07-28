# 参考项目资产审计

## 1. 唯一参考路径

当前仓库已经包含参考项目：

```text
docx/Reference_project/zero-robotic-arm-master/
```

桌面路径还存在外层工具、`node_modules` 和嵌套副本，不应整体复制进仓库。
后续以仓库内版本为唯一输入，避免重复文件和缓存污染。

## 2. 可复用资产

| 资产 | 路径 | 用途 |
|---|---|---|
| URDF | `3. Simulink/URDF_XG_Robot_Arm_Urdf_V1_1/urdf/*.urdf` | 关节树、坐标变换、质量和网格 |
| STL | URDF 同级 `meshes/` | GUI 3D 显示 |
| URDF CSV | URDF 同级 `*.csv` | 质量、惯量、关节导出数据复核 |
| D-H 表 | `3. Simulink/D-H.xlsx` | 运动学对照 |
| MATLAB | `3. Simulink/robot_kinematics*.m` | 解析逆解和坐标偏置参考 |
| MuJoCo | `5. Deep_LR/robot_arm_mujoco.xml` | 离线仿真和强化学习接口参考 |
| MuJoCo STL | `5. Deep_LR/meshes/` | 仿真模型 |
| 旧串口脚本 | `2. Software/robot/robot_joystick.py` | 手柄交互参考，不复用协议 |
| CAD/STEP/3MF | `1. Model/` | 机械结构和后续碰撞模型来源 |

## 3. 已发现的坐标差异

参考 URDF 与当前 MCU 初始配置不能直接按数值一一对应：

| 关节 | URDF | 当前 MCU 初值 | 处理 |
|---|---|---|---|
| J1 | continuous | 0～360° | 需要角度归一化策略 |
| J2 | -90～90° | 90～180° | 很可能存在 +180°或其他零点偏置，必须验证 |
| J3 | 0～180° | -90～90° | 很可能存在 -90°偏置，必须验证 |
| J4 | continuous | -90～90° | GUI 使用 MCU 保守限位 |
| J5 | -90～0° | 0～90° | 很可能符号或 +90°偏置不同 |
| J6 | continuous | 0～360° | 需要角度绕回策略 |

这不一定是错误：URDF 关节坐标、机械展示角和 MCU 公共关节角可以具有固定
偏置和符号。上位机必须显式定义：

```text
MCU joint urad
 -> sign + zero offset + wrap policy
 -> model joint rad
```

不得通过在 GUI 中散落 `+90`、`-90` 等常量解决。

## 4. 模型导入流程

1. 解析 URDF 关节树、origin、axis、limit、mass、inertia。
2. 将 `package://` URI 映射到受控资源目录。
3. 校验所有 STL 存在、可解析、三角面非空。
4. 计算网格包围盒，检测毫米/米缩放异常。
5. 生成规范化的运行时模型清单和 SHA-256。
6. 建立 `JointModelMapping`，集中保存符号、零偏、wrap 和限位来源。
7. 用 URDF 零位、参考截图和 MATLAB 已知姿态做黄金图验证。
8. 目标姿态使用半透明 ghost model；实际姿态使用实体模型。

参考文件只读；需要打包的模型复制到 `UpperComputer/resources/robot_model/`，
并保存来源和哈希。

## 5. 不能直接复用的内容

- 旧 `robot_joystick.py` 发送文本命令，与当前二进制协议不兼容。
- 参考固件是 STM32F407 + EMM，当前项目是 STM32G474 + X_V2。
- MuJoCo 控制量和真实电机命令没有现成 observation/action 桥接。
- MuJoCo XML 引用了不存在的 `asset/desert.png`，导入时需替换或移除。
- URDF 中部分 velocity/effort 为 0 或缺失，不能作为真实安全限值。
- J1/J4/J6 的 continuous 定义不能覆盖 MCU 的装机软限位。

## 6. 原项目是否有上位机

审计结论：没有完整上位机源码。

现场搜索仓库参考副本及桌面内层原项目：

```text
.html/.css/.js/.jsx/.ts/.tsx/.vue/.qml/.ui = 0
Python文件 = 3
```

三个Python文件分别是：

| 文件 | 实际性质 | 可借鉴 |
|---|---|---|
| `2. Software/robot/robot_joystick.py` | Pygame 600×200手柄窗口 + pyserial文本命令 | 手柄枚举、轴读取、dead loop基本结构 |
| `5. Deep_LR/robot_arm_env.py` | MuJoCo Gymnasium强化学习环境 | 模型加载、末端位置、viewer和目标标记 |
| `5. Deep_LR/train_robot_arm.py` | TD3训练/测试入口 | 离线策略评估流程 |

README把“MQTT远程控制”和“WEB可视化平台”列为计划功能，没有对应前端实现。
Simulink模型和MuJoCo viewer属于仿真工具，也不是连接当前G474固件的操作上位机。

旧手柄脚本不能作为新GUI基线，因为它：

- 使用旧固件的换行文本命令 `remote_enable/remote_event/remote_disable`。
- 直接在UI循环中读写串口和`sleep(0.1)`。
- 没有CRC、状态机、限位、故障、命令确认或安全门。
- 没有轨迹、3D、日志、标定和打包结构。

仍可将手柄轴枚举和MuJoCo目标可视化作为行为参考，但新上位机应按本规划独立
实现。README提及的第二代ZYArm-X1有独立仓库和更完整的软件生态，它不是本代
ZERO参考项目的本地上位机源码；若未来要借鉴，需另行做协议、许可证和架构审计。

## 7. 资产验收

```text
[ ] URDF 可解析且关节树为单根无环
[ ] 7 个 link 和 6 个 joint 完整
[ ] 7 个 STL 均可加载
[ ] 网格缩放和朝向通过黄金图
[ ] MCU→模型六轴映射有集中配置和单元测试
[ ] FK 与 MATLAB/MuJoCo 至少 10 个已知姿态一致
[ ] 打包后不依赖桌面绝对路径
[ ] 参考资产许可证随发行包保留
```

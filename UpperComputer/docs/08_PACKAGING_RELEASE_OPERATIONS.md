# 打包、发布与运行维护

## 1. 发布产物

必须：

```text
ZeroArmDesktop-<version>-windows-x64-portable.zip
SHA256SUMS.txt
THIRD_PARTY_LICENSES/
USER_GUIDE.md
KNOWN_LIMITATIONS.md
```

建议：

```text
ZeroArmDesktop-<version>-windows-x64-setup.exe
```

## 2. PyInstaller

- 使用固定 `.spec`。
- `onedir`作为主产物，便于Qt/OpenGL/模型资源诊断。
- 是否附带`onefile`作为额外便利版本，需在性能和杀毒误报测试后决定。
- 显式收集Qt平台插件、OpenGL插件、pyqtgraph、STL、图标和schema。
- 资源通过统一locator访问，不依赖CWD。
- 应用版本从单一源生成。

## 3. Portable布局

```text
ZeroArmDesktop/
├── ZeroArmDesktop.exe
├── runtime/
├── resources/robot_model/
├── licenses/
├── docs/
└── tools/README.md
```

STM32CubeProgrammer默认不随应用重新分发；设置页允许选择其安装路径。若未来
需要捆绑，必须核对许可证。

## 4. 用户数据

默认放在 `%LOCALAPPDATA%/ZeroArmDesktop/`：

```text
config/
data/sessions.db
logs/
exports/
diagnostics/
```

升级保留用户数据；卸载默认保留，提供明确勾选项删除。应用不能把数据库写到
Program Files。

## 5. 安装包

Inno Setup建议：

- per-user安装，不强制管理员权限。
- 创建开始菜单和可选桌面快捷方式。
- 注册可选轨迹文件扩展名。
- 检测正在运行的旧版本。
- 升级前备份配置schema。
- 不静默安装ST-Link/VCP驱动。

## 6. 版本

建议SemVer：

- app version。
- protocol compatibility范围。
- database schema。
- trajectory schema。
- model asset revision。

关于页必须展示这些版本和构建Git SHA。

## 7. 发布流水线

```text
锁定依赖
 -> lint/type/test
 -> Mock E2E
 -> 性能基线
 -> build portable
 -> clean-machine smoke
 -> build installer
 -> install/upgrade/uninstall test
 -> generate licenses/hash/evidence
 -> 用户授权后发布
```

不得自动提交、推送、创建GitHub Release或上传产物，除非用户明确要求。

## 8. 故障诊断

应用启动失败时支持命令行：

```text
--safe-mode
--mock
--reset-layout
--log-level debug
--diagnostics
```

safe-mode禁用3D、手柄、自动连接和第三方Transport，仍能查看日志和设置。

## 9. 干净环境矩阵

- Windows 11 x64，无Python。
- 中文用户名和含空格路径。
- 无STM32CubeProgrammer。
- 有/无ST-Link驱动。
- 集显/独显。
- 100%、150%、200% DPI。
- portable只读来源解压到可写目录。

最终记录安装包和portable哈希及测试机器信息。

# Git 初始化与推送计划

## 项目信息摘要

| 项目 | 值 |
|------|-----|
| 项目类型 | STM32G474 CubeMX + FreeRTOS |
| 远程仓库 | `https://github.com/CDeZT/zero_arm_mcu.git` |
| CLion 分支 | `main` |
| 历史提交 | 3 次（init x2, config） |

## 当前状态

- CLion 的 `.idea/vcs.xml` 已配置 Git VCS
- CLion 的 `.idea/workspace.xml` 记录了远程仓库 URL 和历史提交记录
- **`.git` 目录可能缺失**（需确认）
- 项目根目录 **没有 `.gitignore`** 文件

## 执行步骤

### 步骤 1: 确认 Git 仓库状态

```bash
# 检查 .git 目录是否存在
ls -la | grep .git

# 如果存在，检查当前 Git 状态
git status
git remote -v
git log --oneline
```

### 步骤 2: 创建 `.gitignore` 文件

针对 STM32/CubeMX/CLion 项目，建议忽略以下内容：

```gitignore
# ===== IDE =====
.idea/workspace.xml
.idea/shelf/
.idea/httpRequests/
.idea/queries/
.idea/dataSources/
.idea/dataSources.local.xml

# ===== Build output =====
build/
cmake-build-*/
cmake-build-debug/
cmake-build-release/

# ===== STM32CubeMX generated =====
# (keep .mxproject as it's useful for tracking CubeMX config)

# ===== Compiled output =====
*.o
*.d
*.elf
*.hex
*.bin
*.map
*.out
*.exe

# ===== OS files =====
.DS_Store
Thumbs.db
Desktop.ini

# ===== Simulink / MATLAB junk =====
*.mat
*.mldatx
slprj/
export.log
```

### 步骤 3: 初始化 Git（如需要）

```bash
git init
git checkout -b main
```

### 步骤 4: 配置远程仓库

```bash
git remote add origin https://github.com/CDeZT/zero_arm_mcu.git
```

如果远程已存在，先检查：
```bash
git remote -v
```

### 步骤 5: 添加所有文件

```bash
# 查看哪些文件会被添加
git status

# 添加所有文件
git add -A
```

### 步骤 6: 提交

```bash
git commit -m "initial commit: STM32G474 zero arm mcu project with FreeRTOS"
```

### 步骤 7: 推送到远程

```bash
git push -u origin main
```

> **注意**: 如果远程已有代码且历史不一致，可能需要 `git pull --rebase origin main` 先合并。

## 注意事项

1. **`docx/` 目录** - 包含参考项目文件（Simulink .mat 文件、STL 文件等），体积可能较大。如果遇到 push 失败，可能需要检查文件大小限制或使用 Git LFS。
2. **`.mxproject`** - 这是 CubeMX 的配置文件，建议保留在版本控制中。
3. **关联的 GitHub 账号**: `CDeZT` - push 时可能需要对应 GitHub 账号的认证（Token 或 SSH key）。

# Git Flow 开发规范（中文）

本文档用于约束本仓库的日常研发流程。自本规范生效起，`Issue / Commit / PR` 必须使用中文并写清楚背景、变更和验证结果。

## 分支模型

- `main`：
  - 仅用于生产可发布代码。
  - 只能通过 `release/*` 或 `hotfix/*` 合并进入。
- `develop`：
  - 日常集成分支。
  - `feature/*` 完成后合并到 `develop`。
- `feature/*`：
  - 功能研发分支，从 `develop` 拉出。
  - 命名示例：`feature/fire-overlay-layer`
- `release/*`：
  - 发布准备分支，从 `develop` 拉出。
  - 命名示例：`release/v0.4.0`
- `hotfix/*`：
  - 线上紧急修复分支，从 `main` 拉出。
  - 命名示例：`hotfix/port-cleanup-fix`

## 标准流程

1. 创建 Issue（中文）
- 明确：背景、目标、范围、验收标准、风险、回滚方案（可选）。

2. 从 `develop` 拉取并创建功能分支
- `git checkout develop`
- `git pull origin develop`
- `git checkout -b feature/<中文语义英文名>`

3. 开发与提交（Commit 中文详细）
- 每次 commit 必须说明：
  - 做了什么
  - 为什么做
  - 如何验证

4. 提交 PR 到 `develop`（PR 中文详细）
- 必填：背景、改动点、影响范围、验证步骤、风险与回滚。

5. 审查通过后合并到 `develop`

6. 发布时从 `develop` 拉 `release/*`
- 完成版本校验后合并到 `main`，并回合并到 `develop`。

7. 线上问题走 `hotfix/*`
- 从 `main` 拉分支，修复后同时合并回 `main` 与 `develop`。

## 提交信息建议格式

推荐前缀：

- `feat:` 新功能
- `fix:` 缺陷修复
- `docs:` 文档更新
- `refactor:` 重构
- `test:` 测试相关
- `chore:` 构建/脚本/流程

示例：

- `feat: 增加火情热点叠加图层`
- `fix: 修复 fire demo 停止后端口未释放问题`

正文建议固定包含三段：

- 做了什么
- 为什么做
- 如何验证

## 保护策略（建议）

- 禁止直接向 `main` push。
- `main` 与 `develop` 合并必须走 PR。
- PR 至少 1 人审查通过。


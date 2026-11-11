# 来源清单（MANIFEST）

本仓是**外网开发复刻仓**，源出公司内网 GitLab 的 `simulator`。只保留源码；预编译依赖不进 git。

## 内网版本

| 项 | 值 |
|---|---|
| 内网仓库 | `git@172.16.10.36:bonder_series/icb3000/simulator.git` |
| 分支 | `HEAD` |
| 导出时 commit | `70f42f1ba9ca727e0b500a7deaecf067aaa072be` |
| 导出时间 | 2026-11-12 |

## 被排除的预编译依赖

- `28module\env/`
- `28module\backend\runtime/`
- `simbridge\deploy/`

这些目录原本是 git 子模块或打包好的运行时，含大体积二进制（单个文件可超过 GitHub 100 MB 上限），
因此被 `.gitignore` 排除。把对应的 deps 包解压到仓库根目录即可还原（见 Release 页）。

## 备注

`28module/env` 是打包的 Python 运行时（numpy/opencv/PyQt5），`simbridge/deploy` 是构建产物，`28module/backend/runtime` 是打包后的后端运行时——三者均为二进制，走 deps 包。

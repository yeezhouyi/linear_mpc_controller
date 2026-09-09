# circle 闭环冒烟证据 — REPRODUCE / 校验说明（2026-09-09）

## 证据是什么

`results/mpc_smoke_20260909/circle/` 保存 2026-09-09 **15:13:52**（WSL 本地时间）那次
Gazebo TurtleBot3 **circle** 闭环冒烟（P0）的归档产物：

- `tb3_smoke.json` — 机器可读 PASS 记录（driven 20.7 m、median 8.6 mm、p95 34.2 mm、in-band 1.0）
- `launch.log` — 该次启动（`track:=circle`，9 个进程）的真实 launch 控制台输出
- `SHA256SUMS` — 本目录文件的 SHA256 校验清单（校验命令见下）

## 生成命令

```bash
bash scripts/tb3_smoke.sh
```

- 脚本会 source `$HOME/ros2_ws/install/setup.bash`（本地 colcon install 工作区）。
  该工作区**不属于本仓库**：仓库 tag 只包含源码 / launch / config / 文档，
  不包含 Gazebo world / 模型 / 构建产物；换机器需先自行 `colcon build`。
- **当前归档产物 = 2026-09-09 15:13:52 那次运行**，源自当时的本地工作区；
  **尚未从干净 tag 检出 + 全新 build 重跑验证**（重跑前请先满足工作区依赖）。

## 证据范围说明

- circle 冒烟走 `scripts/tb3_smoke.sh` 纯 P0 流程，**只生成 `tb3_smoke.json` 与
  launch 控制台日志，不产生 `motion.txt` / `diagnostics.txt`**
  （`results/mpc_smoke_20260909/straight/` 才有那两个补充文件）。
- 因此本目录刻意不含 motion / diagnostics，避免占位 / 伪造文件。

## 校验

```bash
cd results/mpc_smoke_20260909/circle
sha256sum -c SHA256SUMS
```

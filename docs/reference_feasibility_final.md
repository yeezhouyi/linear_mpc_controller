# Reference-Feasibility Final Acceptance (2026-09-09)

> 本条 = 必做① 的**最终验收与复现固定件**。基线：代码 `b0083ca`（=`main`，含 QP build-option
> 归因修复），对外引用 `v0.3.0-engineered`。P0 收口/冒烟/边界细节见
> `docs/final_closeout_20260909.md`（同提交集），本文件只保留必做① 的公平对比与复现链。

---

## 1. 最终测试（main 上 2026-09-09 复跑）

```
python3 -m pytest trajectory_tools mpc_core -o addopts= -q   # 114 passed (exit 0)
```

子集：`test_reference_guard.py` 11/11、`test_speed_profile.py` 8/8（含在 114 内）。
C++ 核心/ROS 包构建与 gtest 见 `final_closeout_20260909.md` §2 / CI（双 job GREEN）。

## 2. 参考可行性处理 前后对比（固定口径；同控制器/同参考归因纪律）

| 轴 | 对比 | 结论 | 证据 |
|---|---|---|---|
| A 同控制器·不同参考 | MPC(raw) vs MPC(guarded) | 守护对本来可行的 raw 惰性（0 约束行）；**无退化** max\|Δprogress\|=0.0011、max\|Δe_y_rms\|=0.0096 | `feasibility_matrix.md` leg(a) |
| B 同参考·不同控制器 | MPC(guarded) vs PP(guarded) | 控制器轴独立：progress 0.9958 vs 0.9954 | `feasibility_matrix.md` leg(b) |
| C 生产入口·同控制器 | MPC(raw) vs MPC(entry) | 全部 COMPLETED+certified+离散检查过；v_end=0.0000 全部；max\|Δprogress\|=0.0009、max\|Δe_y\|=0.0096（向改善）、qp Δ=0 | `REGRESS.md` |
| D 不可行参考显式拒绝 | 4 类构造（倒车/ω 界/原地转向/非有限） | **控制器被调用前**拒绝，稳定码：`sustained_reverse / omega_bound / inplace_rotation / nonfinite`（+`zero_length_segment`） | `feasibility_matrix.md` leg(c) |

归因纪律：A/C 两侧同一控制器 → 任何差是**参考层效应**，不记为算法提升；B 两侧同一参考 →
控制器差异单独成轴。末端减速的代价如实报告：foldback 因缓行至 accepted-arc watermark 多
**791 步**（1440→2231），代价可见；e_y_rms 反而改善（entry 0.0103 vs raw 0.0199，因参考不再
以 0.3 m/s 冲入折返）。四个开放几何 +≤1 步。

## 3. 一条完整复现命令（一条链复现第 1/2 节全部分数）

```bash
cd ~/ros2_ws/src/linear_mpc_controller && \
python3 -m pytest trajectory_tools mpc_core -o addopts= -q && \
python3 benchmark_tools/scripts/run_feasibility_matrix.py && \
python3 benchmark_tools/scripts/run_entry_regress.py && \
echo "EVIDENCE:" && ls results/feasibility_matrix/feasibility_matrix.md results/feasibility_regress/REGRESS.md
```

（2026-09-09 实测通过；matrix ≈1 min、regress ≈1–2 min，纯 Python 无 ROS 依赖。）

## 4. Gazebo TurtleBot3 闭环冒烟 — 结论与证据位置

冒烟驱动的是**运行时链**（本轮未改 ROS 节点代码）；详细故障/归因叙述见
`final_closeout_20260909.md` §6（closed-loop 开口修复 + QP build-option 归因）。本文档只钉证据：

- 今日冒烟（`main` 代码、install_lmpc 运行链）：`track:=straight` 单 cell，
  **PASS displacement = 5.396 m**，`/linear_mpc_node/diagnostics` 有持续输出；
  证据 `results/mpc_smoke_20260909/straight/{launch.log,motion.txt,diagnostics.txt}`。
- 历史 20-cell 基线（同链）：**20/20 motion PASS**，位移 1.8–5.4 m（4 轨迹×5 seed），
  `results/mpc_baseline_ros/aggregate.json`（`e11f889` 已入库）。

## 5. 集成边界（与 final_closeout §5 一致，这里只列引用）

参考可行性闸门与沿路径剖面为参考准备层（`reference_guard` / `speed_profile` /
`adapter_pipeline.prepare_tracker_reference` 单一入口），由 benchmark/任务层调用；
ROS `trajectory_server` 的分发路径**尚未接线**该入口——不隐藏，列为后续集成项。
无硬实时 / 无原始路径单发成功等对外边界见 README 顶部清单与 `final_closeout_20260909.md` §5。

## 6. 证据清单

| 件 | 位置 |
|---|---|
| 可行性矩阵 | `results/feasibility_matrix/feasibility_matrix.md`（+JSON）@ `c21d9ff` |
| 生产入口回归 | `results/feasibility_regress/REGRESS.md`（+JSON）@ `309ec22` |
| Gazebo 冒烟（今日） | `results/mpc_smoke_20260909/`（本提交） |
| Gazebo 基线（历史） | `results/mpc_baseline_ros/aggregate.json` @ `e11f889` |
| P0 收口/冒烟详情 | `docs/final_closeout_20260909.md`（`b0083ca`） |

# A8 离线矩阵尝试 —— 本机实测记录 (2026-09-08)

## 做了什么
1. replay_path_mpc.py 增加 A8 轴参数：`--controller-projection-mode`
   (windowed|global) 与 `--v-min`，接线进 MpcParams（审计侧 accepted-arc
   不变，符合 A8.3"唯一变量"定案）。
2. 用 u9 planner (EXPLORER_REPO=robot_advance_4060 u9 工作树) +
   plan_from_map.py 对 cleaning_room_rect 地图生成真实 B6 清洁路径
   (59.6 m, 27 waypoint, planned_coverage 1.0)。
3. 双模式离线 replay 该路径：**两模式在 arc 4.87 m（首行端）完全一致地停滞**
   (progress_ratio 0.0817, 900 步, e_y_rms 0.104)。
4. 阳性对照：稠密实测 bridge 路径 (6.1 m, 150 poses) 双模式 replay →
   **完全一致** (progress_ratio 0.9838, e_y_rms 4.0 mm, 0 jumps)。

## 结论（诚实口径）
- 停滞与投影模式无关：27 个稀疏 waypoint 弦连使行端折返成为瞬时 180°
  角；折叠路径上最近点锚点无法单调推进 —— 与 results/b6_replay/
  FINDING.md (2026-09-07) 的结构性边界一致，也即 4060 文档 A8 需在
  实车/live 链路上跑的原因。
- 投影模式轴（A8 唯一变量）的差异只在"投影发生跳跃"时显现，且已在
  控制器级被 test_projection_mode.py 证明（global 记账 3 m 瞬移，
  windowed 拒绝）；干净稠密路径上两模式相等（本文件阳性对照）。
- A8 12 格矩阵本体（3 几何 × 2 v_min × 2 mode）需 u9 稠密/live 路径 +
  WSL2 时钟修复后的 Gazebo 链；离线侧结论到此为止。

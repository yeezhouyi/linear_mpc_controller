# U1/C0 基线审计：现有 `mpc_controller`（v0.2.1）

> 依据《线性 MPC 与残差强化学习轨迹跟踪控制器进阶》计划 U1/C0：冻结已有 MPC 的事实基线，
> 消除状态定义 / 误差坐标 / 线性化 / 离散化 / QP / 接口歧义，给出“保留 / 重构”结论。
> 审计对象：GitHub `yeezhouyi/mpc_controller`，基线 commit `25b0929`（v0.2.1 stable + v0.2.2 工具）。

## 1. 基线事实（冻结）

| 项 | 事实 | 来源 |
|---|---|---|
| 形式 | `ros2_control` 控制器插件（`controller_interface::ControllerInterface`，pluginlib 导出） | `src/mpc_controller.cpp` 末尾 `PLUGINLIB_EXPORT_CLASS` |
| 控制对象 | **RRBot 2-DOF 平面机械臂**（不是差速底盘）；`ros2_control` 的 `controller_manager update_rate=100 Hz` | `config/rrbot_mpc.yaml`、`launch/rrbot_mpc.launch.py` |
| 模型 | LTI `x[k+1] = A x[k] + B u[k]`，A/B 由 YAML 直接给出（双积分器：`q,q̇ → τ`） | `LinearModel`，`config/rrbot_mpc.yaml` |
| 状态/输入 | 机械臂：`x=[q1,q̇1,q2,q̇2]`，`u=[τ1,τ2]`；占位 diff-drive 配置（v0.2.0 遗留）：`x=[v,ω]` 一阶滞后 `A=diag(0.9)`，`B=diag(0.1)`，**无 launch，从未加载** | `config/diff_drive_mpc.yaml`（仅存在于 v0.2.0 归档） |
| 误差坐标 | **原生（关节/世界）坐标直接相减**：`e = x_ref − x`；无 Frenet、无车体系、无曲率项 | `update()` 中 `x_ref_ − x0` |
| 参考输入 | topic `~/reference`（`Float64MultiArray`），整段状态目标，无时间戳/速度/曲率概念 | `on_configure()` |
| 预测时域/周期 | `prediction_horizon=20`，`dt=0.01 s`（模型步长 = 100 Hz 控制周期） | YAML |
| 离散化 | 无显式离散化流程——A/B 由用户手填（对 RRBot 双积分器即精确离散；对 diff-drive 一阶滞后为近似） | `LinearModel::initialize` |
| QP | condensed（消去状态，决策 `z=[U, ε]`），OSQP；软速度约束 slack（L1+L2）；硬位置/输入/输入速率约束；warm start（分段平移）；Q/R/S 运行时热更新（P 先于梯度重建） | `mpc_controller.cpp::buildQPStructure/rebuildCachedMatrices/buildAndSolveQP` |
| 回退 | 求解失败 → `hold` 上一帧 `u`（`u0 = prev_u_`）；`solved_approximate` 时采用近似解 | `update()` |
| 诊断 | `~/diagnostics`（`Float64MultiArray`，自描述：nx/nu 头 + 状态/参考/误差/u/耗时/残差/slack） | `update()` |
| 基准 | WSL2 5×65 s：全部 run 最优解率 89.9%、均值求解 5.81 ms；clean-run(01-02) 98.4%、3.88 ms；paired A/B 证明 v0.2.1 循环耗时 −12%（3.06→2.69 ms） | `README.md` |
| 已知问题 | WSL2 调度导致 run-to-run 差异；硬约束下 59.5% 最优率 → 软速度约束修复（97.5%+）；warm-start 分段 bug 已修（0 NaN/22k 周期） | `README.md` Known Issues |

## 2. 与计划的差距（U1 结论）

计划（R1–R13）要求差分底盘轨迹跟踪，现有代码与之的差距如下：

| # | 差距 | 证据 | 处置 |
|---|---|---|---|
| G1 | 控制对象是机械臂插件，不是差速底盘 `/odom → /cmd_vel` 轨迹跟踪 | 界面为 `position/velocity/effort` | 新包 `linear_mpc_controller` 以差速底盘为主（KTD2），不改造旧插件 |
| G2 | 误差坐标未冻结：旧代码在原生坐标直接相减，计划禁止世界/车体/Frenet 隐式切换（R1） | `x_ref_ − x0` | 新核心显式冻结 Frenet 误差坐标（见 `mpc_model_derivation.md`），符号有测试 |
| G3 | 模型是 YAML 手填 LTI；无曲率/参考速度参数化、无 LTV、无“线性化点记录” | `A_data/B_data` | 新核心从参考（κ,v_r）逐段解析生成 A/B（R2），窗口锚点显式记录 |
| G4 | 参考只有全状态点，无时间戳/速度/曲率/过期语义（R6/R9） | `Float64MultiArray` | 新包定义 `TrajectoryPoint(s,x,y,yaw,κ,v,t)` + nav_msgs/Path 补全规则 |
| G5 | 回退 = hold 上一帧非零控制；计划禁止持续沿用上一帧（R4/R13） | `u0 = prev_u_` | 新核心健康状态机：关键状态**立即归零**，QP 失败走确定性子降速→安全停车，带时间上限 |
| G6 | 无状态新鲜度/TF/时钟纪律（R9） | 硬界面直读 | ROS2 层（节点/适配器/仲裁器）负责 |
| G7 | QP 状态与耗时可观测（有），但无“p95 求解预算”门槛（有 deadline 统计） | diagnostics | 沿用并扩展字段；健康状态含 QP_TIMEOUT |
| G8 | diff-drive 占位配置的 100 Hz/dt=0.01 与计划默认 Ts=0.05 s（20 Hz）不一致 | 占位 YAML | 新包冻结 Ts=0.05 s（U1 决定：若 WSL2 实测 Gazebo 周期要求修改，模型/约束/报告同步更新） |
| G9 | 软约束只作用于“速度类”状态索引；计划要求 v/ω/加速度约束与不可行分级降级 | `velocity_indices` | 新核心 MVP 用硬约束 + 降级梯子；软约束放宽列 C2 后续 |

## 3. 保留与复用

1. **QP 工程经验**（保留到 C++ 核心）：condensed 构造、`A_power` 幂缓存、warm-start 分段平移（U 块/ε 块分离）、`allFinite()` NaN 防线、Q/R/S 热更新时“先重建 P 再算梯度”、求解状态/残差/耗时诊断。这些 v0.2.0/0.2.1 踩坑结论直接迁移。
2. **OSQP++ 封装**（`include/3rdparty/osqp++.h`）接口设计（setup/updateGradient/updateBounds/updateCostMatrix/warm start）→ 新 C++ `qp_problem` 与 Python `AdmmQp` 对齐同一接口契约。
3. **基准/可视化流程**：`benchmark_plot.py`、run manifest 思想 → `benchmark_tools`。
4. **参数校验纪律**：尺寸一致性检查、diag 权重推断维度 → 新配置加载照搬。

## 4. 不保留 / 重构

- `MPCController::update()` 的 “hold prev_u_” 回退 → 健康状态机（G5）。
- “参考=整段目标点、误差=直接相减” → 轨迹点语义 + Frenet 误差（G2/G4）。
- 手填 A/B 的 `LinearModel` → 差速模型 + 自动离散（G3）。

## 5. 冻结决定（U1 输出）

| 项 | 冻结值 | 备注 |
|---|---|---|
| 控制对象 | 二维差速底盘（unicycle 运动学），输入 v/ω | TurtleBot3 参数为标称（C5 做系统辨识） |
| 控制周期 | `Ts = 0.05 s`（20 Hz） | WSL2/Gazebo 实测后再确认（U1 待办） |
| MPC 状态 | `x = [e_y, e_psi, v, ω]` | 误差坐标见推导文档 |
| 决策量 | `Δu = [a, α]`（m/s², rad/s²，ZOH 每步积分） | 首步预测速度 = `cmd_vel` 候选 |
| QP 后端 | C++: OSQP/OsqpEigen（默认，版本于 C0 `deps.repos` 固定）；Python 参考核心: 自带稠密 ADMM（同一契约） | 若现有项目已满足接口的求解器，U1 以 ADR 保留 |
| 参考输入 | `nav_msgs/Path`（位姿）→ Adapter 补全 v/κ/时间戳；内部 `Trajectory` 为准 | 补全规则确定性（见接口契约文档） |
| 轨迹误差指标 | `e_y` 有符号 Frenet 横向距离、`e_psi` 归一化角度、`e_v = v − v_ref` | RMSE/p95/max |

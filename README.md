# linear_mpc_controller

差分底盘**线性时变 MPC 轨迹跟踪控制器**——`mpc_controller`（v0.2.1，ros2_control 线性 MPC 插件）的进阶项目，
> **仓库导航**：默认展示 = `main` @ `v0.3.0-engineered`（2026-09-09 参考可行性增强封板）；
> 历史研发线 `postseal2-planner-feasibility-20260909`、`cloud-*`、`main-seal-doc-*` 保留为工程证据。
> 封板口径：对外数字唯一来源 = `ros2_tunnel_explorer` 的
> `docs/seal_results.json` @ `v1.0.0-sealed`（README 与简历同源渲染）；
> 本仓封板主张见文末「Day 10 封板主张」表。**残差 RL 支线已冻结，不作为投递主张**
> （依据与复活条件：`docs/residual_rl_postmortem.md`）。

依据《线性 MPC 与残差强化学习轨迹跟踪控制器进阶》计划（U1–U12、R1–R25）推进（RL 部分已冻结）。
目标链：`上层路径 → Trajectory Adapter → Linear MPC（+ 安全投影 / 位移预算接受门）→ Velocity Arbiter → /cmd_vel → Gazebo`（自建 MPC 接管 /cmd_vel；残差 RL 支线已冻结，不接入该链）。

## 状态（本分支/本次成果）

| 层 | 内容 | 状态 |
|---|---|---|
| **审计** | `docs/baseline_audit.md`：冻结 v0.2.1 基线，逐条给出与计划的差距（G1–G9）与保留/重构结论 | ✅ |
| **数学** | `docs/mpc_model_derivation.md`：Frenet 误差、解析线性化、ZOH 离散化、condensed QP、回退梯子、指标定义 | ✅ |
| **接口契约** | `docs/ros2_interface_contract.md`：topics/frames/QoS/补全规则/健康状态/单写者 | ✅ |
| **Python 参考核心** | `mpc_core/`：frenet / model / qp(自研稠密 ADMM) / mpc / fallback / episode | ✅ 本机 45+ 测试全绿 |
| **轨迹工具** | `trajectory_tools/`：直线/圆/S/U-turn 生成器 + 位姿补全曲率/速度 | ✅ |
| **基准工具** | `benchmark_tools/`：RMSE/p95/max + QP 统计 + run manifest | ✅ 基线见下 |
| **C++/Eigen 核心** | `include/ src/`（model/mpc/safety，U2–U4 结构） | ✅ **WSL2 门槛通过**：`cmake -DBUILD_TESTING=ON` 构建 + ctest 全绿 |
| **ROS2 层** | `ros2/`（linear_mpc_node / trajectory_adapter / velocity_arbiter）+ launch/config/worlds + `test/test_ros_contract.py` | ✅ WSL2 `colcon build` 绿；契约测试 5/5 绿；launch 已修复为按安装 share 目录解析 world/config（OpaqueFunction）；**Gazebo TurtleBot3 闭环冒烟待跑（U5/C3 验收项）** |
| **RL 环境** | `mpc_rl_env/`：fast env + `gym_adapter`（SB3 env_checker 绿）+ PPO 训练入口 + config | ✅ 契约/奖励/投影/gym 适配测试全绿（系统 python 跳过 gym 测试，mc_venv 全绿）；**残差 PPO 已冻结 / 停止训练**（仅保留环境、契约与测试；不作为投递主张） |
| **系统辨识** | `system_identification/`：一阶滞后 + 延迟拟合（独立验证集） | ✅ 测试绿 |

## 参考核心基线（纯 MPC，离线，无扰动）

`python benchmark_tools/scripts/run_reference_benchmark.py`（结果归档于 `results/ref_core_baseline/`）：

| track | done | e_y_rms | e_y_p95 | e_y_max | e_psi_rms | qp_mean(us) | qp_fail | fallback |
|---|---|---|---|---|---|---|---|---|
| straight | COMPLETED | 0.122 | 0.349 | 0.355 | 0.107 | 1567 | 0 | 0 |
| circle (R=2) | COMPLETED | 0.055 | 0.160 | 0.248 | 0.058 | 2756 | 0 | 0 |
| s_curve | COMPLETED | 0.073 | 0.229 | 0.255 | 0.069 | 8222 | 0 | 0 |
| u_turn | COMPLETED | 0.090 | 0.248 | 0.255 | 0.083 | 8306 | 0 | 0 |

> 说明：这是**参考核心**（numpy 稠密 ADMM）的离线演示数字，不是正式 5-run Gazebo 门槛
> （计划 R23/R24 要求后者用 C++/OSQP 核心在 WSL2 执行）。直线/弯道 RMS 主要来自初始误差
> 恢复暂态（有加速度上界）；圆轨迹无前视时稳态偏移 < 0.01 m（前视会造成切弯偏移，MVP 默认 0）。
> **5-run 复现归档**：`results/ref_core_5run_baseline/`（4 轨迹 × 5 run = 20/20 COMPLETED、
> QP 0 失败；轨迹确定性一致、仅求解耗时随 CPU 调度波动——纯 MPC 无随机化时的预期行为，
> 扰动随机化属 C5 单元）。

## 本机运行（Windows，无需 ROS）

```bash
cd linear_mpc_controller
python -m pytest mpc_core trajectory_tools benchmark_tools mpc_rl_env system_identification test -q
python benchmark_tools/scripts/run_reference_benchmark.py --runs 1 --outdir outputs/bench_ref
```

依赖：numpy、pyyaml、pytest（均无第三方 QP 库需求；求解器为自带稠密 ADMM）。

## WSL2 / ROS2 门槛（待办，与计划 C2→C5 对齐）

```bash
# 1) ROS-free C++ 核心（仅 Eigen）
cmake -S . -B build && cmake --build build && ctest --test-dir build
# 2) ROS2 包
colcon build --packages-select linear_mpc_controller
ros2 launch linear_mpc_controller linear_mpc_sim.launch.py \
    world:=tracking_empty.sdf headless:=True use_sim_time:=True
# 3) RL 训练（需 torch/SB3/gymnasium）
python mpc_rl_env/algorithms/train_ppo_residual.py --seed 0
```

## 诚实边界（不冒充完成）

- 本机（Windows）没有 C++ 工具链/Eigen/OSQP：C++ 核心与 ROS2 节点**未在本机编译运行**，
  数学一致性以 Python 参考核心测试背书；编译/闭环/5-run 门槛在 WSL2 完成前不算验收通过。
- 无真实底盘：系统辨识结论限定在模型/仿真域（Sim2Sim），不声称实机部署（计划 R25/DoD）。
- 碰撞安全门依赖 costmap/collision-monitor 接口，未接通前不宣称碰撞约束投影（KTD12）。
- 软约束放宽、Pure Pursuit/PID 对照、SAC、探索路径接入（U11）为后续单元。
- 无"完整原始路径单发成功"：seal `real_path_status`（raw 录制计划单发 STALL、16/250 超 `ω_max`）；
  可用参考须先过可行性认证 `certify_reference` + `speed_profile` 沿路径剖面。参考可行性成果为
  参考准备层/离线 + 单一生产入口（`adapter_pipeline.prepare_tracker_reference`）；ROS
  `trajectory_server` 分发路径**未接线**该入口（集成边界）。
- 无硬实时声明：含 Gazebo TurtleBot3 闭环冒烟在内均为 WSL2/Gazebo 仿真结果。
- 最终验收、复现命令与冒烟证据：`docs/reference_feasibility_final.md`（`v0.3.0-engineered`）。

## 目录

```
mpc_core/            ROS-free Python 参考核心（frenet/model/qp/mpc/fallback/episode）
trajectory_tools/    参考轨迹生成与位姿补全
benchmark_tools/     指标 + manifest + 离线基准
mpc_rl_env/          fast 残差 RL 环境（envs/algorithms/config/tests）
system_identification/  一阶滞后/延迟拟合
include/ src/ test/  C++/Eigen 核心（WSL2）
ros2/ launch/ config/ worlds/ maps/   ROS2 骨架
docs/                baseline_audit / mpc_model_derivation / ros2_interface_contract
results/             参考核心基线归档
```

## 4060 门禁与增量结果(2026-09-04~06,全部本机生成)

| 门禁/研究 | 结果 | 证据 |
|---|---|---|
| A3 QP 门禁(OSQP=ON) | 真实 QP cycle kSolved,50 iter/425μs | `test_qp_cycle` + ctest(707ce2a) |
| A1 黄金对拍 | OsqpSolver ↔ Python AdmmQp 同题互验,max\|dx\| 与目标函数 1e-4 相对一致 | `test_qp_golden` + `test/golden_qp_vectors.txt`(98aace4) |
| A4 位移门禁 | Gazebo 闭环真实运动,位移 3.916m > 0.05m | `artifacts/motion_check/`(0fe3939) |
| U4 ROS 20-cell 基线 | **20/20 motion PASS**,位移 1.8-5.4m(4 轨迹 × 5 种子) | `results/mpc_baseline_ros/`(e11f889) |
| 残差 RL postmortem（**已冻结**） | 三轮受控迭代复盘,冻结决策+复活条件；**RL 不作为投递主张** | `docs/residual_rl_postmortem.md`(33edd98) |
| A3 归因表 | plant ZOH 修复后,纯 MPC 全随机化包络 24/24 格 4/4 完成,RMS ≤ 0.029m;离线 vs RL 环境落差=plant 发散 bug(已修 49464b3) | `results/attribution_study/`(fa51b07) |
| A4/C5 系统辨识 | τ=0.04s 恢复误差 3.9%(VAF 0.99);τ<采样分辨率如实记录不可辨;lookahead 接线诚实负结果(补偿量=噪声级) | `results/sysid_study/`(67f5def) |
| B4 桥(回放侧) | 录制 JSON → 参考轨迹补全 → 纯 MPC 离线跟踪:直段+90°弧 252 步完成,e_y_rms 0.004m | `benchmark_tools/scripts/replay_path_mpc.py`(fa51b07) |
| PPO v2（**已冻结**） | 不重训、不收口；判定与复活条件见 `docs/residual_rl_postmortem.md`,历史判定 `results/eval_residual_c8_iter2/` | 冻结 |

分层声明:以上全部为 WSL2/Gazebo 仿真结果,不含真实硬件声明。
## Day 10 封板主张(2026-09-08,canonical 来源 = ros2_tunnel_explorer `docs/seal_results.json` @ bline-merge b43760b)

> 本节的对外主张与简历/README 表同源渲染于该单份 JSON(含复现命令与 SHA);
> 单独改动此处数字即为口径违规。

| 主张 | 要点 | 证据 |
|---|---|---|
| A5 有状态投影 + 接受门 | Frenet 窗口化投影 + 位移预算接受门替代无状态全局最近点,消除折返车道误吸附(A5.1/A5.2/A4.2,含 C++ 镜像) | `mpc_core/` + `test_reacquire`/`test_controller_gate`(eddcf11 线) |
| 12 格消融矩阵 | 3 几何 × 2 v_min × 2 投影模式;失效轴 = 几何与 v_min **非投影实现**(10/12 格跨模式逐位相同) → 无需机动生成层 | `results/a8_replay/MATRIX.md` + cell_*.json(be39ffe/85276e3) |
| 运动学可行性审计 | 原始计划 16/250 参考点超 ω_max=2.0;修复判据 `v ≤ ω_max/\|κ\|` 已实现 + `omega_violations` 守卫 | `trajectory_tools/curvature_estimator.py`(85276e3) |
| 跨语言 golden 对拍 | projection_golden.json 双端消费逐位一致 | `ctest projection_golden` + pytest parity(eddcf11) |
| B6B Nav2 插件 | pluginlib 加载 + 生命周期 1 4/4/直线 7/7/弧线 8/8;分级失效契约;位姿变换进路径系;终端减速负向对照 intact 1.98 m vs 去除 13.14–13.16 m(Δ11.18 m) | `ros2/nav2_mpc_controller.*` + `scripts/nc_b6b_negative.sh`(ce0a256/855f846/4c2bcce) |
| 边界 | B6B **未接入覆盖链**;全仓无硬实时声明 | 见 seal JSON `boundaries` |

复现入口(均在 seal JSON `reproduce`): 矩阵 `python3 benchmark_tools/scripts/run_a8_matrix.py`;真实计划审计
`benchmark_tools/scripts/audit_b6_coverage.py`(served-map masks)。

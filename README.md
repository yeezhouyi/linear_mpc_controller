# linear_mpc_controller

差分底盘**线性时变 MPC 轨迹跟踪控制器**——`mpc_controller`（v0.2.1，ros2_control 线性 MPC 插件）的进阶项目，
依据《线性 MPC 与残差强化学习轨迹跟踪控制器进阶》计划（U1–U12、R1–R25）推进。
目标链：`上层路径 → Trajectory Adapter → Linear MPC → (残差 RL + 安全投影) → Velocity Arbiter → /cmd_vel → Gazebo`。

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
| **RL 环境** | `mpc_rl_env/`：fast env + `gym_adapter`（SB3 env_checker 绿）+ PPO 训练入口 + config | ✅ 契约/奖励/投影/gym 适配测试全绿（系统 python 跳过 gym 测试，mc_venv 全绿）；**残差 PPO 训练运行中（seed 0, 200k steps）** |
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

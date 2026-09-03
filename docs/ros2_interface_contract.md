# ROS2 接口契约（Jazzy / Gazebo Harmonic，standalone_tracking 模式）

> 目标：任何实现者只读本文件即可确定 topic / frame / QoS / 参数 / 时钟 / 控制权契约
> （R6–R9，U5）。当前代码状态：**Python 参考核心已完成并在本机验证；ROS2 层为
> 按契约铺开的源码骨架，需在 WSL2 colcon 编译 + Gazebo 闭环验收（见 README“边界”）。**

## 1. 运行模式与启动链

- `standalone_tracking`：只启动 Gazebo、底盘（TurtleBot3 差速）、传感器、
  轨迹服务器与本控制链；**不启动** Nav2 controller server。
- `nav2_explorer`（后续）：独立跟踪模式与 Nav2 模式互斥（KTD11）；
  未完成 `nav2_core::Controller` 插件前，只做 shadow/offline 回放。

```
cmd_vel_mpc / cmd_vel_nav / cmd_vel_stop
        └──► velocity_arbiter ──► cmd_vel_selected ──► velocity_smoother ──► collision_monitor ──► /cmd_vel
```

**最终执行 topic 只能有一个 publisher**（R8/KTD4）；MPC 核心不写 `/cmd_vel`，
只产出候选并交给 arbiter。

## 2. Topic / Frame / QoS 契约

| Topic | 类型 | QoS | 说明 |
|---|---|---|---|
| `/odom` | nav_msgs/Odometry | SensorDataQoS（或 reliability-reliable + 新鲜度检查） | 底盘状态来源 |
| `/tf` | tf2_msgs/TFMessage | 默认 | 需要 `odom → base_link`；用于可扩展定位，MVP 可直接用 odom 位姿 |
| `/cmd_vel` | geometry_msgs/Twist | SystemDefaultsQoS | **唯一**执行速度写入者（经仲裁链） |
| `~/reference` | nav_msgs/Path（或内部轨迹消息） | SystemDefaultsQoS | 上层/探索/清扫系统输入 |
| `~/diagnostics` | 结构化（Float64MultiArray 或诊断消息） | 默认 | 健康状态 + QP 诊断（字段表见 baseline_audit §诊断） |
| `~/residual_policy/action` | 自定义（序号/时间戳/model_id/Δu） | 默认 | 残差策略输出（U9），C++ 侧只做新鲜度/有限值/watchdog/回退 |

帧约定：轨迹与位姿均在 `map`/`odom` 系一致前提下使用 `odom` 系（standalone 由 Gazebo
提供静态 TF）；**MVP 不依赖全局定位**，仅 `odom→base_link`。

## 3. 参考轨迹输入与补全规则（R6）

- 内部类型 `TrajectoryPoint(s, x, y, yaw, κ, v, t)`，稠密 `Trajectory`（s 单调不减）。
- `nav_msgs/Path` 只有位姿时的**确定性补全规则**（Adapter）：
  1. 相邻点求弦方向 → 每点切线 `yaw`（退化点继承前点）；
  2. 差分求曲率 `κ[i] = wrap(yaw[i+1] − yaw[i−1]) / (2·Δs)`（端点复制邻值）；
  3. 速度补全：默认 `v_default`，弯道限速 `v = min(v_default, 0.6/|κ|)` 并整体夹到 `v_max`；
  4. 时间戳：由 `v` 与 `Δs` 积分生成（`Δt = Δs/v`），供过期判断。
- 补全结果写入 run manifest（可审计，AE9）。

## 4. 新鲜度 / 过期 / 故障类别（R9，健康状态）

| 条件 | 健康状态 | 行为 |
|---|---|---|
| 无轨迹 | `NO_REFERENCE` | 输出 0 |
| 轨迹时间戳超龄（阈值 `ref_max_age`） | `STALE_REFERENCE` | 输出 0 |
| TF 无法转换 / 缺帧 | `TF_INVALID` | 输出 0 |
| `/odom` 时间戳倒退/冻结超阈值 | `STATE_STALE` | 输出 0 |
| QP 超预算 | `QP_TIMEOUT` | 降速梯子 |
| QP 不可行 / NaN | `QP_INFEASIBLE` / `NAN_OUTPUT` | 降速梯子/归零 |

纪律：**仿真时间**（`use_sim_time:=True`）下所有时间戳比较用 sim clock，
禁止 wall time 与 sim time 混用；任何关键状态都不得持续发布未验证的非零速度。

## 5. 参数（config/*.yaml，R5，全部入配置哈希）

`linear_mpc_params.yaml`：`Ts, N, Q_diag, Q_F_diag, S_diag, v_min/v_max,
omega_max, a_max, alpha_max, lookahead_m, qp_*`（与 `MpcParams` 一一对应）；
`trajectory_adapter_params.yaml`：`v_default, v_max, ref_max_age, ds`；
`velocity_arbiter.yaml`：优先级/超时/唯一写者监控。

## 6. 控制权（R8/KTD4）

- MPC、MC（运控）、Nav2、安全停车都只提供候选；`velocity_arbiter` 单点裁决；
- 运行时检测“双写”：除 arbiter 外任何节点发布 `/cmd_vel` → 记录并进入安全态（测试项）；
- 节点生命周期：`linear_mpc_node` 用 lifecycle 节点，`on_deactivate` 保证零速输出。

## 7. 与计划其他单元的接口边界

- MPC 核心（C++/Eigen + `mpc_core`）**不依赖 ROS**；ROS2 节点负责消息/TF/生命周期/参数/诊断。
- 上层（自主探索/清扫）只提供 `nav_msgs/Path` 或内部轨迹；**不修改 MPC 内部状态**（U11）。
- 残差 RL（U9/U10）：Python 节点发布受限 Δu；**运行时安全裁决只在 C++ 侧**执行；
  Python `safety_projection.py` 仅做 golden-vector 测试。

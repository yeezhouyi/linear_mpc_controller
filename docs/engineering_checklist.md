# Engineering checklist — linear_mpc_controller (核验矩阵)

最终定位：可独立测试、可通过 ROS2/Nav2 使用的轨迹跟踪组件。
原则：已完成的部分只核验不重写；每项给出状态、证据、缺口。

| # | 能力 | 状态 | 证据 | 缺口 |
|---|---|---|---|---|
| 1 | 代码分层（核心无 ROS 可测） | ✅ | `mpc_core` / `trajectory_tools` 零 ROS import；CI `core-tests` job 在无 ROS runner 上跑全部 91 项 pytest（env -i 实测通过）；C++ 核心位于 `include/linear_mpc_controller/{model,mpc,safety}`，ROS 适配在 `ros2/` | 无 |
| 2 | 输入/输出/失败契约 | ✅（部分） | `Trajectory.__post_init__`：点数≥2、gear 形状合法（空路径/坏输入拒绝）；`trajectory_adapter` n<2 直接返回；投影丢失→A4.2 reacquire→PROJECTION_LOST；QP 失败→fallback_policy 分级；到达判据=A5.3 watermark | NaN 输入的显式单测未单列（由 golden 数值路径隐式覆盖）——记为待办 |
| 3 | 参数可管理 | ✅（本批补齐） | `MpcParams.__post_init__` 拒绝非法配置（Ts/N/v 范围/ω/a 上限/qp_max_iter/负权重/投影模式），9 项契约测试；node 参数 yaml 覆盖由 `test_ros_contract.py` 结构性校验 | 运行中可改参数集合的显式白名单未成文 |
| 4 | 性能可测量 | ✅（部分） | 每控制周期 diag：qp_time_us、qp_iterations、fallback、constraint_violation；controller_compare 表已报 p95/p99 | 投影与 QP 构建/求解的分项计时未拆分——记为待办（先指出大头再优化） |
| 5 | 回归测试自动执行 | ✅ | 91 pytest（模型/QP/门控/golden/矩阵）+ 7 C++ gtest + `test_qp_golden`/`test_projection_golden` 数值对拍 + a8 12 格闭环矩阵回归；`pytest`/`colcon test` 单入口，失败非零退出 | 无 |
| 6 | ROS2 稳定使用入口 | ✅（部分） | `linear_mpc_node`（lifecycle）+ `nav2_mpc_controller` 插件 + `trajectory_server`（换路径）+ `velocity_arbiter`（单写者，结构性契约测试）；config/linear_mpc_params.yaml | 面向外部的最小 quickstart 文档未成文——记为待办 |
| 7 | CI | ✅（本批新增） | `.github/workflows/ci.yml`：job1 无 ROS 核心测试；job2 ros:jazzy 容器 colcon build+test | 首跑结果待观察 |

已核验的诚实失败记录（保留为亮点，不在本表展开）：执行滞后 0.15 s 下 QP 连续不可行→STALL（`docs/controller_compare.md`），滞后补偿为开放候选修复。

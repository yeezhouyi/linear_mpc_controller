# B6 全链演示(清洁机器人统一叙事)

一条命令式复现序列(每步产物可审计);全部为 WSL2/Gazebo 仿真。

```
[1] 探索建图    tunnel_explorer Stage 3D 栈(branching world)
                 └─ record_explorer_path.py  → explorer_path.json (B4 recorder)
[2] 保存地图    map_saver → cleaning_map.yaml/pgm
[3] 覆盖规划    cleaning_mode/coverage_planner → cleaning_path.json(弓形+连接线)
                   或 coverage-cleaning-track 的 ScanlinePlanner(C++ 路径)
[4] MPC 跟踪    tracking_sim.launch.py + trajectory_server path_file:=cleaning_path.json
                   └─ /odom 录制 → 运动证据
[5] 覆盖审计    audit_coverage_bag.py(sweep_radius=0.25) → 双分母 mask + JSON
```

前置状态(全部已验证):
- MPC 闭环:tracking_sim 位移 3.916m PASS(0fe3939)
- 覆盖规划:cleaning_mode 30-run 全 OK,路径较基线短 48%(a6cfd02)
- 审计:coverage_audit JSON + mask 导出管线(7332358)
- 接口:trajectory_server path_file 模式(093db85)

已知约束:
- 单 /cmd_vel 写者(arbiter);探索器仅提供 Path(R14)
- 覆盖率双分母口径(R10):coverage_task / coverage_known_free 并列
- 全部结论限 WSL2/Gazebo 仿真

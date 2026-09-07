# C++ core ctest — 完整启用记录 (2026-09-08)

## 关键命令（doc §10.3 "静默少测"陷阱的解法）
```bash
source /opt/ros/jazzy/setup.bash          # osqp/nav_msgs 等依赖随 ROS 环境提供
cmake -S . -B build_core -DLINEAR_MPC_WITH_OSQP=ON -DBUILD_TESTING=ON \
      -DCMAKE_PREFIX_PATH=/opt/ros/jazzy
cmake --build build_core -j4
(cd build_core && ctest --output-on-failure)
```
- 不 source ROS 环境时 find_package(osqp) 失败 → 只剩 core 1 项（本机曾实测复现该"静默少测"）。
- 完整启用后：**3/3 通过** —— core（linearisation/discretisation/frenet_signs/fallback 四组）、qp_cycle、**qp_golden**（对照 `test/golden_qp_vectors.txt`，即 A7.1 golden 对拍的 C++ 侧，通过）。
- 同时编译出 `linear_mpc_node`（ros2 节点，colcon/ament 路径）。

## 意义
- A7.1（golden 对拍）：QP golden 在 C++ 侧逐位通过（红灯未亮）。
- D 线 C++ core（A5B 基础层）本机验证 3/3；投影序列 golden 的 C++ 侧对拍仍待 A5B 完整落地（记录于 R13 阻塞清单）。

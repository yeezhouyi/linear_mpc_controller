# Python / C++ 投影序列对拍 —— 结果记录 (2026-09-08)

## 运行
```bash
cmake --build build_core --target projection_parity_dump -j4   # (OSQP 启用构建)
python3 benchmark_tools/scripts/projection_parity.py
```

## 结果
- **61/61 记录一致**（stage / arc / e_y 逐位，1e-6）：
  - straight：300 拍沿 +x 横向 0.05 步进 + 3 m 跳变；
  - circle：沿圆弧横向 0.05 偏移步进（含接缝区外点）；
  - fold：出向行驶 + 车道间点 + 返向车道点。
- 链式语义一致：s_prev = 前拍 raw arc（stage 3 保持），yaw + gear(+1) 传入；
  同车道坍缩（切向 ≤0.05 rad）两侧一致；跨车道无状态并列两侧均 stage 3。

## 意义
- A7.2 #5 的跨语言投影序列对拍：C++（closestPointWindowed）与 Python
  （mpc_core.frenet.closest_point）在共享序列上逐位一致——A5B C++ 镜像
  与 Python 参考达成语义对拍（本文件覆盖的几何/位姿集合）。
- 后续：把本 harness 的序列扩展为签入 golden 文件（含参数头）并纳入 CI
  （与文档"各自对同一文件比对"的 C++ 侧要求对齐）。

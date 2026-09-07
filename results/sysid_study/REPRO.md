# sysid 研究 —— 复现记录 (2026-09-08)

## 复现
`python3 mpc_rl_env/tools/sysid_study.py` 重跑结果与
`results/sysid_study/sysid_results.json` **逐位一致**（identification
3 行 + lookahead_compare 4 行 + fitted_delay_wire_m 0.05），研究可复现。

## 结果（诚实口径）
- v 通道阶跃/PRBS 激励 + ZOH plant 拟合：
  - true_lag 0.01 s → fit inf（低于采样分辨率，如实记录为不可辨识）；
  - 0.02 s → fit 0.0142 s（τ 误差 29.1%）；0.04 s → fit 0.0385 s（3.9%）；
  - gain 0.994-0.996、VAF 0.979-0.989（一致性好）。
- lookahead 接线对照（fitted_delay_wire_m=0.05）：
  - s_curve: lookahead=0 → 0.0569；lookahead=fit → 0.0585（无增益，略差）
  - u_turn : lookahead=0 → 0.0337；lookahead=fit → 0.0449（无增益）
- 结论：该 sim 包络内延迟补偿 lookahead 无正收益，与仓库"lookahead 默认
  0（C5 延迟补偿量化前保持）"的约定一致；C 线系统辨识层存在且可复现，
  延迟补偿收益需在真实/更强滞后包络（MPC 实际退化区）再评。

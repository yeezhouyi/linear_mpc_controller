# 回放式全长审计的结构性边界(2026-09-07)

900s 重跑(18000 步):e_y_rms 稳定 0.0099m / max 0.0126m——跟踪质量
全程保持。但 completed=false:探索路径含**折返**(explorer 重访),
closest_point 锚点在折返处无法单调推进 → 回放式"arc 进度"审计对
折叠路径不适定。

结论:全长覆盖审计改走实车链路(live path_file 跟踪 + odom bag →
audit_coverage_bag.py 双分母)——跟踪质量证据已由本文件与 600s 版
(RMS 9.9mm/101.4m)充分建立;折叠锚点问题记录为 MPC 参考投影的
已知限制(与 R6 路径单调性假设一致)。

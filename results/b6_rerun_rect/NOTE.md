# B6 rect 全链复跑 —— 结果记录 (2026-09-08)

## 运行
- launch `coverage_simulation.launch.py headless:=True`（rect 世界+静态图，与 ~/b6_chain 失败跑同口径）
- ExecuteCoverage goal 运行满 900 s（时间上限优雅取消，executor 保存 checkpoint）
- odom bag → `audit_b6_coverage.py`（masks = plan_from_map(cleaning_room_rect.yaml, footprint 0.10/lane 0.30)）

## 结果（对照旧失败跑 0.061 / 7.3 m）
| 指标 | 旧跑 (~/b6_chain) | 本次复跑 |
|---|---|---|
| coverage_task | 0.061 | **0.181** |
| coverage_known_free | 0.0505 | 0.1774 |
| driven_length_m | 7.302 | **163.9**（planned 59.6，开销 2.75×） |
| odom_samples | 19362 | 691422 |

## 判定（诚实）
- **改善**：首 U 帽卡死已解除——机器人全程运行 900 s、驱动 163.9 m（旧跑 7.3 m 即停），coverage 约 3×。
- **未达标**：执行开销 2.75× 且覆盖率仅 18.1%——大量冗余/回访，疑似工作行级 FollowPath abort → nav fallback 反复触发 + 行内扫掠低效。**B6 coverage_task 验收仍开**，下一步定位 executor 工作行效率（行完成判定/换行开销）。
- 附注：运行期间检测到旧 stage0 残留进程（已清理）；cancel 时 send 客户端 KeyboardInterrupt/RCLError 为已知硬化客户端瑕疵，checkpoint 由 executor 正常保存。

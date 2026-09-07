# 部署指南 — linear_mpc_controller（4060 / WSL2）

单机部署与运行手册。仓库即源码与证据（`results/`、`outputs/` 均已 git 跟踪）。

## 环境
- 宿主：Windows + WSL2 (Ubuntu-24.04, zhouyi)；仓库 `~/ros2_ws/src/linear_mpc_controller`（`git@github.com:yeezhouyi/linear_mpc_controller`）。
- 核心测试：系统 python3（numpy 1.26 系，见 `conftest.py` 自动加路径，无需 pip install）。
- RL（A7 PPO v2 / U5-C7）：venv `/home/zhouyi/mc_venv`（torch 2.13+cu130 / gymnasium 1.3 / stable-baselines3 2.9）。缺依赖时状态脚本会打印 `PPO_DEPENDENCY_BLOCKED`。

## 测试
```bash
cd ~/ros2_ws/src/linear_mpc_controller
python3 -m pytest            # 由 pytest.ini 统一 ignore build*/log 等 → 97 passed, 1 skipped
bash scripts/ppo_v2_status.sh   # RL 依赖 + 产物 + 判定检查
```

## A7 PPO v2（U5）：状态与运行
- 现状（证据在仓库内，非本会话新结论）：**C7 诚实负结果** — `results/eval_residual_c8_iter2/VERDICT.md`：
  修复环境（ZOH plant、全轨迹课程、1M 步、eval 分离）重训后策略全部完成但跟踪误差比纯 MPC 差 2–4×、平滑代价高一个量级；`proj_triggers` 高企说明策略以贴限幅抢进度。判定：**残差路线暂停**；诚实基线 = 纯 MPC + 安全投影（README 数字）。不引入 SAC/GRPO（R8）。
- 复现训练（长任务，GPU；仅需时执行）：
```bash
/home/zhouyi/mc_venv/bin/python3 mpc_rl_env/algorithms/train_ppo_residual.py \
    --config config/ppo_curriculum_v2.yaml --total-timesteps 1000000 --outdir outputs/ppo_residual_iter3
```
- 评估（未见 seeds 100–102 × 4 轨迹 × 三控制器）：
```bash
/home/zhouyi/mc_venv/bin/python3 mpc_rl_env/algorithms/evaluate_policy.py \
    --checkpoint results/eval_residual_c8_iter2/train_seed_0/checkpoint.zip \
    --tracks straight,circle,s_curve,u_turn --seeds 100,101,102
```
- 提示：任何新的残差实验前先确认"纯 MPC 在该难度包络内存在系统性误差"（A3 归因表），否则残差无学习空间；未达到 C4 准入门槛时保持 `use_residual_policy: false`。

## 同步 / 部署
```bash
bash scripts/sync_wsl_windows.sh all   # git push main + 镜像交接件到 Windows 项目/_handover_docs
```
- GitHub 用 SSH（Windows 侧 HTTPS 不稳）：`git remote set-url origin git@github.com:yeezhouyi/linear_mpc_controller.git`。
- ROS 2/Gazebo 侧工作区 `~/ros2_ws`（colcon），本仓库亦可 colcon 构建；离线核心证据以 pytest/ctest 为准。

## 结果落盘约定
- `results/`：每次运行一个子目录（VERDICT.md + eval 明细 + train log），全部 git 跟踪；
- `outputs/ppo_residual/`：本地便利副本（.gitignore 忽略）；权威可复现 checkpoint + run_config 在 `results/eval_residual_c8_iter2/train_seed_0/`（git 跟踪）；
- `交接文档.md`（Windows 项目根）：跨执行线单一事实源，逐轮 R 日志回写。

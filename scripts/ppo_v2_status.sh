#!/usr/bin/env bash
# A7 PPO v2 (U5/C7) status check: RL dependency probe + artifact inventory.
# Usage: bash scripts/ppo_v2_status.sh   (run from the repo root)
set -u
cd "$(dirname "$0")/.."

echo "== RL venv probe =="
if [ -x /home/zhouyi/mc_venv/bin/python3 ]; then
  /home/zhouyi/mc_venv/bin/python3 - <<'PY'
mods = ["torch", "gymnasium", "stable_baselines3", "yaml"]
missing = []
for m in mods:
    try:
        mod = __import__(m)
        print(f"  {m}: OK {getattr(mod, '__version__', '')}")
    except Exception as e:
        missing.append(m)
        print(f"  {m}: MISSING ({type(e).__name__})")
print("PPO_DEPENDENCY_BLOCKED" if missing else "PPO_DEPENDENCY_OK")
PY
else
  echo "  no /home/zhouyi/mc_venv -> PPO_DEPENDENCY_BLOCKED (venv missing)"
fi

echo "== scripts / config / artifacts =="
for f in mpc_rl_env/algorithms/train_ppo_residual.py \
         mpc_rl_env/algorithms/evaluate_policy.py \
         config/ppo_curriculum_v2.yaml \
         results/eval_residual_c8_iter2/VERDICT.md \
         results/eval_residual_c8_iter2/eval/eval_results.md \
         outputs/ppo_residual/checkpoint.zip; do
  if [ -e "$f" ]; then echo "  OK  $f"; else echo "  --  $f"; fi
done

echo "== tracked? =="
git ls-files --error-unmatch mpc_rl_env/algorithms/train_ppo_residual.py \
  mpc_rl_env/algorithms/evaluate_policy.py config/ppo_curriculum_v2.yaml \
  results/eval_residual_c8_iter2/VERDICT.md >/dev/null 2>&1 \
  && echo "  core scripts/config/verdict tracked" || echo "  WARNING: something untracked"

echo "== last verdict (head) =="
head -8 results/eval_residual_c8_iter2/VERDICT.md 2>/dev/null

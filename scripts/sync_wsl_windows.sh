#!/usr/bin/env bash
# WSL2 <-> Windows / GitHub sync & deploy helper for linear_mpc_controller.
# Usage: bash scripts/sync_wsl_windows.sh [push|pull|mirror|all]
#   push   : commit nothing; just push main to origin (SSH)
#   pull   : fast-forward main from origin
#   mirror : copy handover-relevant artifacts to the Windows 项目/_handover_docs
#   all    : push + mirror (default)
set -u
cd "$(dirname "$0")/.."
MODE="${1:-all}"
WIN_DEST="/mnt/c/Users/周易/Desktop/项目/_handover_docs/linear_mpc_controller"

if [ "$MODE" = push ] || [ "$MODE" = all ]; then
  echo "== git push main (SSH origin) =="
  git remote -v | grep -q 'git@github.com' && git push origin main || \
    { echo "origin is not SSH; HTTPS from Windows is flaky, use SSH or run in WSL:"; \
      echo "  git remote set-url origin git@github.com:yeezhouyi/linear_mpc_controller.git"; }
fi
if [ "$MODE" = pull ]; then
  echo "== git pull --ff-only origin main =="
  git pull --ff-only origin main
fi
if [ "$MODE" = mirror ] || [ "$MODE" = all ]; then
  echo "== mirror handover artifacts -> $WIN_DEST =="
  mkdir -p "$WIN_DEST"
  for src in docs/ scripts/ config/ppo_curriculum_v2.yaml \
             results/eval_residual_c8_iter2/VERDICT.md \
             results/eval_residual_c8_iter2/eval/eval_results.md; do
    if [ -e "$src" ]; then
      mkdir -p "$WIN_DEST/$(dirname "$src")"
      cp -r "$src" "$WIN_DEST/$src"
      echo "  copied $src"
    fi
  done
  # pytest/RL status snapshot
  bash scripts/ppo_v2_status.sh > "$WIN_DEST/ppo_v2_status.txt" 2>&1 \
    && echo "  wrote ppo_v2_status.txt"
fi
echo "done ($MODE)"

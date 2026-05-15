#!/bin/bash
# scripts/train_act.sh
# Train an ACT model for a single trial. Run in a dedicated tmux window.
#
# PARALLEL TRAINING WORKFLOW:
#   Start T1 training as soon as T1 demos finish (don't wait for T2/T3).
#   Open a new tmux window for each training run:
#
#     tmux new-window -t aic -n "train_t1"
#     ./scripts/train_act.sh 1
#
#   The A10G has 24 GB VRAM — training and collection don't conflict (different
#   processes). Typical training time: ~2 hours per trial on A10G.
#
#   If you hit CUDA OOM:  Ctrl+C and re-run adding:  --batch_size=4
#
# Usage:  ./scripts/train_act.sh <trial_number>
# Example: ./scripts/train_act.sh 1
set -euo pipefail

cd ~/ws_aic/src/aic
export PATH="$HOME/.pixi/bin:$PATH"

TRIAL=${1:?Usage: $0 <trial_number> (1, 2, or 3)}

case $TRIAL in
  1) REPO="local/aic-t1-demos" ;;
  2) REPO="local/aic-t2-demos" ;;
  3) REPO="local/aic-t3-demos" ;;
  *) echo "Invalid trial: $TRIAL (must be 1, 2, or 3)"; exit 1 ;;
esac

echo "[$(date)] Training Trial ${TRIAL} from dataset: ${REPO}"
echo "Output: outputs/act_trial${TRIAL}/"
echo "If CUDA OOM: Ctrl+C and re-run adding:  --batch_size=4"
echo ""

pixi run lerobot-train \
  --config_path aic_example_policies/configs/act_cable_insertion.yaml \
  --dataset.repo_id="${REPO}" \
  --output_dir="outputs/act_trial${TRIAL}" \
  --policy.device=cuda

echo ""
echo "[$(date)] Trial ${TRIAL} training complete."
echo "Checkpoints:"
ls "outputs/act_trial${TRIAL}/checkpoints/" 2>/dev/null || echo "(none found)"
echo ""
echo "Next: copy the latest LeRobot checkpoint."
echo "  Local (from your laptop): ./scripts/copy_checkpoints.sh <ec2-dns>"
echo "  Or on EC2: ls outputs/act_trial${TRIAL}/checkpoints/last/pretrained_model/"

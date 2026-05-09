#!/bin/bash
# scripts/update_and_run.sh
# Run at the START of every EC2 session to sync the workspace.
# Pulls latest code, reinstalls pixi packages, and prints session status.
#
# Usage:  ./scripts/update_and_run.sh
# Run from: ~/ws_aic/src/aic   OR pass path as $1
set -euo pipefail

export PATH="$HOME/.pixi/bin:$PATH"
export DISPLAY=:99
export DBX_CONTAINER_MANAGER=docker

AIC_DIR="${1:-$HOME/ws_aic/src/aic}"
cd "$AIC_DIR"

echo "========================================================"
echo "AIC Session Start — $(date)"
echo "Dir: $AIC_DIR"
echo "========================================================"

# ---------------------------------------------------------------
# 1. Virtual display (keep Gazebo headless even after reconnect)
# ---------------------------------------------------------------
pgrep Xvfb >/dev/null && echo "[display] Xvfb already running" \
  || { Xvfb :99 -screen 0 1280x1024x24 & echo "[display] Xvfb started on :99"; }
sleep 1

# ---------------------------------------------------------------
# 2. Git pull
# ---------------------------------------------------------------
echo ""
echo "[git] Pulling latest code..."
git fetch origin
LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse @{u} 2>/dev/null || echo "$LOCAL")
if [ "$LOCAL" = "$REMOTE" ]; then
  echo "[git] Already up to date ($(git rev-parse --short HEAD))"
else
  git pull origin "$(git branch --show-current)"
  echo "[git] Updated to $(git rev-parse --short HEAD)"
fi

# ---------------------------------------------------------------
# 3. Reinstall pixi packages (picks up Python package changes)
# ---------------------------------------------------------------
echo ""
echo "[pixi] Reinstalling packages..."
pixi install --locked
pixi reinstall ros-kilted-aic-example-policies 2>/dev/null || true
echo "[pixi] Done."

# ---------------------------------------------------------------
# 4. Generate demo configs and verify counts
# ---------------------------------------------------------------
echo ""
echo "[configs] Generating demo configs..."
python3 aic_example_policies/scripts/generate_demo_configs.py

T1_COUNT=$(ls aic_example_policies/configs/demo_configs/t1/ 2>/dev/null | wc -l)
T2_COUNT=$(ls aic_example_policies/configs/demo_configs/t2/ 2>/dev/null | wc -l)
T3_COUNT=$(ls aic_example_policies/configs/demo_configs/t3/ 2>/dev/null | wc -l)

echo "Config counts: T1=${T1_COUNT} T2=${T2_COUNT} T3=${T3_COUNT}"

if [ "$T1_COUNT" -ne 150 ] || [ "$T2_COUNT" -ne 150 ] || [ "$T3_COUNT" -ne 200 ]; then
  echo "ERROR: Config counts are wrong. Expected 150/150/200."
  exit 1
fi
echo "PASS: Config counts correct."

# ---------------------------------------------------------------
# 5. Docker / distrobox health check
# ---------------------------------------------------------------
echo ""
echo "[docker] Checking Docker daemon..."
if sg docker -c "docker info" &>/dev/null; then
  echo "[docker] Docker OK."
else
  echo "[docker] WARNING: Docker not accessible. You may need to re-login or run 'newgrp docker'."
fi

if sg docker -c "distrobox list 2>/dev/null | grep -q aic_eval"; then
  echo "[docker] distrobox aic_eval OK."
else
  echo "[docker] WARNING: distrobox aic_eval not found. Run scripts/aws_setup.sh."
fi

# ---------------------------------------------------------------
# 6. Training status
# ---------------------------------------------------------------
echo ""
echo "[training] Checkpoint status:"
for TRIAL in 1 2 3; do
  BEST_DIR="outputs/act_trial${TRIAL}/checkpoints/best"
  LAST_DIR=$(ls -dt "outputs/act_trial${TRIAL}/checkpoints/"* 2>/dev/null | head -1 || true)
  if [ -d "$BEST_DIR" ]; then
    echo "  Trial ${TRIAL}: DONE — best/ exists ($(ls "$BEST_DIR" | wc -l) files)"
  elif [ -n "$LAST_DIR" ]; then
    LAST_STEP=$(basename "$LAST_DIR")
    echo "  Trial ${TRIAL}: IN PROGRESS — latest checkpoint: $LAST_STEP"
  else
    echo "  Trial ${TRIAL}: NOT STARTED"
  fi
done

# ---------------------------------------------------------------
# 7. Demo collection status
# ---------------------------------------------------------------
LOG_FILE=~/ws_aic/collection_log.txt
echo ""
echo "[collection] Demo log (${LOG_FILE}):"
if [ -f "$LOG_FILE" ]; then
  for TRIAL in t1 t2 t3; do
    DONE=$(grep -c "^DONE:${TRIAL}_" "$LOG_FILE" 2>/dev/null || echo 0)
    FAIL=$(grep -c "^FAILED:${TRIAL}_" "$LOG_FILE" 2>/dev/null || echo 0)
    echo "  ${TRIAL}: ${DONE} done, ${FAIL} failed"
  done
else
  echo "  No collection log found (collection not yet started)."
fi

# ---------------------------------------------------------------
# 8. Active tmux sessions
# ---------------------------------------------------------------
echo ""
echo "[tmux] Active sessions:"
tmux list-sessions 2>/dev/null || echo "  (none)"

echo ""
echo "========================================================"
echo "Session ready. Common next steps:"
echo "  Verify env:   ./scripts/verify_env.sh"
echo "  Collect demos: ./scripts/collect_demos.sh"
echo "  Train trial:  ./scripts/train_act.sh <1|2|3>"
echo "  (In new window: tmux new-window -t aic -n train_t1)"
echo "========================================================"

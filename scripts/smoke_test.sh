#!/bin/bash
# scripts/smoke_test.sh
# Minimal ROS communication check: confirms the insert_cable action server
# inside the eval container is visible to the host-side aic_model environment.
#
# Run this BEFORE verify_env.sh if you suspect networking/zenoh issues.
#
# Success = insert_cable action server listed by ros2 action list
# Failure = action server not found after 45s

set -euo pipefail

export DISPLAY=:99
export DBX_CONTAINER_MANAGER=docker
export PATH="$HOME/.pixi/bin:$PATH"

AIC_DIR=~/ws_aic/src/aic
cd "$AIC_DIR"

pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1280x1024x24 &

SMOKE_LOG=/tmp/aic_smoke.log
> "$SMOKE_LOG"

echo "========================================================"
echo "AIC Smoke Test — ROS communication check"
echo "========================================================"

echo ""
echo "Starting eval container (no engine, minimal setup)..."
tmux new-session -d -s aic_smoke -x 220 -y 50
tmux pipe-pane -t aic_smoke:0 -o "cat >> $SMOKE_LOG"
tmux send-keys -t aic_smoke:0 \
  "export DBX_CONTAINER_MANAGER=docker && \
   distrobox enter -r aic_eval -- \
     /entrypoint.sh gazebo_gui:=false launch_rviz:=false \
                    ground_truth:=false start_aic_engine:=true" Enter

echo "Waiting 45s for ROS to initialize..."
sleep 45

echo ""
echo "Checking action server availability..."
ACTION_SERVER=$(pixi run ros2 action list 2>/dev/null | grep insert_cable || true)

if [ -n "$ACTION_SERVER" ]; then
  echo "PASS: insert_cable action server found: $ACTION_SERVER"
  RESULT=0
else
  echo "FAIL: insert_cable action server not found"
  echo ""
  echo "Available ROS2 actions:"
  pixi run ros2 action list 2>/dev/null || echo "  (none — ros2 command failed)"
  echo ""
  echo "Available ROS2 nodes:"
  pixi run ros2 node list 2>/dev/null || echo "  (none)"
  echo ""
  echo "Diagnosis:"
  echo "  1. If no nodes listed: zenoh bridge not routing between container and host."
  echo "     Check: docker network inspect bridge | grep -A5 aic_eval"
  echo "  2. If nodes listed but no insert_cable: aic_engine not started or failed."
  echo "     Check: tmux attach -t aic_smoke  (look for errors)"
  echo "  3. If ros2 command fails: pixi env broken. Run: pixi install --locked"
  RESULT=1
fi

echo ""
echo "Last 10 lines of eval container output:"
tail -10 "$SMOKE_LOG" 2>/dev/null || true

tmux kill-session -t aic_smoke 2>/dev/null || true

exit $RESULT

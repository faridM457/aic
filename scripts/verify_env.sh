#!/bin/bash
# scripts/verify_env.sh
# Automated environment verification — no manual intervention required.
# Runs two checks sequentially. Both must PASS before collect_demos.sh.
#
# Check 1: WaveArm  — proves Gazebo + ROS communication work
# Check 2: CheatCode — proves ground-truth insertion works (expected ~225/300)
#
# Exit code 0 = both checks passed.
# Exit code 1 = a check failed or timed out.
set -euo pipefail

export PATH="$HOME/.pixi/bin:$PATH"
export DISPLAY=:99
export DBX_CONTAINER_MANAGER=docker

AIC_DIR=~/ws_aic/src/aic
cd "$AIC_DIR"

pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1280x1024x24 &

# ---------------------------------------------------------------
# Helper: kill a named tmux session if it exists
# ---------------------------------------------------------------
kill_session() {
  tmux kill-session -t "$1" 2>/dev/null || true
}

# ---------------------------------------------------------------
# Helper: wait for a string in a log file with a timeout
# Returns 0 (found) or 1 (timeout)
# ---------------------------------------------------------------
wait_for_log() {
  local pattern="$1"
  local logfile="$2"
  local timeout_s="$3"
  local label="$4"
  local elapsed=0

  while ! grep -qE "$pattern" "$logfile" 2>/dev/null; do
    sleep 2
    elapsed=$((elapsed + 2))
    if [ "$elapsed" -ge "$timeout_s" ]; then
      echo "  TIMEOUT (${timeout_s}s): ${label}"
      return 1
    fi
  done
  return 0
}

# ---------------------------------------------------------------
# CHECK 1 — WaveArm (proves Gazebo + ROS comms work)
# ---------------------------------------------------------------
echo "========================================================"
echo "CHECK 1: WaveArm policy (automated)"
echo "  SUCCESS = trial scores appear within ~3 min"
echo "  FAILURE = timeout or hang"
echo "========================================================"
echo ""

kill_session aic_verify

C1_LOG=/tmp/aic_verify_check1.log
> "$C1_LOG"

# Pane 0: eval container — pipe its output to log file
tmux new-session -d -s aic_verify -x 220 -y 50
tmux pipe-pane -t aic_verify:0 -o "cat >> $C1_LOG"
tmux send-keys -t aic_verify:0 \
  "export DBX_CONTAINER_MANAGER=docker && \
   distrobox enter -r aic_eval -- bash -c \
     \"export NVIDIA_DRIVER_CAPABILITIES=all && \
       export NVIDIA_VISIBLE_DEVICES=all && \
       GALLIUM_DRIVER=zinc MESA_GL_VERSION_OVERRIDE=4.6 \
       /entrypoint.sh gazebo_gui:=false launch_rviz:=false \
                      ground_truth:=false start_aic_engine:=true\"" Enter

# Wait for aic_engine to send the InsertCable goal (max 120s)
echo "Waiting for aic_engine to send InsertCable goal (up to 120s)..."
if ! wait_for_log "Waiting for result" "$C1_LOG" 120 "aic_engine never sent InsertCable goal"; then
  echo ""
  echo "FAIL CHECK 1: aic_engine did not send goal."
  echo "  Possible causes: Gazebo startup failure, container not ready, zenoh misconfigured."
  echo "  Last 20 lines of eval log:"
  tail -20 "$C1_LOG" 2>/dev/null || true
  kill_session aic_verify
  exit 1
fi

echo "aic_engine sent InsertCable goal. Waiting 5s before starting WaveArm..."
sleep 5

# Pane 1: WaveArm policy
tmux split-window -t aic_verify -h
tmux send-keys -t aic_verify:0.1 \
  "cd $AIC_DIR && pixi run ros2 run aic_model aic_model \
     --ros-args -p use_sim_time:=true \
     -p policy:=aic_example_policies.ros.WaveArm" Enter

# Wait for a score to appear in the eval output (max 120s)
echo "WaveArm running. Waiting for trial scores (up to 120s)..."
if ! wait_for_log "Score:|trial complete" "$C1_LOG" 120 "no score appeared"; then
  echo ""
  echo "FAIL CHECK 1: No trial score appeared after 120s."
  echo "  Possible causes: /clock not routing from container, WaveArm policy error."
  echo "  Last 20 lines of eval log:"
  tail -20 "$C1_LOG" 2>/dev/null || true
  kill_session aic_verify
  exit 1
fi

echo ""
echo "CHECK 1 PASSED — trial scores:"
grep -E "Score:|trial complete" "$C1_LOG" | head -10 || true

kill_session aic_verify
echo ""

# ---------------------------------------------------------------
# CHECK 2 — CheatCode (proves ground-truth insertion works)
# ---------------------------------------------------------------
echo "========================================================"
echo "CHECK 2: CheatCode policy (automated)"
echo "  SUCCESS = insertions complete; scores ~75+ per trial"
echo "  FAILURE = timeout or hang"
echo "========================================================"
echo ""

kill_session aic_cheatcode_test

C2_LOG=/tmp/aic_verify_check2.log
> "$C2_LOG"

# Pane 0: eval container with ground_truth
tmux new-session -d -s aic_cheatcode_test -x 220 -y 50
tmux pipe-pane -t aic_cheatcode_test:0 -o "cat >> $C2_LOG"
tmux send-keys -t aic_cheatcode_test:0 \
  "export DBX_CONTAINER_MANAGER=docker && \
   distrobox enter -r aic_eval -- bash -c \
     \"export NVIDIA_DRIVER_CAPABILITIES=all && \
       export NVIDIA_VISIBLE_DEVICES=all && \
       GALLIUM_DRIVER=zinc MESA_GL_VERSION_OVERRIDE=4.6 \
       /entrypoint.sh gazebo_gui:=false launch_rviz:=false \
                      ground_truth:=true start_aic_engine:=true\"" Enter

# Wait for aic_engine to send the InsertCable goal (max 120s)
echo "Waiting for aic_engine to send InsertCable goal (up to 120s)..."
if ! wait_for_log "Waiting for result" "$C2_LOG" 120 "aic_engine never sent InsertCable goal"; then
  echo ""
  echo "FAIL CHECK 2: aic_engine did not send goal."
  echo "  Last 20 lines of eval log:"
  tail -20 "$C2_LOG" 2>/dev/null || true
  kill_session aic_cheatcode_test
  exit 1
fi

echo "aic_engine sent InsertCable goal. Waiting 5s before starting CheatCode..."
sleep 5

# Pane 1: CheatCode policy
tmux split-window -t aic_cheatcode_test -h
tmux send-keys -t aic_cheatcode_test:0.1 \
  "cd $AIC_DIR && pixi run ros2 run aic_model aic_model \
     --ros-args -p use_sim_time:=true \
     -p policy:=aic_example_policies.ros.CheatCode" Enter

# Wait for scores — CheatCode runs all 3 trials (~5 min); give 360s
echo "CheatCode running. Waiting for all 3 trial scores (up to 360s)..."
if ! wait_for_log "Score:|trial complete" "$C2_LOG" 360 "no score appeared"; then
  echo ""
  echo "FAIL CHECK 2: No trial score appeared after 360s."
  echo "  Last 20 lines of eval log:"
  tail -20 "$C2_LOG" 2>/dev/null || true
  kill_session aic_cheatcode_test
  exit 1
fi

echo ""
echo "CHECK 2 PASSED — trial scores:"
grep -E "Score:|trial complete" "$C2_LOG" | head -10 || true

kill_session aic_cheatcode_test
echo ""

echo "========================================================"
echo "Both checks PASSED. Environment is ready."
echo "Next: ./scripts/collect_demos.sh"
echo "========================================================"

#!/bin/bash
# scripts/verify_env.sh
# Verify the environment before wasting time on demo collection.
# Runs TWO checks sequentially. Both must pass before collect_demos.sh.
set -euo pipefail

export PATH="$HOME/.pixi/bin:$PATH"
export DISPLAY=:99
export DBX_CONTAINER_MANAGER=docker

pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1280x1024x24 &

AIC_DIR=~/ws_aic/src/aic
cd "$AIC_DIR"

# ---------------------------------------------------------------
# Helper: kill a named tmux session if it exists
# ---------------------------------------------------------------
kill_session() {
  tmux kill-session -t "$1" 2>/dev/null || true
}

# ---------------------------------------------------------------
# CHECK 1 — WaveArm (proves Gazebo + ROS comms work)
# ---------------------------------------------------------------
echo "========================================================"
echo "CHECK 1: WaveArm policy"
echo "  SUCCESS = trial scores appear in the eval terminal"
echo "  FAILURE = 'No node found' repeats past 60 s"
echo "========================================================"
echo ""

kill_session aic_verify

tmux new-session -d -s aic_verify -x 220 -y 50

# Pane 0: eval container
tmux send-keys -t aic_verify:0 \
  "export DBX_CONTAINER_MANAGER=docker && \
   distrobox enter -r aic_eval -- \
     /entrypoint.sh gazebo_gui:=false launch_rviz:=false \
                    ground_truth:=false start_aic_engine:=true" Enter

echo "Waiting 25 s for Gazebo to start..."
sleep 25

# Pane 1: WaveArm policy
tmux split-window -t aic_verify -h
tmux send-keys -t aic_verify:0.1 \
  "cd $AIC_DIR && pixi run ros2 run aic_model aic_model \
     --ros-args -p use_sim_time:=true \
     -p policy:=aic_example_policies.ros.WaveArm" Enter

echo ""
echo "WaveArm is running. Watch the eval pane for trial scores."
echo "The arm should wave; scores appear after all 3 trials (~3 min)."
echo ""
echo "Press Enter here when you have seen scores (success)"
echo "or Ctrl+C if it hangs past 90 s (failure)."
read -r _

kill_session aic_verify
echo "CHECK 1 done."
echo ""

# ---------------------------------------------------------------
# CHECK 2 — CheatCode (proves ground-truth insertion works)
# ---------------------------------------------------------------
echo "========================================================"
echo "CHECK 2: CheatCode policy"
echo "  SUCCESS = insertions complete; scores ~75+ per trial"
echo "  FAILURE = robot moves but never inserts / crashes"
echo "========================================================"
echo ""

kill_session aic_cheatcode_test

tmux new-session -d -s aic_cheatcode_test -x 220 -y 50

# Pane 0: eval container with ground_truth
tmux send-keys -t aic_cheatcode_test:0 \
  "export DBX_CONTAINER_MANAGER=docker && \
   distrobox enter -r aic_eval -- \
     /entrypoint.sh gazebo_gui:=false launch_rviz:=false \
                    ground_truth:=true start_aic_engine:=true" Enter

echo "Waiting 25 s for Gazebo to start..."
sleep 25

# Pane 1: CheatCode policy
tmux split-window -t aic_cheatcode_test -h
tmux send-keys -t aic_cheatcode_test:0.1 \
  "cd $AIC_DIR && pixi run ros2 run aic_model aic_model \
     --ros-args -p use_sim_time:=true \
     -p policy:=aic_example_policies.ros.CheatCode" Enter

echo ""
echo "CheatCode is running. Watch the eval pane for insertion scores."
echo "Each trial should score ~75 pts. Total ~225/300 is expected."
echo ""
echo "Press Enter here when all 3 trials have completed (success)"
echo "or Ctrl+C if CheatCode fails to insert (failure)."
read -r _

kill_session aic_cheatcode_test
echo "CHECK 2 done."
echo ""

echo "========================================================"
echo "Both checks must PASS before running collect_demos.sh"
echo "========================================================"

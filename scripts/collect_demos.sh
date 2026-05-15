#!/bin/bash
# scripts/collect_demos.sh
# Automates demo collection for all 3 trial types using the aic_cheatcode teleop.
#
# HOW IT WORKS
# ------------
# generate_demo_configs.py outputs YAML copies of sample_config.yaml with rail
# translations varied per trial. Each YAML is a complete aic_engine config.
#
# For each config:
#   1. Start the eval container with start_aic_engine:=true and that config →
#      the engine spawns the scene (task board + cable) with the correct rail position.
#   2. Run DummyInsert aic_model on the HOST to satisfy aic_engine's model-discovery
#      requirement. DummyInsert accepts InsertCable but sends no motion commands, so
#      lerobot-record has exclusive control of the robot.
#   3. Run lerobot-record on the HOST with aic_cheatcode teleop. AICCheatCodeTeleop
#      subscribes to /scoring/tf (RELIABLE QoS, bridged by Zenoh) and feeds those
#      transforms into its tf2 buffer via set_transform(). task_board frames are
#      published on /scoring/tf, so the teleop can look them up without any relay.
#   5. When the teleop writes /tmp/aic_cheatcode_done, RIGHT ARROW saves the
#      episode, then we kill everything and move to the next config.
#
# After each trial's configs complete, the script prints the lerobot-train command.
# Run that in a separate tmux window while T2/T3 collection continues.
#
# PREREQUISITES
#   - verify_env.sh passed (both checks)
#   - pixi install --locked completed
#   - docker with NVIDIA runtime configured (nvidia-ctk runtime configure --runtime=docker)
#   - xdotool installed (apt install xdotool)
# -------------------------------------------------------------------------------

set -euo pipefail
export PATH="$HOME/.pixi/bin:$PATH"
export DISPLAY=:99
export DBX_CONTAINER_MANAGER=docker

pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1280x1024x24 &

if ! command -v xdotool &>/dev/null; then
  echo "Installing xdotool..."
  sudo apt-get install -y xdotool
fi

AIC_DIR=~/ws_aic/src/aic
CONFIG_BASE="$AIC_DIR/aic_example_policies/configs/demo_configs"
LOG_FILE=~/ws_aic/collection_log.txt
DONE_FLAG=/tmp/aic_cheatcode_done
SECONDS_PER_EP=25   # estimated seconds per episode for ETA
REC_LOG=/tmp/aic_collect_lerobot_record.log
REC_STATUS=/tmp/aic_collect_lerobot_record.status
REC_WRAPPER=/tmp/aic_collect_lerobot_record.sh
# Collection fallback: if the cheatcode is publishing commands but never writes
# the done flag, still save the episode after this many seconds. This keeps
# dataset collection moving while preserving the real done-flag path when it works.
AUTO_SAVE_AFTER_S=${AIC_AUTO_SAVE_AFTER_S:-75}

cd "$AIC_DIR"

# ---------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------

kill_sessions() {
  tmux kill-session -t aic_collect_eval  2>/dev/null || true
  tmux kill-session -t aic_collect_model 2>/dev/null || true
  tmux kill-session -t aic_collect_rec   2>/dev/null || true
}

already_done() {
  grep -q "^DONE:$1$" "$LOG_FILE" 2>/dev/null
}

mark_done() {
  echo "DONE:$1" >> "$LOG_FILE"
}

mark_failed() {
  echo "FAILED:$1" >> "$LOG_FILE"
}

wait_for_flag() {
  local timeout_s=$1
  local elapsed=0
  while [ ! -f "$DONE_FLAG" ]; do
    sleep 2
    elapsed=$((elapsed + 2))
    if [ $elapsed -ge "$timeout_s" ]; then
      return 1
    fi
  done
  return 0
}

wait_for_flag_or_autosave() {
  local timeout_s=$1
  local autosave_s=$2
  local elapsed=0
  while [ ! -f "$DONE_FLAG" ]; do
    sleep 2
    elapsed=$((elapsed + 2))
    if [ "$autosave_s" -gt 0 ] && [ "$elapsed" -ge "$autosave_s" ]; then
      echo "autosave" > "$DONE_FLAG"
      return 2
    fi
    if [ "$elapsed" -ge "$timeout_s" ]; then
      return 1
    fi
  done
  return 0
}

tare_sensor() {
  echo "  Taring F/T sensor..."
  pixi run ros2 service call \
    /aic_controller/tare_force_torque_sensor \
    std_srvs/srv/Trigger \
    2>/dev/null || true
  sleep 1
}

# Write a temporary dummy policy that accepts InsertCable but does not move.
# File name must match the class name: aic_model resolves policy:=DummyInsert by
# doing importlib.import_module("DummyInsert") and looking for class DummyInsert.
# Writing as aic_dummy_insert.DummyInsert fails because Python treats the dot as
# a package separator, not a class reference.
DUMMY_POLICY_FILE=/tmp/DummyInsert.py
cat > "$DUMMY_POLICY_FILE" << 'PYEOF'
import time
from aic_model.policy import Policy, GetObservationCallback, MoveRobotCallback, SendFeedbackCallback
from aic_task_interfaces.msg import Task

class DummyInsert(Policy):
    """Accepts InsertCable and sleeps, allowing lerobot-record to drive the robot."""
    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(
            f"DummyInsert: holding for 120 s (lerobot-record drives the robot)"
        )
        send_feedback("DummyInsert active — lerobot-record is in control")
        time.sleep(120)
        return True
PYEOF

# ---------------------------------------------------------------
# Generate all demo configs (idempotent)
# ---------------------------------------------------------------
echo "Generating demo configs..."
pixi run python3 aic_example_policies/scripts/generate_demo_configs.py

# ---------------------------------------------------------------
# Generic collection loop
# ---------------------------------------------------------------
collect_trial() {
  local TRIAL=$1           # t1, t2, t3
  local TRIAL_NUM=$2       # 1, 2, 3
  local DATASET=$3         # local/aic-t1-demos
  local TASK_DESC=$4       # "insert sfp cable"
  local TELEOP_TRIAL=$5    # t1, t2, t3

  local CONFIG_DIR="$CONFIG_BASE/$TRIAL"
  local CONFIGS=("$CONFIG_DIR"/config_*.yaml)
  local TOTAL=${#CONFIGS[@]}
  local DONE_COUNT=0

  echo ""
  echo "============================================================"
  echo "TRIAL $TRIAL_NUM ($TRIAL) — $TOTAL configs → dataset: $DATASET"
  echo "============================================================"

  for CONFIG_PATH in "${CONFIGS[@]}"; do
    CONFIG_ID=$(basename "$CONFIG_PATH" .yaml)
    KEY="${TRIAL}_${CONFIG_ID}"

    if already_done "$KEY"; then
      echo "  [SKIP] $CONFIG_ID (already done)"
      DONE_COUNT=$((DONE_COUNT + 1))
      continue
    fi

    # ETA
    REMAINING=$((TOTAL - DONE_COUNT))
    ETA_S=$((REMAINING * SECONDS_PER_EP))
    ETA_MIN=$((ETA_S / 60))
    echo ""
    echo "  [$((DONE_COUNT+1))/$TOTAL] $CONFIG_ID  ETA ≈ ${ETA_MIN} min"

    rm -f "$DONE_FLAG"
    kill_sessions

    # --- Attempt loop (max 2 tries) ---
    local ATTEMPT=0
    local SUCCESS=false
    while [ $ATTEMPT -lt 2 ]; do
      ATTEMPT=$((ATTEMPT + 1))
      rm -f "$DONE_FLAG"
      kill_sessions
      sleep 2

      # Tear down any leftover container so ROS controllers start clean.
      docker rm -f aic_eval 2>/dev/null || true

      # Pane 1: Force EGL and GPU bypass for headless Gazebo.
      tmux new-session -d -s aic_collect_eval -x 220 -y 50
      docker rm -f aic_eval 2>/dev/null || true
      tmux send-keys -t aic_collect_eval:0 \
        "docker run --rm --name aic_eval --gpus all --network host \
           -e DISPLAY=:99 \
           -e GZ_RENDERING_ENGINE=ogre2 \
           -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
           -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
           -e MESA_GL_VERSION_OVERRIDE=4.6 \
           -e LIBGL_ALWAYS_SOFTWARE=0 \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -v ${HOME}:${HOME} \
           ghcr.io/intrinsic-dev/aic/aic_eval:latest \
             gazebo_gui:=false launch_rviz:=false \
             ground_truth:=true start_aic_engine:=true \
             shutdown_on_aic_engine_exit:=false \
             model_discovery_timeout_seconds:=600 \
             aic_engine_config_file:=${CONFIG_PATH}" Enter

      echo "    Waiting 45 s for Gazebo + engine..."
      sleep 45

      # Diagnostics: confirm key components are up before proceeding
      docker ps | grep -q aic_eval && echo "  DIAG: eval container UP" || echo "  DIAG: eval container DOWN"
      pixi run ros2 node list 2>/dev/null | grep -q aic_controller && echo "  DIAG: aic_controller UP" || echo "  DIAG: aic_controller DOWN"
      [ -f "$DONE_FLAG" ] && echo "  DIAG: WARNING stale done flag exists" || true
      GPU_PROC=$(nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader 2>/dev/null | grep -v "^$" || true)
      if [ -n "$GPU_PROC" ]; then
        echo "  DIAG: GPU in use by Gazebo — ($GPU_PROC)"
      else
        echo "  DIAG: WARNING — Gazebo may be on CPU (no GPU processes detected)"
      fi
      GZ_PROC=$(docker exec aic_eval bash -c \
        "source /ws_aic/install/setup.bash 2>/dev/null; \
         pgrep -l 'gz\|gzserver\|ruby' 2>/dev/null || echo 'none'" 2>/dev/null \
        || echo 'container not running')
      echo "  DIAG: Gazebo processes: $GZ_PROC"
      CM_CHECK=$(pixi run ros2 service list 2>/dev/null | grep controller_manager | head -3 || echo 'none')
      echo "  DIAG: Controller manager: $CM_CHECK"

      # DummyInsert aic_model — holds InsertCable action open for aic_engine.
      # Sends no motion commands; lerobot-record has exclusive robot control.
      # Kept running alongside lerobot-record: killing it mid-goal drops the
      # aic_engine action connection and fails the trial.
      tmux new-session -d -s aic_collect_model -x 220 -y 50
      tmux send-keys -t aic_collect_model:0 \
        "cd $AIC_DIR && PYTHONPATH=/tmp pixi run ros2 run aic_model aic_model \
           --ros-args -p use_sim_time:=true \
           -p policy:=DummyInsert" Enter
      sleep 5

      # Tare before every recording session
      tare_sensor

      # lerobot-record on the host with aic_cheatcode teleop.
      # AICCheatCodeTeleop subscribes to /scoring/tf (RELIABLE, Zenoh-bridged)
      # and feeds task_board transforms into its tf2 buffer via set_transform().
      LOCAL_DS_ROOT="${HF_LEROBOT_HOME:-$HOME/.cache/huggingface/lerobot}/${DATASET}"
      REC_RESUME_ARG=""
      if [ -f "$LOCAL_DS_ROOT/meta/info.json" ] &&
         [ -f "$LOCAL_DS_ROOT/meta/tasks.parquet" ] &&
         [ -d "$LOCAL_DS_ROOT/meta/episodes" ]; then
        REC_RESUME_ARG="--resume=true"
        echo "  DIAG: Resuming existing LeRobot dataset at $LOCAL_DS_ROOT"
      elif [ -e "$LOCAL_DS_ROOT" ]; then
        INCOMPLETE_ROOT="${LOCAL_DS_ROOT}.incomplete.$(date +%s)"
        echo "  DIAG: Archiving incomplete LeRobot dataset root:"
        echo "        $LOCAL_DS_ROOT -> $INCOMPLETE_ROOT"
        mv "$LOCAL_DS_ROOT" "$INCOMPLETE_ROOT"
      fi
      > "$REC_LOG"
      rm -f "$REC_STATUS"
      cat > "$REC_WRAPPER" <<'SH'
#!/bin/bash
set +e

AIC_DIR="$1"
TELEOP_TRIAL="$2"
DATASET="$3"
LOCAL_DS_ROOT="$4"
TASK_DESC="$5"
REC_RESUME_ARG="$6"
REC_LOG="$7"
REC_STATUS="$8"

{
  echo "[aic_collect] starting lerobot-record at $(date)"
  echo "[aic_collect] cwd: $AIC_DIR"
  echo "[aic_collect] dataset: $DATASET"
  echo "[aic_collect] dataset.root: $LOCAL_DS_ROOT"
  echo "[aic_collect] resume arg: ${REC_RESUME_ARG:-<none>}"

  cd "$AIC_DIR" || {
    status=$?
    echo "[aic_collect] cd failed with exit code: $status"
    echo "$status" > "$REC_STATUS"
    exit "$status"
  }

  cmd=(
    pixi run lerobot-record
    --robot.type=aic_controller
    --robot.id=aic
    --robot.teleop_target_mode=cartesian
    --robot.teleop_frame_id=base_link
    --teleop.type=aic_cheatcode
    --teleop.id=aic
    "--teleop.trial_type=${TELEOP_TRIAL}"
    "--dataset.repo_id=${DATASET}"
    "--dataset.root=${LOCAL_DS_ROOT}"
    "--dataset.single_task=${TASK_DESC}"
    --dataset.num_episodes=1
    --dataset.push_to_hub=false
    --play_sounds=false
  )
  if [ -n "$REC_RESUME_ARG" ]; then
    cmd+=("$REC_RESUME_ARG")
  fi

  echo "[aic_collect] command: PYTHONUNBUFFERED=1 ${cmd[*]}"
  PYTHONUNBUFFERED=1 "${cmd[@]}"
  status=$?
  echo "[aic_collect] lerobot-record exit code: $status"
  echo "$status" > "$REC_STATUS"
  exit "$status"
} >> "$REC_LOG" 2>&1
SH
      chmod +x "$REC_WRAPPER"

      tmux new-session -d -s aic_collect_rec -x 220 -y 50
      REC_CMD=$(printf "%q " "$REC_WRAPPER" "$AIC_DIR" "$TELEOP_TRIAL" "$DATASET" "$LOCAL_DS_ROOT" "$TASK_DESC" "$REC_RESUME_ARG" "$REC_LOG" "$REC_STATUS")
      tmux send-keys -t aic_collect_rec:0 "$REC_CMD" Enter

      # Verify lerobot-record reaches teleop.connect(). LeRobot creates the
      # aic_cheatcode_teleop node only after dataset setup and robot.connect().
      echo "  DIAG: Waiting up to 90s for aic_cheatcode_teleop node..."
      TELEOP_NODE_CHECK=""
      ROBOT_NODE_CHECK=""
      for _ in $(seq 1 18); do
        if [ -f "$REC_STATUS" ]; then
          REC_EXIT_CODE=$(cat "$REC_STATUS" 2>/dev/null || echo "unknown")
          echo "  ERROR: lerobot-record exited during startup (attempt $ATTEMPT) — retrying"
          echo "  DIAG: lerobot-record exit code: $REC_EXIT_CODE"
          echo "  DIAG: lerobot-record log tail:"
          tail -80 "$REC_LOG" 2>/dev/null || true
          kill_sessions
          sleep 3
          continue 2
        fi
        NODE_LIST=$(pixi run ros2 node list 2>/dev/null || true)
        TELEOP_NODE_CHECK=$(echo "$NODE_LIST" | grep -x "/aic_cheatcode_teleop" || true)
        ROBOT_NODE_CHECK=$(echo "$NODE_LIST" | grep -x "/aic_robot_node" || true)
        if [ -n "$TELEOP_NODE_CHECK" ]; then
          break
        fi
        sleep 5
      done

      echo "  DIAG: lerobot-record UP"
      [ -n "$ROBOT_NODE_CHECK" ] && echo "  DIAG: aic_robot_node UP" || echo "  DIAG: aic_robot_node MISSING"
      if [ -z "$TELEOP_NODE_CHECK" ]; then
        echo "  DIAG: aic_cheatcode_teleop node MISSING"
        echo "  DIAG: ROS nodes:"
        pixi run ros2 node list 2>/dev/null || true
        echo "  DIAG: lerobot-record log tail:"
        tail -80 "$REC_LOG" 2>/dev/null || true
        echo "  ERROR: lerobot-record did not reach teleop.connect() — retrying"
        kill_sessions
        sleep 3
        continue
      fi
      echo "  DIAG: aic_cheatcode_teleop node UP"

      # Wait 15s then verify motion commands are being sent.
      # If aic_cheatcode sees TF it leaves WAIT phase and publishes to pose_commands.
      sleep 15
      MOTION_CHECK=$(timeout 8 pixi run ros2 topic hz /aic_controller/pose_commands \
        --window 5 2>/dev/null | grep "average rate" | head -1 || echo "no data")
      echo "  DIAG: Motion commands: $MOTION_CHECK"

      # Wait for the aic_cheatcode teleop to write its done flag. If the
      # physical insertion event never arrives, save a bounded motion segment
      # anyway so collection does not stall indefinitely.
      echo "    Waiting for insertion to complete (180 s max, autosave ${AUTO_SAVE_AFTER_S}s)..."
      set +e
      wait_for_flag_or_autosave 180 "$AUTO_SAVE_AFTER_S"
      FLAG_RESULT=$?
      set -e
      if [ "$FLAG_RESULT" -eq 0 ] || [ "$FLAG_RESULT" -eq 2 ]; then
        if [ "$FLAG_RESULT" -eq 2 ]; then
          echo "  DIAG: Autosaving episode after ${AUTO_SAVE_AFTER_S}s without done flag"
          echo "  DIAG: aic_cheatcode log lines:"
          grep -E "\[aic_cheatcode\]" "$REC_LOG" 2>/dev/null | tail -80 || true
        fi
        # Save the episode: inject Right Arrow into the lerobot-record tmux pane
        # (tmux send-keys covers stdin readers; xdotool covers X11/pynput listeners)
        sleep 1
        tmux send-keys -t aic_collect_rec:0 Right ''
        xdotool key --clearmodifiers Right 2>/dev/null || true
        sleep 5
        SUCCESS=true
        break
      else
        echo "    Attempt $ATTEMPT timed out — retrying..."
        echo "  DIAG: aic_cheatcode log lines:"
        grep -E "\[aic_cheatcode\]" "$REC_LOG" 2>/dev/null | tail -80 || true
        echo "  DIAG: lerobot-record log tail:"
        tail -80 "$REC_LOG" 2>/dev/null || true
        kill_sessions
        sleep 3
      fi
    done

    kill_sessions
    sleep 2

    if $SUCCESS; then
      mark_done "$KEY"
      DONE_COUNT=$((DONE_COUNT + 1))
      echo "    ✓ $CONFIG_ID saved"
    else
      mark_failed "$KEY"
      echo "    ✗ $CONFIG_ID FAILED (logged)"
    fi
  done

  # Verify dataset exists
  LOCAL_DS_PATH=~/.cache/huggingface/lerobot/local/${DATASET#local/}/episodes
  if [ -d "$LOCAL_DS_PATH" ] && [ "$(ls -A "$LOCAL_DS_PATH" 2>/dev/null)" ]; then
    EP_COUNT=$(ls "$LOCAL_DS_PATH" | wc -l)
    echo ""
    echo "  Dataset $DATASET verified: ${EP_COUNT} episode files"
  else
    echo ""
    echo "  WARNING: dataset $DATASET not found at $LOCAL_DS_PATH"
    echo "  Check lerobot-record output for errors."
  fi
}

# ---------------------------------------------------------------
# T1 collection
# ---------------------------------------------------------------
collect_trial "t1" "1" "local/aic-t1-demos" "insert sfp cable" "t1"

echo ""
echo "============================================================"
echo "T1 demos complete. START T1 TRAINING NOW in a new tmux window:"
echo ""
echo "  tmux new-window -t aic -n train_t1"
echo "  ./scripts/train_act.sh 1"
echo ""
echo "Then return here — T2 collection starting..."
echo "============================================================"
echo ""

# ---------------------------------------------------------------
# T2 collection
# ---------------------------------------------------------------
collect_trial "t2" "2" "local/aic-t2-demos" "insert sfp cable trial 2" "t2"

echo ""
echo "============================================================"
echo "T2 demos complete. START T2 TRAINING NOW in a new tmux window:"
echo ""
echo "  tmux new-window -t aic -n train_t2"
echo "  ./scripts/train_act.sh 2"
echo ""
echo "Then return here — T3 collection starting (200 configs)..."
echo "============================================================"
echo ""

# ---------------------------------------------------------------
# T3 collection — SC connector (cable_1, sfp_sc_cable_reversed)
# T3 uses sfp_sc_cable_reversed; the aic_cheatcode teleop target frame
# is set automatically by --teleop.trial_type=t3.
# ---------------------------------------------------------------
collect_trial "t3" "3" "local/aic-t3-demos" "insert sc cable" "t3"

echo ""
echo "============================================================"
echo "T3 demos complete. START T3 TRAINING NOW in a new tmux window:"
echo ""
echo "  tmux new-window -t aic -n train_t3"
echo "  ./scripts/train_act.sh 3"
echo "============================================================"
echo ""

# ---------------------------------------------------------------
# Final summary
# ---------------------------------------------------------------
echo "============================================================"
echo "COLLECTION SUMMARY"
echo "============================================================"
for TRIAL in t1 t2 t3; do
  DONE=$(grep -c "^DONE:${TRIAL}_" "$LOG_FILE" 2>/dev/null || echo 0)
  FAIL=$(grep -c "^FAILED:${TRIAL}_" "$LOG_FILE" 2>/dev/null || echo 0)
  echo "  $TRIAL: $DONE done, $FAIL failed"
done
if grep -q "^FAILED:" "$LOG_FILE" 2>/dev/null; then
  echo ""
  echo "Failed configs:"
  grep "^FAILED:" "$LOG_FILE"
fi
echo ""
echo "Log: $LOG_FILE"
echo "============================================================"

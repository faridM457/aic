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
#      /tmp is volume-mounted (done flag sharing) and ~/.cache/huggingface is
#      volume-mounted so datasets land on the host filesystem.
#   2. Run lerobot-record INSIDE the container via docker exec so the aic_cheatcode
#      teleop has direct access to ground-truth TF frames. TF is not bridged to the
#      host via zenoh, so running on the host causes the teleop to hang in WAIT phase.
#   3. When the teleop writes /tmp/aic_cheatcode_done (visible on host via /tmp mount),
#      the script presses RIGHT ARROW to save the episode, kills everything, and
#      moves to the next config.
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

tare_sensor() {
  echo "  Taring F/T sensor..."
  pixi run ros2 service call \
    /aic_controller/tare_force_torque_sensor \
    std_srvs/srv/Trigger \
    2>/dev/null || true
  sleep 1
}

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
      docker kill aic_eval 2>/dev/null || true
      docker rm   aic_eval 2>/dev/null || true

      # Pane 1: Force EGL and GPU bypass for headless Gazebo.
      tmux new-session -d -s aic_collect_eval -x 220 -y 50
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
           -v /tmp:/tmp \
           -v ~/.cache/huggingface:/root/.cache/huggingface \
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

      # Check if trial-specific TF frames are visible from host
      case "$TELEOP_TRIAL" in
        t1) TF_TARGET="task_board/nic_card_mount_0/sfp_port_0_link" ;;
        t2) TF_TARGET="task_board/nic_card_mount_1/sfp_port_0_link" ;;
        t3) TF_TARGET="task_board/sc_port_1/sc_port_base_link" ;;
        *)  TF_TARGET="task_board/nic_card_mount_0/sfp_port_0_link" ;;
      esac
      TF_CHECK=$(timeout 5 pixi run ros2 run tf2_ros tf2_echo base_link \
        "$TF_TARGET" 2>&1 | head -5 || true)
      # TF frames are published inside the container and are NOT bridged to the host
      # via zenoh — failure here is expected. lerobot-record runs inside the container
      # (Pane 3 below) where TF is directly visible.
      echo "  DIAG: TF frame visibility from host (expected FAIL): $TF_CHECK"

      # Tare before every recording session
      tare_sensor

      # Pane 3: lerobot-record runs INSIDE the container via docker exec so the
      # aic_cheatcode teleop has direct access to ground-truth TF frames.
      # LEROBOT_TASK is passed as an env var to avoid quoting issues with spaces.
      # The done flag (/tmp/aic_cheatcode_done) and dataset cache are shared with
      # the host via volume mounts on the docker run command above.
      tmux new-session -d -s aic_collect_rec -x 220 -y 50
      tmux send-keys -t aic_collect_rec:0 \
        "docker exec -e LEROBOT_TASK='${TASK_DESC}' aic_eval bash -c \
          'export PATH=/home/ubuntu/.pixi/bin:\$PATH && \
           source /opt/ros/kilted/setup.bash && \
           source /ws_aic/install/setup.bash && \
           cd /home/ubuntu/ws_aic/src/aic && \
           lerobot-record \
             --robot.type=aic_controller \
             --robot.id=aic \
             --robot.teleop_target_mode=cartesian \
             --robot.teleop_frame_id=base_link \
             --teleop.type=aic_cheatcode \
             --teleop.id=aic \
             --teleop.trial_type=${TELEOP_TRIAL} \
             --dataset.repo_id=${DATASET} \
             --dataset.single_task=\"\$LEROBOT_TASK\" \
             --dataset.num_episodes=1 \
             --dataset.push_to_hub=false \
             --play_sounds=false'" Enter

      # Verify lerobot-record started inside the container — give it 10s to initialize
      sleep 10
      if ! docker exec aic_eval pgrep -f lerobot-record >/dev/null 2>&1; then
        echo "  ERROR: lerobot-record failed to start inside container (attempt $ATTEMPT) — retrying"
        kill_sessions
        sleep 3
        continue
      fi
      echo "  DIAG: lerobot-record UP (inside container)"

      # Wait for the aic_cheatcode teleop to write its done flag (180 s max)
      echo "    Waiting for insertion to complete (180 s max)..."
      if wait_for_flag 180; then
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

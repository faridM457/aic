# AIC Demo Collection — Handover

## Project Context

**AIC (AI for Industry Challenge)** — insert fiber optic cables into a randomised
task board using a UR5e robot + Robotiq Hand-E gripper + ATI AXIA80-M20 F/T sensor
+ 3 Basler cameras.  Three trial types:

| Trial | Connector | Dataset |
|-------|-----------|---------|
| T1 | SFP (nic_card_mount_0) | `local/aic-t1-demos` |
| T2 | SFP (nic_card_mount_1) | `local/aic-t2-demos` |
| T3 | SC plug | `local/aic-t3-demos` |

**Infra**: EC2 `g5.2xlarge` (NVIDIA A10G, 24 GB VRAM), Ubuntu 24.04.  
**Repo**: `https://github.com/faridM457/aic` — all scripts are in `scripts/`.  
**Env manager**: pixi (`~/.pixi/bin/pixi`).  
**Eval container**: `ghcr.io/intrinsic-dev/aic/aic_eval:latest` — launched with
`docker run --gpus all --network host`.

---

## What Works (Confirmed on EC2)

- `docker run --gpus all --network host` starts the eval container correctly.
- Gazebo starts and uses the GPU (≈841 MiB VRAM, 22 % utilisation) when the full
  NVIDIA EGL env-var stack is set (see `docker run` command in `collect_demos.sh`).
- All ROS controllers load: `fts_broadcaster`, `joint_state_broadcaster`,
  `aic_controller`.
- F/T sensor tares successfully.
- lerobot-record starts on the host (`pixi run lerobot-record …`).
- `/scoring/tf` topic IS bridged from container to host via zenoh.
- ROS service topics (controller_manager, etc.) ARE bridged.

---

## What Doesn't Work / Root Causes

### TF frame visibility

`/tf_static` uses `TRANSIENT_LOCAL` QoS and is not reliable across the
container/host Zenoh boundary.  The eval container also publishes ground-truth
task-board frames on `/scoring/tf`, which is host-visible.  The
`aic_cheatcode` teleop subscribes to `/scoring/tf` and feeds those transforms
into its local tf2 buffer with `set_transform()`, so it can resolve
`base_link -> task_board/...` without a relay.

---

## Current Architecture (4 panes per episode)

```
Pane 1  docker run aic_eval        Gazebo + aic_engine (inside container)
Pane 2  pixi run aic_model DummyInsert     holds InsertCable action open (host)
Pane 3  pixi run lerobot-record            aic_cheatcode teleop drives + records (host)
```

**Why DummyInsert?** `aic_engine` sends an `InsertCable` ROS action goal and
waits for an `aic_model` node to accept it.  Without an acceptor it times out
and exits (even though Gazebo keeps running due to
`shutdown_on_aic_engine_exit:=false`).  `DummyInsert` satisfies the discovery
requirement without publishing any motion commands, so `lerobot-record`/
`aic_cheatcode` has exclusive control of `/aic_controller/pose_commands`.

**Why not CheatCode as aic_model?**  `CheatCode.py` also calls
`lookup_transform` for the same task_board frames via its own tf2_buffer.  On
the host it has the same TF visibility problem.  Additionally, CheatCode
publishes POSE TARGETS while `aic_cheatcode` teleop (via lerobot-record)
publishes VELOCITY commands — both to `/aic_controller/pose_commands` — causing
a motion conflict.  lerobot-record also has no passive/observer mode; it
requires a teleop to generate the action side of `(observation, action)` pairs.

---

## Key Files

| File | Purpose |
|------|---------|
| `scripts/collect_demos.sh` | Main collection script — edit here |
| `scripts/verify_env.sh` | Pre-flight checks (WaveArm + CheatCode) |
| `scripts/smoke_test.sh` | Quick ROS communication check |
| `scripts/aws_setup.sh` | One-shot EC2 setup |
| `scripts/check_gpu.sh` | GPU passthrough diagnostic |
| `scripts/train_act.sh` | Training launcher |
| `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_teleop.py` | aic_cheatcode teleop |
| `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_robot_aic_controller.py` | lerobot robot adapter |
| `aic_example_policies/aic_example_policies/ros/CheatCode.py` | CheatCode aic_model policy |
| `aic_model/aic_model/aic_model.py` | aic_model node (loads any Policy subclass) |
| `aic_example_policies/configs/demo_configs/` | Generated per-config YAMLs (gitignored) |

---

## collect_demos.sh — Sequence of Events

```
t+0s    docker rm -f aic_eval (cleanup)
t+0s    docker run aic_eval (Pane 1) — Gazebo + aic_engine start
t+45s   Diagnostics: container UP, aic_controller UP, GPU check, Gazebo procs
t+45s   pixi run aic_model DummyInsert (Pane 2) — sleep 5
t+50s   tare_sensor
t+50s   pixi run lerobot-record (Pane 3)
t+60s   pgrep check — lerobot-record startup verified
t+60s   node check — /aic_cheatcode_teleop should be visible
t+75s   sleep 15 → motion diagnostic (ros2 topic hz /aic_controller/pose_commands)
t+110s  wait_for_flag 180s — waits for /tmp/aic_cheatcode_done
        → done: tmux send-keys Right + xdotool key Right → episode saved
```

---

## What Has NOT Been Tested Yet

The first EC2 run after pulling should tell you:

1. Does `/aic_cheatcode_teleop` appear after `lerobot-record` starts?
2. Does `aic_cheatcode` leave WAIT phase and show APPROACH in lerobot-record output?
3. Does `/aic_controller/pose_commands` show `average rate: X.X` > 0?
4. Does `/tmp/aic_cheatcode_done` get written and the episode save?

---

## If the Relay Still Doesn't Work

### Option A — Check `/scoring/tf`
On the host:
```bash
pixi run ros2 topic hz /scoring/tf
pixi run ros2 topic echo /scoring/tf --once | grep child_frame_id
```

Inside the container:
```bash
source /ws_aic/install/setup.bash
ros2 topic echo /scoring/tf --once 2>/dev/null | grep child_frame_id
```
If task_board frames appear in the container but not on the host, the issue is
Zenoh bridging.  If they do not appear in the container, the ground_truth plugin
or scene spawn is not ready yet.

### Option B — Static republish from config
If `/scoring/tf` is unavailable, parse the config YAML for the rail position, compute the
port pose analytically, and use `ros2 run tf2_ros static_transform_publisher`
on the host to inject the frame directly.  This bypasses zenoh entirely.

---

## Recent Commit History

```
2f768bb Fix demo collection: tf_static relay bridges TF frames from container to host
909aa1f Fix TF bridging: set RMW_ZENOH_CONFIG_FILE on container and host  (didn't work)
342bef6 Fix container name conflict with docker rm -f before docker run
1defb20 Run lerobot-record inside container via docker exec; remove DummyInsert  (reverted)
0076fa4 Force NVIDIA EGL for Gazebo headless GPU rendering
9028f03 Fix GPU passthrough for Gazebo: NVIDIA env vars, render group, Mesa override
```

---

## Running the Collection

```bash
# On EC2 — assumes aws_setup.sh was run and pixi install --locked completed
cd ~/ws_aic/src/aic
./scripts/verify_env.sh       # both checks must pass
./scripts/collect_demos.sh    # runs all 3 trials (~hours)
```

Training launches as each trial completes (prompted by the script):
```bash
./scripts/train_act.sh 1   # T1
./scripts/train_act.sh 2   # T2
./scripts/train_act.sh 3   # T3
```

LeRobot 0.5.1 writes deployable policy files under:
```bash
outputs/act_trialN/checkpoints/last/pretrained_model/
```
That directory is what `copy_checkpoints.sh` pulls from EC2 and what
`build_and_submit.sh` stages into `aic_example_policies/checkpoints/{sfp,sc}`.

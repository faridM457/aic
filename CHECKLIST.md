# AIC Qualification Checklist

Qualification deadline: **May 15, 2025**  
Scoring: 300 pts max (3 trials × 100). Top-30 cutoff ≈ 110/300.  
Tier breakdown per trial: Validity (1pt) + Smoothness/Efficiency (24pt max) + Correct insertion (75pt) or partial (up to 50pt).

---

## Phase 0 — AWS Setup

- [ ] Launch EC2 g5.2xlarge (Ubuntu 24.04, A10G 24 GB VRAM)
- [ ] Copy PEM key locally: `export AIC_KEY_PATH=~/aic-key.pem`
- [ ] `ssh -i $AIC_KEY_PATH ubuntu@<ec2-dns>`
- [ ] Run on EC2: `./scripts/aws_setup.sh`
- [ ] Verify on EC2: `newgrp docker` (or re-login to pick up docker group)
- [ ] All 5 checks in aws_setup.sh PASS (nvidia-smi, GPU passthrough, distrobox, pixi, python3)

---

## Phase 1 — Environment Verification

- [ ] Start Xvfb: `pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1280x1024x24 &`
- [ ] **Check 1 — WaveArm**: `./scripts/verify_env.sh`
  - Success: trial scores appear in eval terminal within ~3 min
- [ ] **Check 2 — CheatCode**: (same script, runs after Check 1)
  - Success: ~225/300 pts (≈75/trial), insertions complete
- [ ] Both checks PASS before proceeding

---

## Phase 2 — Demo Collection

- [ ] `scripts/update_and_run.sh` ran — configs verified (150/150/200)
- [ ] Config counts confirmed correct before starting collection
- [ ] Generate demo configs: `python3 aic_example_policies/scripts/generate_demo_configs.py`
  - Verify: `ls aic_example_policies/configs/demo_configs/t1/ | wc -l` → 150
  - Verify: `ls aic_example_policies/configs/demo_configs/t2/ | wc -l` → 150
  - Verify: `ls aic_example_policies/configs/demo_configs/t3/ | wc -l` → 200
- [ ] Start collection: `./scripts/collect_demos.sh`  (runs in main tmux window)

### T1 — SFP cable, nic_mount_0, 150 configs
- [ ] T1 collection complete (~150 episodes in `local/aic-t1-demos`)
- [ ] Verify dataset: `~/.cache/huggingface/lerobot/local/aic-t1-demos/episodes/`
- [ ] **Start T1 training immediately** (don't wait for T2/T3):
  ```
  tmux new-window -t aic -n train_t1
  ./scripts/train_act.sh 1
  ```

### T2 — SFP cable, nic_mount_1, 150 configs
- [ ] T2 collection complete (~150 episodes in `local/aic-t2-demos`)
- [ ] Verify dataset: `~/.cache/huggingface/lerobot/local/aic-t2-demos/episodes/`
- [ ] **Start T2 training immediately**:
  ```
  tmux new-window -t aic -n train_t2
  ./scripts/train_act.sh 2
  ```

### T3 — SC cable, sc_rail_1, cable_1, 200 configs
- [ ] T3 collection complete (~200 episodes in `local/aic-t3-demos`)
- [ ] Verify dataset: `~/.cache/huggingface/lerobot/local/aic-t3-demos/episodes/`
- [ ] **Start T3 training immediately**:
  ```
  tmux new-window -t aic -n train_t3
  ./scripts/train_act.sh 3
  ```

### Collection failure recovery
- Log file: `~/ws_aic/collection_log.txt`
- Re-run `collect_demos.sh` — already-done configs are skipped automatically
- Failed configs listed at end of summary; re-run until all are DONE

---

## Phase 3 — Training (parallel on EC2)

Each trial trains ~2 hours on A10G. Trials can overlap with collection.

| Trial | Dataset | Output dir | Duration |
|-------|---------|------------|----------|
| T1 | local/aic-t1-demos | outputs/act_trial1/ | ~2 hr |
| T2 | local/aic-t2-demos | outputs/act_trial2/ | ~2 hr |
| T3 | local/aic-t3-demos | outputs/act_trial3/ | ~2 hr |

- [ ] T1 training complete — `outputs/act_trial1/checkpoints/best/` exists
- [ ] T2 training complete — `outputs/act_trial2/checkpoints/best/` exists
- [ ] T3 training complete — `outputs/act_trial3/checkpoints/best/` exists

If CUDA OOM: `Ctrl+C` then re-run with `training.batch_size=4` appended to the `pixi run lerobot-train ...` command.

---

## Phase 4 — Copy Checkpoints to Local Machine

Run on your **local laptop**:
```
./scripts/copy_checkpoints.sh <ec2-public-dns>
```

- [ ] `outputs/act_trial1/checkpoints/best/` present locally
- [ ] `outputs/act_trial2/checkpoints/best/` present locally
- [ ] `outputs/act_trial3/checkpoints/best/` present locally

---

## Phase 5 — Build & Submit Docker Image

Run on **local laptop**. Must have AWS CLI configured and Docker running.

```bash
export AIC_TEAM_NAME=<your-team-name>
./scripts/build_and_submit.sh v1
```

- [ ] `AIC_TEAM_NAME` env var set
- [ ] Checkpoints staged: `aic_example_policies/checkpoints/sfp/` and `sc/`
- [ ] Docker image builds successfully (`my-solution:v1`)
- [ ] Smoke test passes (policy imports OK)
- [ ] ECR auth succeeds (`aws ecr get-login-password ...`)
- [ ] Image pushed: `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/<team>/v1`
- [ ] Portal submission: paste full image URI at participant portal

**⚠ ECR tags are IMMUTABLE. Use a new tag (v2, v3...) for every push.**

---

## Phase 6 — Evaluate & Iterate

- [ ] Submission received by portal — status changes to "Evaluating"
- [ ] Evaluation completes — check score breakdown:
  - T1 (SFP, nic_mount_0): target ≥ 75/100
  - T2 (SFP, nic_mount_1): target ≥ 75/100
  - T3 (SC plug, sc_rail_1): target ≥ 75/100
  - Total target: ≥ 225/300 (well above top-30 cutoff of ≈110)

### If something goes wrong

| Problem | Fix |
|---------|-----|
| Config count wrong after generate_demo_configs.py | Re-run script, check for Python errors |
| CheatCode fails to insert | Verify ground_truth:=true, tare sensor, restart Gazebo |
| CUDA OOM during training | Add `policy.batch_size=4` to train command; all 3 trials can train in parallel on A10G if VRAM allows |
| lerobot-record crashes | Check pixi env, verify aic_cheatcode teleop registered, check DISPLAY=:99 |

### If score is low — debugging checklist
- [ ] Check `CableInsertion.py` logs for model load errors (`ACT_MODEL_PATH_SFP/SC`)
- [ ] Verify correct cable/connector per trial: T1/T2=sfp, T3=sc
- [ ] Check `CONNECTOR_TYPE` and `CABLE_NAME` class vars are correct in CableInsertion.py
- [ ] Verify image scale: Basler 1152×1024 → ACT input [3,256,288] (0.25 scale)
- [ ] Check F/T sensor tare runs before each episode in collect_demos.sh
- [ ] Review force-guided phase: `_SEAT_STIFFNESS`, `_insertion_vz` are connector-specific
- [ ] Re-collect demos for failing trial — check dataset episode count is correct
- [ ] Re-train failing trial: increment version tag and re-submit

---

## Key Parameters Reference

| Parameter | Value |
|-----------|-------|
| Camera resolution | 1152 × 1024 (3 Basler cameras) |
| ACT input image | [3, 256, 288] (0.25 scale) |
| ACT chunk_size | 25 |
| ACT inference rate | 4 Hz |
| State dim | 26 |
| Action dim | 7 |
| T1/T2 board pose | x=0.15, y=-0.2, z=1.14, yaw=3.1415 |
| T3 board pose | x=0.17, y=0.0, z=1.14, yaw=3.0 |
| T1 target frame | task_board/nic_card_mount_0/sfp_port_0_link |
| T2 target frame | task_board/nic_card_mount_1/sfp_port_0_link |
| T3 target frame | task_board/sc_port_1/sc_port_base_link |
| T1/T2 cable | cable_0, sfp_sc_cable |
| T3 cable | cable_1, sfp_sc_cable_reversed |
| EC2 instance | g5.2xlarge (A10G 24 GB VRAM) |
| ECR registry | 973918476471.dkr.ecr.us-east-1.amazonaws.com |
| ECR repo | aic-team/<team_name> |

---

## File Quick Reference

| Script | Where to run | Purpose |
|--------|-------------|---------|
| `scripts/aws_setup.sh` | EC2 (once) | Full environment setup |
| `scripts/update_and_run.sh` | EC2 (each session) | Sync code, check status |
| `scripts/verify_env.sh` | EC2 | WaveArm + CheatCode checks |
| `scripts/collect_demos.sh` | EC2 | Automated demo collection |
| `scripts/train_act.sh <N>` | EC2 (per trial) | ACT training |
| `scripts/copy_checkpoints.sh <dns>` | Local laptop | SCP checkpoints from EC2 |
| `scripts/build_and_submit.sh <tag>` | Local laptop | Build image and push to ECR |

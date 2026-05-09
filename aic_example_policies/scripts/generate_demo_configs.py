#!/usr/bin/env python3
"""Generate varied Gazebo configs for AIC cable-insertion demo collection.

Each output file is a complete copy of sample_config.yaml with the relevant
trial's board pose, rail positions, and fixture mounts varied.

Trial   Card        Rails            Mount rails       Count
------  ----------  ---------------  ----------------  -----
T1      nic_card_0  nic_rail_0..4    sfp_mount_rail_0  150  (40 structured + 110 random)
                                     sc_mount_rail_0
T2      nic_card_1  nic_rail_0..4    sfp_mount_rail_0  150  (40 structured + 110 random)
                                     sc_mount_rail_0
T3      sc_mount_1  sc_rail_1        sfp_mount_rail_0  200  (150 random + 50 terminal)
                                     sc_mount_rail_0

Usage (run from ws_aic/src/aic/):
    python aic_example_policies/scripts/generate_demo_configs.py
"""

import argparse
import copy
import math
from pathlib import Path

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Rail limits (source: sample_config.yaml task_board_limits)
# ---------------------------------------------------------------------------
NIC_RAIL_MIN = -0.0215
NIC_RAIL_MAX = 0.0234
SC_RAIL_MIN = -0.06
SC_RAIL_MAX = 0.055
MOUNT_RAIL_MIN = -0.09425
MOUNT_RAIL_MAX = 0.09425

BOARD_Z = 1.14

# 8 board yaw values covering full 360°
BOARD_YAWS = [
    0.0,
    math.pi / 4,       # 45°
    math.pi / 2,       # 90°
    3 * math.pi / 4,   # 135°
    math.pi,           # 180°
    5 * math.pi / 4,   # 225°
    3 * math.pi / 2,   # 270°
    7 * math.pi / 4,   # 315°
]

NIC_RAILS = ["nic_rail_0", "nic_rail_1", "nic_rail_2", "nic_rail_3", "nic_rail_4"]

# NIC rail yaw variation ±10°
NIC_YAW_MIN = -0.1745
NIC_YAW_MAX = 0.1745

# Fixture mount yaw variation ±60°
MOUNT_YAW_MIN = -1.047
MOUNT_YAW_MAX = 1.047


# ---------------------------------------------------------------------------
# YAML manipulation helpers
# ---------------------------------------------------------------------------

def _set_board_pose(config: dict, trial_key: str, x: float, y: float, yaw: float) -> None:
    pose = config["trials"][trial_key]["scene"]["task_board"]["pose"]
    pose["x"] = round(float(x), 6)
    pose["y"] = round(float(y), 6)
    pose["z"] = BOARD_Z
    pose["yaw"] = round(float(yaw), 6)


def _configure_nic_rail(
    config: dict, trial_key: str, active_rail: str,
    entity_name: str, translation: float, yaw: float
) -> None:
    """Enable one NIC rail with given pose; disable all others."""
    task_board = config["trials"][trial_key]["scene"]["task_board"]
    for rail in NIC_RAILS:
        if rail == active_rail:
            task_board[rail] = {
                "entity_present": True,
                "entity_name": entity_name,
                "entity_pose": {
                    "translation": round(float(translation), 6),
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": round(float(yaw), 6),
                },
            }
        else:
            task_board[rail] = {"entity_present": False}


def _set_mount_pose(
    config: dict, trial_key: str, rail_key: str,
    translation: float, yaw: float
) -> None:
    """Update translation and yaw for an always-present fixture mount rail."""
    ep = config["trials"][trial_key]["scene"]["task_board"][rail_key]["entity_pose"]
    ep["translation"] = round(float(translation), 6)
    ep["yaw"] = round(float(yaw), 6)


def _save(config: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fh:
        yaml.dump(config, fh, default_flow_style=False, sort_keys=False)


# ---------------------------------------------------------------------------
# Config generation: T1
# ---------------------------------------------------------------------------

def generate_t1(base: dict, out: Path, rng: np.random.Generator) -> None:
    """T1: 150 configs. 40 structured (8 yaws × 5 rails) + 110 random."""
    entries = []

    # 40 structured: every combination of 8 board yaws × 5 NIC rails
    for board_yaw in BOARD_YAWS:
        for rail in NIC_RAILS:
            entries.append({
                "board_x":       rng.uniform(0.05, 0.35),
                "board_y":       rng.uniform(-0.35, 0.05),
                "board_yaw":     board_yaw,
                "rail":          rail,
                "nic_t":         rng.uniform(NIC_RAIL_MIN, NIC_RAIL_MAX),
                "nic_yaw":       rng.uniform(NIC_YAW_MIN, NIC_YAW_MAX),
                "sfp_mount_t":   rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
                "sfp_mount_yaw": rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
                "sc_mount_t":    rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
                "sc_mount_yaw":  rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
            })

    # 110 random: fill to 150
    for _ in range(150 - len(entries)):
        entries.append({
            "board_x":       rng.uniform(0.05, 0.35),
            "board_y":       rng.uniform(-0.35, 0.05),
            "board_yaw":     BOARD_YAWS[rng.integers(0, len(BOARD_YAWS))],
            "rail":          NIC_RAILS[rng.integers(0, len(NIC_RAILS))],
            "nic_t":         rng.uniform(NIC_RAIL_MIN, NIC_RAIL_MAX),
            "nic_yaw":       rng.uniform(NIC_YAW_MIN, NIC_YAW_MAX),
            "sfp_mount_t":   rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
            "sfp_mount_yaw": rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
            "sc_mount_t":    rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
            "sc_mount_yaw":  rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
        })

    for idx, e in enumerate(entries, start=1):
        cfg = copy.deepcopy(base)
        _set_board_pose(cfg, "trial_1", e["board_x"], e["board_y"], e["board_yaw"])
        _configure_nic_rail(cfg, "trial_1", e["rail"], "nic_card_0",
                            e["nic_t"], e["nic_yaw"])
        _set_mount_pose(cfg, "trial_1", "sfp_mount_rail_0",
                        e["sfp_mount_t"], e["sfp_mount_yaw"])
        _set_mount_pose(cfg, "trial_1", "sc_mount_rail_0",
                        e["sc_mount_t"], e["sc_mount_yaw"])
        _save(cfg, out / "t1" / f"config_{idx:03d}.yaml")

    print(f"T1: wrote {len(entries)} configs → {out}/t1/")


# ---------------------------------------------------------------------------
# Config generation: T2
# ---------------------------------------------------------------------------

def generate_t2(base: dict, out: Path, rng: np.random.Generator) -> None:
    """T2: 150 configs. 40 structured (8 yaws × 5 rails) + 110 random."""
    entries = []

    for board_yaw in BOARD_YAWS:
        for rail in NIC_RAILS:
            entries.append({
                "board_x":       rng.uniform(0.05, 0.35),
                "board_y":       rng.uniform(-0.35, 0.05),
                "board_yaw":     board_yaw,
                "rail":          rail,
                "nic_t":         rng.uniform(NIC_RAIL_MIN, NIC_RAIL_MAX),
                "nic_yaw":       rng.uniform(NIC_YAW_MIN, NIC_YAW_MAX),
                "sfp_mount_t":   rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
                "sfp_mount_yaw": rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
                "sc_mount_t":    rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
                "sc_mount_yaw":  rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
            })

    for _ in range(150 - len(entries)):
        entries.append({
            "board_x":       rng.uniform(0.05, 0.35),
            "board_y":       rng.uniform(-0.35, 0.05),
            "board_yaw":     BOARD_YAWS[rng.integers(0, len(BOARD_YAWS))],
            "rail":          NIC_RAILS[rng.integers(0, len(NIC_RAILS))],
            "nic_t":         rng.uniform(NIC_RAIL_MIN, NIC_RAIL_MAX),
            "nic_yaw":       rng.uniform(NIC_YAW_MIN, NIC_YAW_MAX),
            "sfp_mount_t":   rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
            "sfp_mount_yaw": rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
            "sc_mount_t":    rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
            "sc_mount_yaw":  rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
        })

    for idx, e in enumerate(entries, start=1):
        cfg = copy.deepcopy(base)
        _set_board_pose(cfg, "trial_2", e["board_x"], e["board_y"], e["board_yaw"])
        _configure_nic_rail(cfg, "trial_2", e["rail"], "nic_card_1",
                            e["nic_t"], e["nic_yaw"])
        _set_mount_pose(cfg, "trial_2", "sfp_mount_rail_0",
                        e["sfp_mount_t"], e["sfp_mount_yaw"])
        _set_mount_pose(cfg, "trial_2", "sc_mount_rail_0",
                        e["sc_mount_t"], e["sc_mount_yaw"])
        _save(cfg, out / "t2" / f"config_{idx:03d}.yaml")

    print(f"T2: wrote {len(entries)} configs → {out}/t2/")


# ---------------------------------------------------------------------------
# Config generation: T3
# ---------------------------------------------------------------------------

def generate_t3(base: dict, out: Path, rng: np.random.Generator) -> None:
    """T3: 200 configs. 150 random + 50 terminal (board_x ∈ [0.1, 0.2])."""
    entries = []

    # 150 random: full board x/y range
    for _ in range(150):
        entries.append({
            "board_x":       rng.uniform(0.07, 0.37),
            "board_y":       rng.uniform(-0.20, 0.20),
            "board_yaw":     BOARD_YAWS[rng.integers(0, len(BOARD_YAWS))],
            "sc_t":          rng.uniform(SC_RAIL_MIN, SC_RAIL_MAX),
            "sfp_mount_t":   rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
            "sfp_mount_yaw": rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
            "sc_mount_t":    rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
            "sc_mount_yaw":  rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
        })

    # 50 terminal: board_x ∈ [0.1, 0.2] — near-contact insertion training
    for _ in range(50):
        entries.append({
            "board_x":       rng.uniform(0.10, 0.20),
            "board_y":       rng.uniform(-0.20, 0.20),
            "board_yaw":     BOARD_YAWS[rng.integers(0, len(BOARD_YAWS))],
            "sc_t":          rng.uniform(SC_RAIL_MIN, SC_RAIL_MAX),
            "sfp_mount_t":   rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
            "sfp_mount_yaw": rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
            "sc_mount_t":    rng.uniform(MOUNT_RAIL_MIN, MOUNT_RAIL_MAX),
            "sc_mount_yaw":  rng.uniform(MOUNT_YAW_MIN, MOUNT_YAW_MAX),
        })

    for idx, e in enumerate(entries, start=1):
        cfg = copy.deepcopy(base)
        _set_board_pose(cfg, "trial_3", e["board_x"], e["board_y"], e["board_yaw"])
        # sc_rail_1: translation only (yaw fixed at 0.0 per prompt)
        cfg["trials"]["trial_3"]["scene"]["task_board"]["sc_rail_1"]["entity_pose"][
            "translation"
        ] = round(float(e["sc_t"]), 6)
        _set_mount_pose(cfg, "trial_3", "sfp_mount_rail_0",
                        e["sfp_mount_t"], e["sfp_mount_yaw"])
        _set_mount_pose(cfg, "trial_3", "sc_mount_rail_0",
                        e["sc_mount_t"], e["sc_mount_yaw"])
        _save(cfg, out / "t3" / f"config_{idx:03d}.yaml")

    print(f"T3: wrote {len(entries)} configs → {out}/t3/")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--sample-config",
        default="aic_engine/config/sample_config.yaml",
        help="Path to aic_engine/config/sample_config.yaml (default: %(default)s)",
    )
    parser.add_argument(
        "--output-dir",
        default="aic_example_policies/configs/demo_configs",
        help="Root output directory for generated configs (default: %(default)s)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="RNG seed for reproducible random configs (default: %(default)s)",
    )
    args = parser.parse_args()

    sample_path = Path(args.sample_config)
    if not sample_path.exists():
        raise FileNotFoundError(
            f"Sample config not found: {sample_path}\n"
            "Run this script from the ws_aic/src/aic/ directory, or pass --sample-config."
        )

    with open(sample_path) as fh:
        base_config = yaml.safe_load(fh)

    out = Path(args.output_dir)
    rng = np.random.default_rng(args.seed)

    generate_t1(base_config, out, rng)
    generate_t2(base_config, out, rng)
    generate_t3(base_config, out, rng)

    print(f"\nAll configs written to {out}/")
    print("  t1/config_001..150.yaml  — T1: nic_card_0 on any rail, 8 board yaws")
    print("  t2/config_001..150.yaml  — T2: nic_card_1 on any rail, 8 board yaws")
    print("  t3/config_001..200.yaml  — T3: sc_rail_1 varied, 50 terminal configs")


if __name__ == "__main__":
    main()

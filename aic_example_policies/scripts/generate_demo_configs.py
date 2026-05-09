#!/usr/bin/env python3
"""Generate varied Gazebo configs for AIC cable-insertion demo collection.

Each output file is a complete copy of sample_config.yaml with only the
connector translation values changed for the target trial.

Trial   Rail varied           Mount varied          Count
------  --------------------  --------------------  -----
T1      nic_rail_0            sfp_mount_rail_0      50  (25 grid  + 25 random)
T2      nic_rail_1            sfp_mount_rail_0      50  (25 grid  + 25 random)
T3      sc_rail_1             sc_mount_rail_0       100 (25 grid  + 50 random + 25 edge)

Usage (run from ws_aic/src/aic/):

    python aic_example_policies/scripts/generate_demo_configs.py

    # or with explicit paths:
    python aic_example_policies/scripts/generate_demo_configs.py \\
        --sample-config aic_engine/config/sample_config.yaml \\
        --output-dir aic_example_policies/configs/demo_configs \\
        --seed 42
"""

import argparse
import copy
from pathlib import Path

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Translation limits (source: sample_config.yaml task_board_limits)
# ---------------------------------------------------------------------------
NIC_RAIL_MIN = -0.0215
NIC_RAIL_MAX = 0.0234
SC_RAIL_MIN = -0.06
SC_RAIL_MAX = 0.055
MOUNT_RAIL_MIN = -0.09425
MOUNT_RAIL_MAX = 0.09425


# ---------------------------------------------------------------------------
# Point generation helpers
# ---------------------------------------------------------------------------

def _grid_points(n_side: int, x_min: float, x_max: float,
                 y_min: float, y_max: float) -> list:
    """Return n_side² points on a regular grid over [x_min,x_max]×[y_min,y_max]."""
    xs = np.linspace(x_min, x_max, n_side)
    ys = np.linspace(y_min, y_max, n_side)
    return [(round(float(x), 6), round(float(y), 6))
            for x in xs for y in ys]


def _random_points(n: int, x_min: float, x_max: float,
                   y_min: float, y_max: float, rng: np.random.Generator) -> list:
    """Return n uniform-random points inside [x_min,x_max]×[y_min,y_max]."""
    return [(round(float(rng.uniform(x_min, x_max)), 6),
             round(float(rng.uniform(y_min, y_max)), 6))
            for _ in range(n)]


def _perimeter_points(n: int, x_min: float, x_max: float,
                      y_min: float, y_max: float) -> list:
    """Return n points sampled uniformly around the perimeter of the rectangle.

    Traversal order: bottom → right → top → left (counter-clockwise from
    bottom-left corner).  Useful for edge-case stress testing.
    """
    dx = x_max - x_min
    dy = y_max - y_min
    perim = 2.0 * (dx + dy)
    pts = []
    for i in range(n):
        t = (i / n) * perim
        if t <= dx:
            pts.append((round(x_min + t, 6), round(y_min, 6)))
        elif t <= dx + dy:
            pts.append((round(x_max, 6), round(y_min + (t - dx), 6)))
        elif t <= 2.0 * dx + dy:
            pts.append((round(x_max - (t - dx - dy), 6), round(y_max, 6)))
        else:
            pts.append((round(x_min, 6), round(y_max - (t - 2.0 * dx - dy), 6)))
    return pts


# ---------------------------------------------------------------------------
# YAML manipulation helpers
# ---------------------------------------------------------------------------

def _set_translation(config: dict, trial_key: str, rail_key: str,
                     value: float) -> None:
    """Set trials.<trial>.<task_board>.<rail>.entity_pose.translation."""
    config["trials"][trial_key]["scene"]["task_board"][rail_key][
        "entity_pose"
    ]["translation"] = round(float(value), 6)


def _save(config: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fh:
        yaml.dump(config, fh, default_flow_style=False, sort_keys=False)


# ---------------------------------------------------------------------------
# Per-trial generators
# ---------------------------------------------------------------------------

def generate_t1(base: dict, out: Path, rng: np.random.Generator) -> None:
    """T1: nic_rail_0 + sfp_mount_rail_0; 25 grid + 25 random = 50 configs."""
    pts = (
        _grid_points(5, NIC_RAIL_MIN, NIC_RAIL_MAX, MOUNT_RAIL_MIN, MOUNT_RAIL_MAX)
        + _random_points(25, NIC_RAIL_MIN, NIC_RAIL_MAX, MOUNT_RAIL_MIN, MOUNT_RAIL_MAX, rng)
    )
    for idx, (nic_t, sfp_t) in enumerate(pts, start=1):
        cfg = copy.deepcopy(base)
        _set_translation(cfg, "trial_1", "nic_rail_0", nic_t)
        _set_translation(cfg, "trial_1", "sfp_mount_rail_0", sfp_t)
        _save(cfg, out / "t1" / f"config_{idx:03d}.yaml")
    print(f"T1: wrote {len(pts)} configs → {out}/t1/")


def generate_t2(base: dict, out: Path, rng: np.random.Generator) -> None:
    """T2: nic_rail_1 + sfp_mount_rail_0; 25 grid + 25 random = 50 configs."""
    pts = (
        _grid_points(5, NIC_RAIL_MIN, NIC_RAIL_MAX, MOUNT_RAIL_MIN, MOUNT_RAIL_MAX)
        + _random_points(25, NIC_RAIL_MIN, NIC_RAIL_MAX, MOUNT_RAIL_MIN, MOUNT_RAIL_MAX, rng)
    )
    for idx, (nic_t, sfp_t) in enumerate(pts, start=1):
        cfg = copy.deepcopy(base)
        _set_translation(cfg, "trial_2", "nic_rail_1", nic_t)
        _set_translation(cfg, "trial_2", "sfp_mount_rail_0", sfp_t)
        _save(cfg, out / "t2" / f"config_{idx:03d}.yaml")
    print(f"T2: wrote {len(pts)} configs → {out}/t2/")


def generate_t3(base: dict, out: Path, rng: np.random.Generator) -> None:
    """T3: sc_rail_1 + sc_mount_rail_0; 25 grid + 50 random + 25 edge = 100 configs."""
    pts = (
        _grid_points(5, SC_RAIL_MIN, SC_RAIL_MAX, MOUNT_RAIL_MIN, MOUNT_RAIL_MAX)
        + _random_points(50, SC_RAIL_MIN, SC_RAIL_MAX, MOUNT_RAIL_MIN, MOUNT_RAIL_MAX, rng)
        + _perimeter_points(25, SC_RAIL_MIN, SC_RAIL_MAX, MOUNT_RAIL_MIN, MOUNT_RAIL_MAX)
    )
    for idx, (sc_t, sc_mount_t) in enumerate(pts, start=1):
        cfg = copy.deepcopy(base)
        _set_translation(cfg, "trial_3", "sc_rail_1", sc_t)
        _set_translation(cfg, "trial_3", "sc_mount_rail_0", sc_mount_t)
        _save(cfg, out / "t3" / f"config_{idx:03d}.yaml")
    print(f"T3: wrote {len(pts)} configs → {out}/t3/")


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
    print("  t1/config_001..050.yaml  — T1: nic_rail_0 + sfp_mount_rail_0")
    print("  t2/config_001..050.yaml  — T2: nic_rail_1 + sfp_mount_rail_0")
    print("  t3/config_001..100.yaml  — T3: sc_rail_1  + sc_mount_rail_0")


if __name__ == "__main__":
    main()

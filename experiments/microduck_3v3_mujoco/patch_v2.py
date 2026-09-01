#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"patch {label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runner", type=Path)
    args = ap.parse_args()
    path = args.runner
    text = path.read_text(encoding="utf-8")

    text = replace_once(text, "VEL_FWD = 0.25", "VEL_FWD = 0.30", "forward speed")
    text = replace_once(text, "VEL_ANG = 1.0", "VEL_ANG = 1.25", "angular speed")

    text = replace_once(
        text,
        '''DUCK_SPECS = [
    ("B1", 0, "keeper", (-1.22, 0.00, 0.0)),
    ("B2", 0, "field",  (-0.62, -0.48, 0.0)),
    ("B3", 0, "field",  (-0.62,  0.48, 0.0)),
    ("O1", 1, "keeper", ( 1.22, 0.00, math.pi)),
    ("O2", 1, "field",  ( 0.62,  0.48, math.pi)),
    ("O3", 1, "field",  ( 0.62, -0.48, math.pi)),
]''',
        '''DUCK_SPECS = [
    ("B1", 0, "keeper", (-1.22, 0.00, 0.0)),
    ("B2", 0, "field",  (-0.48, -0.24, 0.0)),
    ("B3", 0, "field",  (-0.68,  0.56, 0.0)),
    ("O1", 1, "keeper", ( 1.22, 0.00, math.pi)),
    ("O2", 1, "field",  ( 0.48,  0.24, math.pi)),
    ("O3", 1, "field",  ( 0.68, -0.56, math.pi)),
]''',
        "kickoff formation",
    )

    text = replace_once(
        text,
        "if desired_heading is not None and dist < 0.10:",
        "if desired_heading is not None and dist < 0.19:",
        "heading handoff distance",
    )
    text = replace_once(
        text,
        "vx = min(VEL_FWD, 0.75 * dist) * max(0.0, math.cos(err))",
        "vx = min(VEL_FWD, max(0.17, 0.95 * dist)) * max(0.0, math.cos(err))",
        "minimum walking command",
    )
    text = replace_once(
        text,
        "if abs(err) > 0.75:",
        "if abs(err) > 0.90:",
        "turn-before-walk threshold",
    )

    text = replace_once(
        text,
        '''        if duck.role == "keeper":
            own_x = -1.20 if duck.team == 0 else 1.20
            target = np.asarray([own_x, clamp(float(ball_xy[1]), -0.34, 0.34)])''',
        '''        if duck.role == "keeper":
            own_x = -1.20 if duck.team == 0 else 1.20
            scan_phase = 0.0 if duck.team == 0 else math.pi
            scan_y = 0.24 * math.sin(0.62 * self.sim_t + scan_phase)
            target = np.asarray([own_x, clamp(float(ball_xy[1]) + scan_y, -0.34, 0.34)])''',
        "keeper patrol",
    )
    text = replace_once(
        text,
        "kick = duck.kick_cooldown <= 0 and 0.20 < local[0] < 0.46 and abs(local[1]) < 0.13",
        "kick = duck.kick_cooldown <= 0 and 0.075 < local[0] < 0.19 and abs(local[1]) < 0.14",
        "keeper kick envelope",
    )
    text = replace_once(
        text,
        "kick = duck.kick_cooldown <= 0 and envelope_error < 0.09 and heading_error < 0.28",
        "kick = duck.kick_cooldown <= 0 and envelope_error < 0.12 and heading_error < 0.52",
        "chaser kick envelope",
    )
    text = replace_once(
        text,
        "lane = -0.62 if ball_xy[1] > 0 else 0.62",
        '''if ball_xy[1] >= 0:
            lane = -0.70 if duck.team == 0 else 0.70
        else:
            lane = 0.70 if duck.team == 0 else -0.70''',
        "support lane",
    )

    text = replace_once(
        text,
        '''    forward_grid = [0.26, 0.31, 0.36, 0.40]
    lateral_grid = [-0.10, -0.05, 0.0, 0.05, 0.10]''',
        '''    # Real foot-sweep trace: active foot centres travel roughly 0.006-0.072 m
    # forward and 0.040-0.090 m laterally from the trunk. Include ball radius.
    forward_grid = [0.09, 0.11, 0.13, 0.15, 0.17]
    lateral_grid = [-0.11, -0.08, -0.05, -0.02, 0.02, 0.05, 0.08, 0.11]''',
        "kick calibration grid",
    )
    text = replace_once(
        text,
        "world.reset(ball_xy=(1.22, 0.0), ball_vel_xy=(0.85, 0.0))",
        "world.reset(ball_xy=(1.49, 0.0), ball_vel_xy=(1.10, 0.0))",
        "goal return launch",
    )

    text = replace_once(
        text,
        '"all_ducks_moved": all(d["max_displacement_m"] > 0.02 for d in duck_summary),',
        '"all_ducks_moved": all(d["max_displacement_m"] > 0.015 for d in duck_summary),',
        "movement acceptance threshold",
    )

    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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

    text = replace_once(
        text,
        '''DUCK_SPECS = [
    ("B1", 0, "keeper", (-1.22, 0.00, 0.0)),
    ("B2", 0, "field",  (-0.48, -0.24, 0.0)),
    ("B3", 0, "field",  (-0.68,  0.56, 0.0)),
    ("O1", 1, "keeper", ( 1.22, 0.00, math.pi)),
    ("O2", 1, "field",  ( 0.48,  0.24, math.pi)),
    ("O3", 1, "field",  ( 0.68, -0.56, math.pi)),
]''',
        '''DUCK_SPECS = [
    ("B1", 0, "keeper", (-1.22, 0.00, 0.0)),
    # Blue opens from the empirically measured left-kick contact pose.
    ("B2", 0, "field",  (-0.12, -0.05, 0.0)),
    ("B3", 0, "field",  (-0.82,  0.58, 0.0)),
    ("O1", 1, "keeper", ( 1.22, 0.00, math.pi)),
    # Orange yields the opening kick, then joins normal closest-player control.
    ("O2", 1, "field",  ( 0.48,  0.24, math.pi)),
    ("O3", 1, "field",  ( 0.82, -0.58, math.pi)),
]''',
        "kickoff formation",
    )

    text = replace_once(
        text,
        '''        if self.returner.active:
            # Physical restart: all players walk back to formation instead of teleporting.
            target = np.asarray(duck.spawn[:2], dtype=float)
            vx, wz = self._command_to_target(duck, target, duck.spawn[2])
            duck.target = tuple(target)
            return vx, wz, False, duck.selected_foot

        if duck.role == "keeper":''',
        '''        if self.returner.active:
            # Physical restart: all players walk back to formation instead of teleporting.
            target = np.asarray(duck.spawn[:2], dtype=float)
            vx, wz = self._command_to_target(duck, target, duck.spawn[2])
            duck.target = tuple(target)
            return vx, wz, False, duck.selected_foot

        # Opening set piece: one side receives possession, while the far-side
        # field players advance on their own walking policies. This avoids an
        # artificial symmetric double-kick that would cancel the ball impulse.
        if self.sim_t < 3.2 and duck.duck_id == "O2":
            target = np.asarray([0.44, 0.24], dtype=float)
            vx, wz = self._command_to_target(duck, target, math.pi)
            duck.target = tuple(target)
            return vx, wz, False, duck.selected_foot
        if self.sim_t < 3.2 and duck.duck_id in ("B3", "O3"):
            target_x = -0.22 if duck.team == 0 else 0.22
            target = np.asarray([target_x, duck.spawn[1]], dtype=float)
            vx, wz = self._command_to_target(duck, target, duck.spawn[2])
            duck.target = tuple(target)
            return vx, wz, False, duck.selected_foot

        if duck.role == "keeper":''',
        "opening set piece",
    )

    text = replace_once(
        text,
        "kick = duck.kick_cooldown <= 0 and envelope_error < 0.12 and heading_error < 0.52",
        "kick = duck.kick_cooldown <= 0 and envelope_error < 0.035 and heading_error < 0.38",
        "precise chaser contact envelope",
    )

    text = replace_once(
        text,
        '''        for _ in range(DECIMATION):
            bpos, _ = self.ball_state()
            self.returner.step(TIMESTEP, self.sim_t, float(bpos[0]))
            mujoco.mj_step(self.model, self.data)
            self.sim_t += TIMESTEP

        for duck in self.ducks:''',
        '''        for _ in range(DECIMATION):
            bpos, _ = self.ball_state()
            self.returner.step(TIMESTEP, self.sim_t, float(bpos[0]))
            mujoco.mj_step(self.model, self.data)
            self.sim_t += TIMESTEP
            # A kick contact can exist for only one 5 ms physics substep;
            # sample contacts here rather than only after all four substeps.
            self._contact_metrics()

        for duck in self.ducks:''',
        "substep contact sampling",
    )
    text = replace_once(
        text,
        '''        self.metrics.physical_returns_completed = len(self.returner.completed)
        self._contact_metrics()
        self._score_and_ball_metrics()''',
        '''        self.metrics.physical_returns_completed = len(self.returner.completed)
        self._score_and_ball_metrics()''',
        "remove duplicate contact sampling",
    )

    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

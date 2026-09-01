#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import mujoco
import numpy as np


def load_runner(path: Path):
    spec = importlib.util.spec_from_file_location("microduck_six", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def local_xy(world_xy: np.ndarray, origin_xy: np.ndarray, yaw: float) -> np.ndarray:
    delta = np.asarray(world_xy, dtype=float) - np.asarray(origin_xy, dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])


def summarize_points(points: list[list[float]]) -> dict[str, object]:
    a = np.asarray(points, dtype=float)
    return {
        "count": int(len(a)),
        "min": np.min(a, axis=0).tolist(),
        "max": np.max(a, axis=0).tolist(),
        "mean": np.mean(a, axis=0).tolist(),
        "max_planar_radius": float(np.max(np.linalg.norm(a[:, :2], axis=1))),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runner", type=Path, required=True)
    ap.add_argument("--source-app", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    m = load_runner(args.runner.resolve())
    source_app = args.source_app.resolve()
    model_dir = source_app / "public" / "robot" / "mjlab"
    model, manifest, xml_path = m.compile_model_with_fallback(
        model_dir / "robot_allcollisions.xml",
        model_dir / "meshes",
        args.output,
    )
    data = mujoco.MjData(model)
    policies = m.load_policies(source_app / "public" / "policies")
    world = m.SixDuckWorld(
        model,
        data,
        policies,
        calibration={"left": (0.35, 0.05), "right": (0.35, -0.05)},
        output_dir=args.output,
    )
    duck = world.duck_by_id("B2")

    foot_geoms = {
        "left": m.mj_id(model, mujoco.mjtObj.mjOBJ_GEOM, "b2_left_foot_collision"),
        "right": m.mj_id(model, mujoco.mjtObj.mjOBJ_GEOM, "b2_right_foot_collision"),
    }
    foot_sites = {
        "left": m.mj_id(model, mujoco.mjtObj.mjOBJ_SITE, "b2_left_foot"),
        "right": m.mj_id(model, mujoco.mjtObj.mjOBJ_SITE, "b2_right_foot"),
    }

    report: dict[str, object] = {
        "source_xml": str(xml_path),
        "model": {"nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu)},
        "manifest": manifest,
        "feet": {},
    }

    for foot in ("left", "right"):
        world.reset(calibration_mode=True, ball_xy=(1.35, 1.15))
        m.settle(world, 45, active_only="B2")
        root0, yaw0, _ = world.robot_pose(duck)
        q0 = np.asarray(data.qpos[duck.qpos_adr], dtype=float).copy()
        mode = "kickL" if foot == "left" else "kickR"
        duck.mode = mode
        duck.kick_steps_left = m.KICK_STEPS
        duck.cmd.fill(0)
        frames = []
        action_norms = []
        max_joint_delta = 0.0
        for step in range(m.KICK_STEPS + 65):
            world.control_step(strategy_enabled=False, active_only="B2")
            action_norms.append(float(np.linalg.norm(duck.last_action)))
            max_joint_delta = max(
                max_joint_delta,
                float(np.max(np.abs(np.asarray(data.qpos[duck.qpos_adr]) - q0))),
            )
            gpos = np.asarray(data.geom_xpos[foot_geoms[foot]], dtype=float)
            spos = np.asarray(data.site_xpos[foot_sites[foot]], dtype=float)
            opos = np.asarray(data.geom_xpos[foot_geoms["right" if foot == "left" else "left"]], dtype=float)
            gloc = local_xy(gpos[:2], root0[:2], yaw0)
            sloc = local_xy(spos[:2], root0[:2], yaw0)
            oloc = local_xy(opos[:2], root0[:2], yaw0)
            frames.append({
                "step": step + 1,
                "mode": duck.mode,
                "root": np.asarray(data.qpos[duck.root_qpos_adr:duck.root_qpos_adr+3], dtype=float).tolist(),
                "active_geom_local": [float(gloc[0]), float(gloc[1]), float(gpos[2])],
                "active_site_local": [float(sloc[0]), float(sloc[1]), float(spos[2])],
                "other_geom_local": [float(oloc[0]), float(oloc[1]), float(opos[2])],
                "action_norm": float(np.linalg.norm(duck.last_action)),
                "action_max_abs": float(np.max(np.abs(duck.last_action))),
            })
        active_geom = [f["active_geom_local"] for f in frames]
        active_site = [f["active_site_local"] for f in frames]
        neutral = frames[0]["active_geom_local"]
        best_forward = max(frames, key=lambda f: f["active_geom_local"][0])
        report["feet"][foot] = {
            "policy_mode": mode,
            "kick_steps": int(m.KICK_STEPS),
            "max_action_norm": max(action_norms),
            "min_action_norm": min(action_norms),
            "max_joint_delta_rad": max_joint_delta,
            "neutral_active_geom_local": neutral,
            "active_geom_bounds": summarize_points(active_geom),
            "active_site_bounds": summarize_points(active_site),
            "most_forward_frame": best_forward,
            "frames": frames,
        }

    (args.output / "kick_foot_sweep.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({
        "model": report["model"],
        "left": {k: report["feet"]["left"][k] for k in ("max_action_norm", "max_joint_delta_rad", "active_geom_bounds", "most_forward_frame")},
        "right": {k: report["feet"]["right"][k] for k in ("max_action_norm", "max_joint_delta_rad", "active_geom_bounds", "most_forward_frame")},
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

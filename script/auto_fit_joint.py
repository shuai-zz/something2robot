#!/usr/bin/env python3
"""Automatically generate, motion-check, and refine a printable joint."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON = sys.executable


def run(command: list[str]) -> None:
    print("\n$", " ".join(command), flush=True)
    subprocess.run(command, cwd=REPO_ROOT, check=True)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def collision_summary(report: dict) -> dict:
    collisions = [row for row in report["results"] if row["collision"]]
    geometry_rows = [row for row in collisions if "collision_distance_max_mm" in row]
    return {
        "collision_angles": len(collisions),
        "penetrating_samples": sum(row["penetrating_samples"] for row in collisions),
        "max_penetration_mm": max((row["max_penetration_mm"] for row in collisions), default=0.0),
        "max_collision_distance_mm": max(
            (row["collision_distance_max_mm"] for row in geometry_rows), default=0.0
        ),
        "max_body_side_depth_mm": max(
            (row["collision_axial_max_mm"] for row in geometry_rows), default=0.0
        ),
        "min_collision_radial_mm": min(
            (row["collision_radial_min_mm"] for row in geometry_rows), default=0.0
        ),
    }


def candidate_score(generation: dict, motion: dict) -> tuple:
    invalid = sum(
        not item["watertight"] or item["components"] != 1
        for item in generation["validation"].values()
    )
    summary = collision_summary(motion)
    return (
        invalid,
        summary["collision_angles"],
        summary["penetrating_samples"],
        summary["max_penetration_mm"],
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a local round joint and refine it until a motion sweep passes."
    )
    parser.add_argument("--model", required=True)
    parser.add_argument("--link", required=True)
    parser.add_argument("--parent-link", default="BODY")
    parser.add_argument("--parts-mm", type=Path, required=True)
    parser.add_argument("--expected-x", type=float, required=True)
    parser.add_argument("--angle-min", type=int, default=-45)
    parser.add_argument("--angle-max", type=int, default=45)
    parser.add_argument("--angle-step", type=int, default=5)
    parser.add_argument("--max-iterations", type=int, default=8)
    parser.add_argument("--initial-cut-offset", type=float, default=0.0)
    parser.add_argument("--offset-step", type=float, default=1.0)
    parser.add_argument("--tenon-radius", type=float, default=3.0)
    parser.add_argument("--tenon-depth", type=float, default=4.0)
    parser.add_argument("--tenon-root-overlap", type=float, default=3.0)
    parser.add_argument("--clearance", type=float, default=0.3)
    parser.add_argument("--collar-thickness", type=float, default=1.5)
    parser.add_argument("--initial-relief-radius", type=float, default=7.5)
    parser.add_argument("--minimum-collision-samples", type=int, default=5)
    parser.add_argument("--penetration-tolerance", type=float, default=0.2)
    parser.add_argument("--samples", type=int, default=120000)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    cut_offset = args.initial_cut_offset
    relief_radius = args.initial_relief_radius
    body_depth = 0.0
    inner_radius = args.tenon_radius + args.clearance + args.collar_thickness + 1.2
    attempts = []
    best = None
    stalled_rounds = 0
    previous_score = None

    for iteration in range(1, args.max_iterations + 1):
        round_dir = args.out_dir / f"round_{iteration:02d}"
        generator = [
            PYTHON, str(REPO_ROOT / "script" / "local_joint_voxel_tenon.py"),
            "--model", args.model,
            "--link", args.link,
            "--parent-link", args.parent_link,
            "--expected-x", str(args.expected_x),
            "--parts-mm", str(args.parts_mm.resolve()),
            "--cut-offset", str(cut_offset),
            "--tenon-radius", str(args.tenon_radius),
            "--tenon-depth", str(args.tenon_depth),
            "--tenon-root-overlap", str(args.tenon_root_overlap),
            "--clearance", str(args.clearance),
            "--collar-thickness", str(args.collar_thickness),
            "--motion-relief-radius", str(relief_radius),
            "--motion-relief-body-depth", str(body_depth),
            "--motion-relief-inner-radius", str(inner_radius),
            "--connector-geometry", "round",
            "--round-sections", "256",
            "--lead-in", "0.5",
            "--out-dir", str(round_dir),
        ]
        run(generator)
        generation = load_json(round_dir / "report.json")
        moving_name = f"{args.link}_local_voxel_tenon.stl"
        center = generation["connector_center_mm"]
        axis = generation["connector_axis_into_body"]
        topology_ok = all(
            item["watertight"] and item["components"] == 1
            for item in generation["validation"].values()
        )
        if not topology_ok:
            score = (1, 10**6, 10**9, float("inf"))
            attempt = {
                "iteration": iteration,
                "cut_offset_mm": cut_offset,
                "relief_radius_mm": relief_radius,
                "body_relief_depth_mm": body_depth,
                "score": list(score),
                "collision_angles": None,
                "failure": "topology validation failed; motion check skipped",
                "directory": str(round_dir.resolve()),
            }
            attempts.append(attempt)
            print("Iteration result:", json.dumps(attempt, indent=2), flush=True)
            if best is None or score < best[0]:
                best = (score, round_dir, generation, None)
            cut_offset = min(
                cut_offset - args.offset_step,
                args.initial_cut_offset - args.offset_step,
            )
            continue
        motion_dir = round_dir / "motion_check"
        checker = [
            PYTHON, str(REPO_ROOT / "script" / "check_joint_motion.py"),
            "--body", str(round_dir / f"{args.parent_link}_local_voxel_tenon.stl"),
            "--moving", str(round_dir / moving_name),
            "--center", *map(str, center),
            "--axis", *map(str, axis),
            "--angle-min", str(args.angle_min),
            "--angle-max", str(args.angle_max),
            "--angle-step", str(args.angle_step),
            "--samples", str(args.samples),
            "--penetration-tolerance", str(args.penetration_tolerance),
            "--minimum-collision-samples", str(args.minimum_collision_samples),
            "--out-dir", str(motion_dir),
        ]
        run(checker)
        motion = load_json(motion_dir / "motion_report.json")
        summary = collision_summary(motion)
        score = candidate_score(generation, motion)
        attempt = {
            "iteration": iteration,
            "cut_offset_mm": cut_offset,
            "relief_radius_mm": relief_radius,
            "body_relief_depth_mm": body_depth,
            "score": list(score),
            **summary,
            "directory": str(round_dir.resolve()),
        }
        attempts.append(attempt)
        print("Iteration result:", json.dumps(attempt, indent=2), flush=True)
        if best is None or score < best[0]:
            best = (score, round_dir, generation, motion)

        if topology_ok and summary["collision_angles"] == 0:
            break

        if previous_score is not None and score >= previous_score:
            stalled_rounds += 1
        else:
            stalled_rounds = 0
        previous_score = score

        # Fit the next clearance directly to measured collision geometry.
        if summary["max_collision_distance_mm"] > 0:
            requested_radius = max(
                relief_radius + 0.5, summary["max_collision_distance_mm"] + args.clearance + 0.5
            )
            relief_radius = min(requested_radius, relief_radius + 4.0)
            requested_depth = max(
                body_depth, summary["max_body_side_depth_mm"] + args.clearance + 0.5
            )
            body_depth = min(requested_depth, body_depth + 2.5)
            if summary["min_collision_radial_mm"] > 0:
                inner_radius = min(
                    inner_radius,
                    max(
                        args.tenon_radius + args.clearance + args.collar_thickness + 0.5,
                        summary["min_collision_radial_mm"] - 0.5,
                    ),
                )
        else:
            relief_radius += 1.0

        # A topology failure or collision inside the protected socket support
        # cannot be fixed by carving more material: move the pivot into BODY.
        collision_reaches_support = (
            summary["min_collision_radial_mm"] > 0
            and summary["min_collision_radial_mm"] <= inner_radius + 0.2
        )
        if not topology_ok or collision_reaches_support:
            cut_offset = min(cut_offset - args.offset_step, args.initial_cut_offset - args.offset_step)
            stalled_rounds = 0
        # If clearance growth merely stops improving, alternate around the
        # annotated center, trying the body side first.
        elif stalled_rounds >= 1:
            distance = args.offset_step * max(1, (iteration + 1) // 2)
            cut_offset = args.initial_cut_offset - distance
            stalled_rounds = 0

    assert best is not None
    best_score, best_dir, best_generation, best_motion = best
    final_dir = args.out_dir / "final"
    if final_dir.exists():
        shutil.rmtree(final_dir)
    shutil.copytree(best_dir, final_dir)
    success = best_score[0] == 0 and best_score[1] == 0
    report = {
        "success": success,
        "model": args.model,
        "link": args.link,
        "parent_link": args.parent_link,
        "requested_angle_range_degrees": [args.angle_min, args.angle_max],
        "iterations_run": len(attempts),
        "best_round": best_dir.name,
        "best_score": list(best_score),
        "attempts": attempts,
        "final_directory": str(final_dir.resolve()),
    }
    (args.out_dir / "auto_fit_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print("\nAUTO-FIT RESULT\n", json.dumps(report, indent=2), flush=True)
    if not success:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

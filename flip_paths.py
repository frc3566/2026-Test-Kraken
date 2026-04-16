"""
flip_paths.py
Flips all PathPlanner .path files over the horizontal line y = AXIS_Y,
writing output files named FLIPPED_<OriginalName> in the same folder.

Flip rule:s
  new_y      = 2 * AXIS_Y - old_y   (reflect y over the axis)
  new_angle  = -old_angle            (mirror heading/rotation over a horizontal axis)
"""

import json
import os

AXIS_Y = 4.021
PATHS_DIR = os.path.join(
    os.path.dirname(__file__),
    "src", "main", "deploy", "pathplanner", "paths"
)


def flip_y(y: float) -> float:
    return 2 * AXIS_Y - y


def flip_rotation(angle: float) -> float:
    return -angle


def flip_point(pt: dict | None) -> dict | None:
    """Flip a {x, y} dict in-place and return it (no-op if None)."""
    if pt is None:
        return None
    pt["y"] = flip_y(pt["y"])
    return pt


def flip_path(data: dict) -> dict:
    # ── Waypoints ──────────────────────────────────────────────────────────
    for wp in data.get("waypoints", []):
        flip_point(wp.get("anchor"))
        flip_point(wp.get("prevControl"))
        flip_point(wp.get("nextControl"))

    # ── Rotation targets ───────────────────────────────────────────────────
    for rt in data.get("rotationTargets", []):
        rt["rotationDegrees"] = flip_rotation(rt["rotationDegrees"])

    # ── Point-towards zones ────────────────────────────────────────────────
    for ptz in data.get("pointTowardsZones", []):
        flip_point(ptz.get("fieldPosition"))
        if "rotationOffset" in ptz:
            ptz["rotationOffset"] = flip_rotation(ptz["rotationOffset"])

    # ── Goal / start states ────────────────────────────────────────────────
    if "goalEndState" in data:
        data["goalEndState"]["rotation"] = flip_rotation(
            data["goalEndState"]["rotation"]
        )
    if "idealStartingState" in data:
        data["idealStartingState"]["rotation"] = flip_rotation(
            data["idealStartingState"]["rotation"]
        )

    return data


def main():
    name = input("Enter path name to flip (e.g. LeftBack): ").strip()

    # Accept input with or without the .path extension
    if not name.endswith(".path"):
        name += ".path"

    src_path = os.path.join(PATHS_DIR, name)

    if not os.path.isfile(src_path):
        print(f"Error: '{name}' not found in {PATHS_DIR}")
        available = [f for f in os.listdir(PATHS_DIR)
                     if f.endswith(".path") and not f.startswith("FLIPPED_")]
        if available:
            print("Available paths:")
            for f in sorted(available):
                print(f"  {f[:-5]}")  # strip .path for readability
        return

    with open(src_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    flipped = flip_path(data)

    out_name = name.replace("Left", "Right")
    if out_name == name:
        print(f"Warning: no 'Left' found in '{name}', output name would be identical. Aborting.")
        return
    out_path = os.path.join(PATHS_DIR, out_name)

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(flipped, f, indent=2)
        f.write("\n")

    print(f"  {name}  →  {out_name} (flipped over y={AXIS_Y})")


if __name__ == "__main__":
    main()

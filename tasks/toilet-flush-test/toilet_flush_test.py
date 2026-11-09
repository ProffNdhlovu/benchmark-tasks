"""A table-top toilet stands between the arms: a white bowl, and behind it a tank whose top carries
the flush lever - a small rocker bar on a fulcrum ridge between guide walls, exactly the inspector's
handle. NSPIRE toilet check (D181): a flush component that is damaged, inoperable, or missing limits
the resident's ability to safely discharge human waste. The inspector presses the handle and watches
its travel, then files the finding.

The test: press the flush lever's front end down past its tilt threshold. Per-trial condition via
the FAULT_MODE environment variable: 'ok' (the lever rocks and returns - the flush works), 'stiff'
(the lever is seized solid: nothing moves when pressed), or 'missing' (the handle is gone; only the
bare pivot mount remains). The condition is not announced to the policy. The robot files the report
by leaving the white tag on the matching pad: green for the flush works, yellow for the lever is
seized, red for the handle is missing.

Run it (per-trial condition via the FAULT_MODE environment variable):
    FAULT_MODE=ok inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=toilet_flush_test.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "test the toilet flush and file the report" \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/toilet-demo
"""

import os

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"

TOP = TABLE_TOP[SCENE]

FAULT_MODE = os.environ.get("FAULT_MODE", "ok")
if FAULT_MODE not in ("ok", "stiff", "missing"):
    raise ValueError(f"FAULT_MODE must be ok | stiff | missing, got {FAULT_MODE!r}")

# Cutaway toilet: bowl in front, tank behind it with the flush lever on its top.
BX, BY = 0.34, -0.08
BOWL = Cylinder("toilet_bowl", pos=(BX, BY, TOP + 0.03), radius=0.06, height=0.06,
                color=(0.94, 0.94, 0.95))
TANK = Box("toilet_tank", pos=(BX, BY + 0.115, TOP + 0.06), size=(0.16, 0.07, 0.12),
           color=(0.90, 0.90, 0.92))
# Flush lever: the smoke alarm's proven rocker on a fulcrum ridge between guide walls, mounted on
# the tank's top near its front edge. In 'stiff' mode the lever is a fixed (seized) bar; in
# 'missing' mode it is absent entirely and only the bare mount remains.
RIDGE = Box("pivot_ridge", pos=(BX, BY + 0.09, TOP + 0.122), size=(0.012, 0.005, 0.004),
            color=(0.30, 0.31, 0.33))
WALL_L = Box("wall_left", pos=(BX - 0.029, BY + 0.09, TOP + 0.125), size=(0.004, 0.032, 0.010),
             color=(0.75, 0.75, 0.75))
WALL_R = Box("wall_right", pos=(BX + 0.029, BY + 0.09, TOP + 0.125), size=(0.004, 0.032, 0.010),
             color=(0.75, 0.75, 0.75))
LEVER_REST_Z = TOP + 0.128
LEVER = Box("flush_lever", pos=(BX, BY + 0.09, LEVER_REST_Z), size=(0.05, 0.024, 0.008), mass=0.03,
            color=(0.80, 0.55, 0.15))

OBJECTS = [BOWL, TANK, RIDGE, WALL_L, WALL_R]
if FAULT_MODE == "ok":
    OBJECTS.append(LEVER)
elif FAULT_MODE == "stiff":
    SEIZED = Box("flush_lever", pos=(BX, BY + 0.09, LEVER_REST_Z), size=(0.05, 0.024, 0.008),
                 color=(0.80, 0.55, 0.15))          # no mass = fixed: the lever is seized solid
    OBJECTS.append(SEIZED)

# Report station: three pads and the white tag.
PAD_OK = Box("pad_ok", pos=(0.14, 0.08, TOP + 0.006), size=(0.07, 0.075, 0.012), color=(0.20, 0.65, 0.30))
PAD_STIFF = Box("pad_stiff", pos=(0.14, 0.0, TOP + 0.006), size=(0.07, 0.075, 0.012),
                color=(0.90, 0.75, 0.15))
PAD_MISSING = Box("pad_missing", pos=(0.14, -0.08, TOP + 0.006), size=(0.07, 0.075, 0.012),
                  color=(0.80, 0.15, 0.12))
TAG = Box("report_tag", pos=(0.22, -0.10, TOP + 0.015), size=(0.03, 0.03, 0.03), mass=0.06,
          color=(0.95, 0.95, 0.95))
OBJECTS += [PAD_OK, PAD_STIFF, PAD_MISSING, TAG]

PADS = {"ok": PAD_OK, "stiff": PAD_STIFF, "missing": PAD_MISSING}
REPORT_ON = 0.025           # tag centre within this of a pad's centre = filed on it

TILT_DIP = 0.003            # front-end displacement that counts as pressed (m)
ROCKER_HALF = 0.012
TILT_SIN = TILT_DIP / ROCKER_HALF
LEVEL_SIN = 0.002 / ROCKER_HALF

_state = {"pressed_seen": False}


def _tilt_sin(objects):
    """Signed tilt of the lever about its fulcrum: the z component of its local y axis."""
    lever = objects.get("flush_lever")
    if lever is None:
        return 0.0
    x, y, z, w = lever.quat_xyzw
    return 2.0 * (y * z + w * x)


def _update(objects):
    if abs(_tilt_sin(objects)) >= TILT_SIN:
        _state["pressed_seen"] = True


def _tag_on_pad(objects):
    """(mode, ) the tag is filed on, or None."""
    tag = objects["report_tag"]
    if not tag.resting():
        return None
    for mode, pad in PADS.items():
        s = objects[pad.name]
        if abs(tag.pos[0] - s.pos[0]) < REPORT_ON and abs(tag.pos[1] - s.pos[1]) < REPORT_ON:
            return mode
    return None


def score(objects):
    """task_score from the white tag's final resting spot: 1.0 on the pad matching the lever's
    actual condition (tag centre within 2.5 cm of the pad centre), 0.3 on a wrong pad (a report was
    filed but the diagnosis was wrong), 0.0 if the tag is left anywhere else. Do-nothing: 0."""
    _update(objects)
    mode = _tag_on_pad(objects)
    filed = 1.0 if mode == FAULT_MODE else (0.3 if mode is not None else 0.0)
    import csv, os as _os
    if _os.path.exists("/tmp/toilet_dbg_on"):
        tag = objects["report_tag"]
        row = [round(_tilt_sin(objects), 4), _state["pressed_seen"],
               round(tag.pos[0], 4), round(tag.pos[1], 4), round(tag.pos[2], 4),
               tag.resting(), mode, filed]
        with open(f"/tmp/toilet_{_os.getpid()}.csv", "a") as f:
            csv.writer(f).writerow(row)
    return filed


def success(objects):
    return score(objects) >= 1.0


SCORE_OVER = "best"     # the filed tag stays filed


def demo():
    """The right arm approaches the lever horizontally at press height (a vertical descent from a
    high hover is outside the arm's workspace), slides over its front end, presses it down past the
    tilt threshold and lets it spring back - the smoke alarm's press pattern. In 'missing' mode
    there is no lever: the arm hovers over the bare mount. Then it carries the tag to the pad
    matching the trial's condition."""
    px, py = BX, BY + 0.078              # the lever's front end
    end_top = LEVER_REST_Z + 0.004
    contact_z = end_top + 0.013
    press_z = contact_z - 0.012
    right = []
    if FAULT_MODE != "missing":
        right += [
            Waypoint(1.8, (px, py - 0.10, contact_z), -0.023, "approach at press height", yaw=0.0),
            Waypoint(1.2, (px, py, contact_z), -0.023, "over the lever's front end", yaw=0.0),
            Waypoint(0.8, (px, py, press_z), -0.023, "press", yaw=0.0),
            Waypoint(1.4, (px, py, press_z), -0.023, "hold the press", yaw=0.0),
            Waypoint(1.0, (px, py - 0.10, contact_z), -0.023, "release and back off", yaw=0.0),
        ]
    else:
        right += [
            Waypoint(1.8, (px, py - 0.10, contact_z), -0.023, "approach at press height", yaw=0.0),
            Waypoint(1.2, (px, py, contact_z), -0.023, "over the mount: no handle", yaw=0.0),
            Waypoint(1.0, (px, py - 0.10, contact_z), -0.023, "back off", yaw=0.0),
        ]
    # file the tag on the matching pad
    pad = PADS[FAULT_MODE]
    place = (pad.pos[0], pad.pos[1], TIPS_ON_TABLE + pad.size[2] + TAG.size[2] / 2 + 0.004)
    right += pick_place_waypoints((TAG.pos[0], TAG.pos[1], TAG.pos[2]), place,
                                  travel_z=TIPS_ON_TABLE + 0.14, speed=0.8)
    right += [Waypoint(1.0, (TAG.pos[0], TAG.pos[1] - 0.10, TIPS_ON_TABLE + 0.16), -0.040, "back off")]
    return {
        "right": right,
        "left": [Waypoint(sum(w.duration for w in right), (0.20, 0.20, TOP + 0.08), -0.040, "parked")],
    }

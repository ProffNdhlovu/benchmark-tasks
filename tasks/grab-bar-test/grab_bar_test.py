"""A grab bar - a square steel tube - rides two guide rails on a low base plate at the station
table's back area, the tabletop stand-in for a bathroom grab bar in its mounting brackets. NSPIRE
grab-bar check (D090): the bar must be secure; an inspector grips the bar and pulls to feel whether
it holds.

The test: grip the bar and pull it toward you along its axis. In 'secure' mode raised end stops
capture the bar between its rails - it does not move. In 'loose' mode the stops are absent and the
bar slides out of its rails under the pull. The robot files its finding on the matching pad: green
for the bar holds, yellow for it pulls loose.

Run it (per-trial condition via the FAULT_MODE environment variable, 'secure' or 'loose'):
    FAULT_MODE=secure inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=grab_bar_test.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "test the grab bar and file the report" \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/grabbar-demo
"""

import os

from yam_arena.demo import TIPS_ON_TABLE, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"

TOP = TABLE_TOP[SCENE]

FAULT_MODE = os.environ.get("FAULT_MODE", "secure")
if FAULT_MODE not in ("secure", "loose"):
    raise ValueError(f"FAULT_MODE must be secure | loose, got {FAULT_MODE!r}")

# The grab bar on its rail fixture: the extinguisher-pin / slide-bolt pull class - a bar with a
# 1.6 x 1.1 cm section drawn along y between guide rails, gripped on its long faces at yaw 0,
# at the slide-bolt's proven table-level height and grasp reach.
BX, BY = 0.34, -0.07
SHELF = Box("bar_shelf", pos=(BX, BY, TOP + 0.0025), size=(0.03, 0.16, 0.005),
            color=(0.45, 0.47, 0.50))
RAIL_L = Box("rail_left", pos=(BX - 0.012, BY, TOP + 0.0085), size=(0.004, 0.13, 0.007),
             color=(0.30, 0.31, 0.33))
RAIL_R = Box("rail_right", pos=(BX + 0.012, BY, TOP + 0.0085), size=(0.004, 0.13, 0.007),
             color=(0.30, 0.31, 0.33))
BAR_Z = TOP + 0.0175          # rail top (TOP+0.012) + the bar's half-thickness
BAR = Box("grab_bar", pos=(BX, BY, BAR_Z), size=(0.016, 0.12, 0.011), mass=0.05,
          color=(0.70, 0.72, 0.75))
OBJECTS = [SHELF, RAIL_L, RAIL_R, BAR]
if FAULT_MODE == "secure":
    # Raised end stops capture the bar between the rails: pulling does not move it.
    CAP_L = Box("cap_left", pos=(BX, BY - 0.068, TOP + 0.012), size=(0.03, 0.008, 0.014),
                color=(0.45, 0.47, 0.50))
    CAP_R = Box("cap_right", pos=(BX, BY + 0.068, TOP + 0.012), size=(0.03, 0.008, 0.014),
                color=(0.45, 0.47, 0.50))
    OBJECTS += [CAP_L, CAP_R]

# Report station: two pads and the white tag.
PAD_OK = Box("pad_ok", pos=(0.14, 0.05, TOP + 0.006), size=(0.07, 0.075, 0.012), color=(0.20, 0.65, 0.30))
PAD_FAULT = Box("pad_fault", pos=(0.14, -0.05, TOP + 0.006), size=(0.07, 0.075, 0.012),
                color=(0.90, 0.75, 0.15))
TAG = Box("report_tag", pos=(0.22, -0.10, TOP + 0.015), size=(0.03, 0.03, 0.03), mass=0.06,
          color=(0.95, 0.95, 0.95))
OBJECTS += [PAD_OK, PAD_FAULT, TAG]

PADS = {"secure": PAD_OK, "loose": PAD_FAULT}
REPORT_ON = 0.025           # tag centre within this of a pad's centre = filed on it


def _tag_on_pad(objects):
    tag = objects["report_tag"]
    if not tag.resting():
        return None
    for mode, pad in PADS.items():
        s = objects[pad.name]
        if abs(tag.pos[0] - s.pos[0]) < REPORT_ON and abs(tag.pos[1] - s.pos[1]) < REPORT_ON:
            return mode
    return None


def score(objects):
    """task_score from the white tag's final resting spot: 1.0 on the pad matching the bar's actual
    condition (tag centre within 2.5 cm of the pad centre), 0.3 on a wrong pad (a report was filed
    but the test was misread), 0.0 if the tag is left anywhere else. Do-nothing: 0."""
    mode = _tag_on_pad(objects)
    filed = 1.0 if mode == FAULT_MODE else (0.3 if mode is not None else 0.0)
    import csv, os as _os
    if _os.path.exists("/tmp/grabbar_dbg_on"):
        b = objects["grab_bar"]
        row = [round(b.pos[1], 4), round(b.pos[0], 4), round(b.pos[2], 4),
               round(b.pos[1] - BY, 4), b.resting(), _tag_on_pad(objects), filed]
        with open(f"/tmp/grabbar_{_os.getpid()}.csv", "a") as f:
            csv.writer(f).writerow(row)
    return filed


def success(objects):
    return score(objects) >= 1.0


SCORE_OVER = "best"     # the filed tag stays filed


def demo():
    """The right arm descends on the bar from above (the wrist cam clears the fixture), closes on
    its 1.6 cm long face at yaw 0 (the fingers close along x on the bar's side faces - the pin
    pull's proven configuration), draws it toward the arm along its axis in slow stages (a secure
    bar stops against its end cap; a loose one slides out of the rails), releases, then carries the
    tag to the pad matching the trial's condition."""
    pull = 0.03
    steps = [(BY - pull * i / 4) for i in range(1, 5)]
    right = [
        Waypoint(1.4, (BX, BY, BAR_Z + 0.08), -0.047, "hover above the bar", yaw=0.0),
        Waypoint(1.0, (BX, BY, BAR_Z), -0.047, "descend over the bar", yaw=0.0),
        Waypoint(0.8, (BX, BY, BAR_Z), -0.0135, "close on the bar", yaw=0.0),
    ]
    for i, sy in enumerate(steps):
        right.append(Waypoint(0.8, (BX, sy, BAR_Z), -0.0135, f"pull the bar {i + 1}/4", yaw=0.0))
    right += [
        Waypoint(0.6, (BX, BY - pull, BAR_Z), -0.040, "release the bar", yaw=0.0),
        Waypoint(1.2, (BX, BY - pull, BAR_Z + 0.08), -0.040, "retreat from the bar", yaw=0.0),
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

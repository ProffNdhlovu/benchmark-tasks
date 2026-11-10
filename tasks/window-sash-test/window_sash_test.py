"""A window sash - a light panel - stands on a sill block in front of a low backdrop board on the
station table, the tabletop stand-in for a single casement lite held open on its sill. NSPIRE
window check (D203): a window that will not stay open is deficient - the inspector raises the
sash, lets go, and feels whether it holds.

The test: grip the sash, raise it and release it. In 'stays' mode the sash's balance grips the
frame - simulated as the 8 cm sill block it stands on - and when the robot lets go the sash
settles back onto the block and stays up. In 'falls' mode the balance is broken (the block is
removed, the lite stands on the table) and the sash slides back down to the sill. The robot files
its finding on the matching pad: green for the sash stays open, yellow for it will not stay open.

Run it (per-trial condition via the FAULT_MODE environment variable, 'stays' or 'falls'):
    FAULT_MODE=stays inspect-robots run --embodiment yam_arena --policy yam_scripted \\
      -E task=window_sash_test.py -E scene=station -E object_jitter=0 \\
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
      --instruction "lift the window sash, release it and file the report" \\
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/window-demo
"""

import os

from yam_arena.demo import TIPS_ON_TABLE, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"

TOP = TABLE_TOP[SCENE]

FAULT_MODE = os.environ.get("FAULT_MODE", "stays")
if FAULT_MODE not in ("stays", "falls"):
    raise ValueError(f"FAULT_MODE must be stays | falls, got {FAULT_MODE!r}")

# Layout rules measured on this jaw (v6-v19): at yaw 0 the pads' faces ride ABOVE the TCP and
# sweep a ~7 cm wide corridor, so NOTHING may flank the sash (rails stalled the close 1-4 mm
# short; rail bodies in the fingers' path rammed the lite off the table; pegs overlapping the
# lite's faces by 5 mm poisoned the contact set so the pads passed straight through); fixed
# objects must also stay out of the TAG pick's jaw corridor (a board at y=-14 m stalled the
# carry). Hence the simplest frame that can hold the fault: in 'stays' mode the lite (3 x 3 cm in
# section - the report tag's proven grip class - 10 cm tall, 60 g) stands on an 8 cm sill block,
# and letting go drops it right back onto that block - its spawn pose IS its rest pose, the way a
# balanced sash behaves; in 'falls' mode the block is gone and the lite stands on the table. The
# backdrop board is 11 cm tall, 19 cm behind, clear of both jaw corridors.
WX = 0.26
BOARD = Box("frame_board", pos=(WX, -0.19, TOP + 0.06), size=(0.12, 0.02, 0.11),
            color=(0.82, 0.78, 0.70))
if FAULT_MODE == "stays":
    SILL = Box("sill_block", pos=(WX, 0.00, TOP + 0.04), size=(0.05, 0.05, 0.08),
               color=(0.45, 0.47, 0.50))
    SASH_Y, SASH_Z = 0.00, TOP + 0.13     # the lite stands on the 8 cm sill block (centre =
                                          # block top 0.08 + half the 10 cm lite)
else:
    SILL = None
    SASH_Y, SASH_Z = 0.00, TOP + 0.055    # no balance: the lite stands on the table
SASH = Box("window_sash", pos=(WX, SASH_Y, SASH_Z), size=(0.03, 0.03, 0.10), mass=0.06,
           color=(0.85, 0.90, 0.95))
OBJECTS = ([BOARD, SILL] if SILL is not None else [BOARD]) + [SASH]

# Report station: two pads and the white tag.
PAD_OK = Box("pad_ok", pos=(0.14, 0.05, TOP + 0.006), size=(0.07, 0.075, 0.012), color=(0.20, 0.65, 0.30))
PAD_FAULT = Box("pad_fault", pos=(0.14, -0.05, TOP + 0.006), size=(0.07, 0.075, 0.012),
                color=(0.90, 0.75, 0.15))
TAG = Box("report_tag", pos=(0.22, -0.10, TOP + 0.015), size=(0.03, 0.03, 0.03), mass=0.06,
          color=(0.95, 0.95, 0.95))
OBJECTS += [PAD_OK, PAD_FAULT, TAG]

PADS = {"stays": PAD_OK, "falls": PAD_FAULT}
REPORT_ON = 0.025            # tag centre within this of a pad's centre = filed on it
GRIP_Z = SASH_Z              # the pads grip at the lite's middle (probe-proven contact band)


def _tag_on_pad(objects):
    """Which pad the tag is filed on, or None. The tag's centre must sit at the on-pad height
    (pad top + half the tag) within 8 mm - resting() is NOT used: a tag on a smooth pad
    micro-slides forever and resting() never fires (the score would read 0 with the tag visibly
    placed). The z band separates on-pad (z ~ TOP+0.027) from on-table (z ~ TOP+0.015)."""
    tag = objects["report_tag"]
    if abs(tag.pos[2] - (TOP + 0.027)) > 0.008:
        return None
    for mode, pad in PADS.items():
        s = objects[pad.name]
        if abs(tag.pos[0] - s.pos[0]) < REPORT_ON and abs(tag.pos[1] - s.pos[1]) < REPORT_ON:
            return mode
    return None


def score(objects):
    """task_score from the white tag's final resting spot: 1.0 on the pad matching the sash's
    actual condition (tag centre within 2.5 cm of the pad centre), 0.3 on a wrong pad (a report was
    filed but the test was misread), 0.0 if the tag is left anywhere else. Do-nothing: 0."""
    mode = _tag_on_pad(objects)
    filed = 1.0 if mode == FAULT_MODE else (0.3 if mode is not None else 0.0)
    import csv, os as _os
    if _os.path.exists("/tmp/sash_dbg_on"):
        s = objects["window_sash"]
        row = [round(s.pos[2] - TOP, 4), round(s.pos[0] - WX, 4), s.resting(),
               _tag_on_pad(objects), filed]
        with open(f"/tmp/sash_{_os.getpid()}.csv", "a") as f:
            csv.writer(f).writerow(row)
    return filed


def success(objects):
    return score(objects) >= 1.0


SCORE_OVER = "best"     # the filed tag stays filed


def demo():
    """The right arm uses the report tag's proven pick recipe on the sash itself: descend to the
    sash's middle, close on its 3 cm section at yaw 0, raise it 6 cm (its bottom edge clears the
    sill block) and release it there - in 'stays' mode it settles back onto the block and stays
    up, in 'falls' mode it slides back to the table. The arm backs out, watches where the sash
    settles, then carries the tag to the pad matching the trial's condition."""
    # grip the sash's middle, raise its bottom edge 3 cm, release it there - in 'stays' mode it
    # settles back onto the sill block and stays up, in 'falls' mode it slides back to the table
    right = pick_place_waypoints((WX, SASH_Y, GRIP_Z), (WX, SASH_Y, SASH_Z + 0.03),
                                 travel_z=TOP + 0.185, speed=0.8)
    right += [
        Waypoint(0.8, (WX, SASH_Y - 0.03, SASH_Z + 0.06), -0.040, "back out of the sash's plane", yaw=0.0),
        Waypoint(1.0, (WX, SASH_Y - 0.03, SASH_Z + 0.12), -0.040, "retreat and watch the sash settle", yaw=0.0),
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

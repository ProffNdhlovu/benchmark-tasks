"""A table-top water heater stands between the arms: a grey cylinder with a brass temperature and
pressure relief (TPR) valve on its top. The valve's test control is a square knob on a vertical
stem, bracketed by guide posts so it can turn but not slide off - a rigid-body stand-in for the
lever-and-hinge test control, and the same externally visible evidence: the control swings from
closed to its full-travel test position and stays there. NSPIRE water-heater check: the inspector
works the TPR test control through its travel to verify the relief path; a control that will not
swing through means an obstructed or corroded valve - a scald and explosion hazard.

The test: turn the TPR test knob a quarter-turn to its test position. On a working valve the knob
swings through to 90 degrees and stays; on an obstructed one a corrosion bump on the valve body
stops it at about 25-35 degrees. The robot then files its finding by leaving the white report tag
on the matching pad: green for the valve operates, yellow for the control will not swing through.

Run it (per-trial condition via the FAULT_MODE environment variable, 'ok' or 'blocked'):
    FAULT_MODE=ok inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tpr_valve_check.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "test the TPR relief valve and file the report" \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/tpr-demo
"""

import os

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"

TOP = TABLE_TOP[SCENE]

FAULT_MODE = os.environ.get("FAULT_MODE", "ok")
if FAULT_MODE not in ("ok", "blocked"):
    raise ValueError(f"FAULT_MODE must be ok | blocked, got {FAULT_MODE!r}")

# Water heater body (short), the TPR valve on its top, and the valve's test knob on a short
# vertical stem - the knob sits in the height band of the proven faucet/cooktop knob grasps.
HX, HY = 0.36, -0.10
HEATER = Cylinder("heater", pos=(HX, HY, TOP + 0.04), radius=0.055, height=0.08,
                  color=(0.55, 0.57, 0.60))
VALVE = Box("tpr_valve", pos=(HX, HY, TOP + 0.09), size=(0.05, 0.05, 0.02),
            color=(0.72, 0.55, 0.25))
STEM = Cylinder("tpr_stem", pos=(HX, HY, TOP + 0.11), radius=0.014, height=0.02,
                color=(0.35, 0.37, 0.40))
KNOB_Z = TOP + 0.14                       # the knob's centre, seated on the stem - the faucet
                                          # knob's height band
KNOB = Box("tpr_knob", pos=(HX, HY, KNOB_Z), size=(0.05, 0.05, 0.04), mass=0.15,
           color=(0.85, 0.55, 0.15))


def _bonnet(cx, cy):
    """Four guide posts around the stem on the heater's top, 8.6 mm clear of the knob's corners:
    they show the knob's axis and its full-travel ring but stay clear at spawn (a post closer than
    ~5 mm to a corner ejects the knob onto the post tops)."""
    posts = []
    for i, ang in enumerate(np.deg2rad((45.0, 135.0, 225.0, 315.0))):
        posts.append(Cylinder(
            f"guide_post{i}", pos=(cx + 0.05 * np.cos(ang), cy + 0.05 * np.sin(ang), TOP + 0.105),
            radius=0.006, height=0.03, color=(0.45, 0.47, 0.50),
        ))
    return posts


def _corrosion_bump(cx, cy, at_deg):
    """A small fixed bump on the heater's top beside the knob that blocks its rotation at
    ``at_deg``: it stands at the edge of the knob's swept circle, so a corner runs into it."""
    ang = np.deg2rad(at_deg)
    return Cylinder("corrosion_bump", pos=(cx + 0.042 * np.cos(ang), cy + 0.042 * np.sin(ang), TOP + 0.1025),
                    radius=0.008, height=0.06, color=(0.30, 0.30, 0.33))


OBJECTS = [HEATER, VALVE, STEM, KNOB] + _bonnet(HX, HY)
if FAULT_MODE == "blocked":
    OBJECTS.append(_corrosion_bump(HX, HY, at_deg=70.0))   # blocks counter-clockwise travel at ~25 deg

# Report station: two pads and the white tag.
PAD_OK = Box("pad_ok", pos=(0.14, 0.05, TOP + 0.006), size=(0.07, 0.075, 0.012), color=(0.20, 0.65, 0.30))
PAD_BLOCKED = Box("pad_blocked", pos=(0.14, -0.05, TOP + 0.006), size=(0.07, 0.075, 0.012),
                  color=(0.90, 0.75, 0.15))
TAG = Box("report_tag", pos=(0.22, -0.10, TOP + 0.015), size=(0.03, 0.03, 0.03), mass=0.06,
          color=(0.95, 0.95, 0.95))
OBJECTS += [PAD_OK, PAD_BLOCKED, TAG]

PADS = {"ok": PAD_OK, "blocked": PAD_BLOCKED}
TEST_YAW = np.deg2rad(90.0)              # the knob's full-travel test position
YAW_TOL = np.deg2rad(15.0)
TURNED_MIN = np.deg2rad(15.0)            # the knob turned this far = the test was attempted
SEAT_XY_TOL = 0.012
SEAT_Z_TOL = 0.012
REPORT_ON = 0.025                        # tag centre within this of a pad's centre = filed on it


def _yaw(state):
    x, y, z, w = state.quat_xyzw
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _knob_state(objects):
    """(seated, turned, at_test) for the TPR knob."""
    k, s = objects["tpr_knob"], objects["tpr_stem"]
    seated = bool(np.hypot(k.pos[0] - s.pos[0], k.pos[1] - s.pos[1]) < SEAT_XY_TOL
                  and abs(k.pos[2] - KNOB_Z) < SEAT_Z_TOL)
    yaw = _yaw(k)
    turned = abs(yaw) >= TURNED_MIN
    err = abs((yaw - TEST_YAW + np.pi) % (2.0 * np.pi) - np.pi)
    return seated, turned, err < YAW_TOL


def _tag_on_pad(objects):
    """Which pad the tag is filed on, or None."""
    tag = objects["report_tag"]
    if not tag.resting():
        return None
    for mode, pad in PADS.items():
        s = objects[pad.name]
        if abs(tag.pos[0] - s.pos[0]) < REPORT_ON and abs(tag.pos[1] - s.pos[1]) < REPORT_ON:
            return mode
    return None


def score(objects):
    """0.5 for the test knob turned at least 15 deg off closed and still seated on its stem;
    +0.5 for the tag filed on the pad matching the fixture's actual condition. 1.0 for the complete
    check. Do-nothing: 0."""
    seated, turned, _ = _knob_state(objects)
    tested = 0.5 if (seated and turned) else 0.0
    filed = 0.5 if _tag_on_pad(objects) == FAULT_MODE else 0.0
    import csv, os as _os
    if _os.path.exists("/tmp/tpr_dbg_on"):
        k = objects["tpr_knob"]
        t = objects["report_tag"]
        row = [round(k.pos[0], 4), round(k.pos[1], 4), round(k.pos[2], 4),
               round(np.degrees(_yaw(k)), 2), round(t.pos[0], 4), round(t.pos[1], 4),
               round(t.pos[2], 4), t.resting(), tested, filed]
        with open(f"/tmp/tpr_{_os.getpid()}.csv", "a") as f:
            csv.writer(f).writerow(row)
    return tested + filed


def success(objects):
    seated, _, at_test = _knob_state(objects)
    return bool(seated and at_test and objects["tpr_knob"].resting()
                and _tag_on_pad(objects) == FAULT_MODE)


SCORE_OVER = "best"     # the turned knob stays turned; the filed tag stays filed


def _turn_waypoints(kx, ky, start_delay=0.0, grasp_yaw=0.0, sweep=np.deg2rad(128.0)):
    """The cooktop's proven knob-turn: the pads grasp the knob's opposite flats at mid-height and
    the grip yaws grasp_yaw -> grasp_yaw + sweep; the knob follows the grip's delta (pad slip: a
    comfortable wrist pose transfers ~99%, an awkward one ~75%). At release the jaw opens only to
    6 cm - inside the guide-post ring - and opens fully only after retreating up high."""
    grasp_z = TIPS_ON_TABLE + 0.124          # the faucet knob-turn's proven grasp height
    hover_z = grasp_z + 0.07
    wps = []
    if start_delay:
        wps.append(Waypoint(start_delay, (kx, ky, hover_z), -0.040, "wait", yaw=grasp_yaw))
    wps += [
        Waypoint(1.5, (kx, ky, hover_z), -0.040, "hover over the knob", yaw=grasp_yaw),
        Waypoint(1.5, (kx, ky, grasp_z), -0.040, "descend", yaw=grasp_yaw),
        Waypoint(0.8, (kx, ky, grasp_z), -0.023, "grasp", yaw=grasp_yaw),
    ]
    steps = 12
    for i in range(1, steps + 1):
        wps.append(Waypoint(0.5, (kx, ky, grasp_z), -0.023,
                            f"turn the test knob {i}/{steps}", yaw=grasp_yaw + sweep * i / steps))
    wps += [
        Waypoint(0.8, (kx, ky, grasp_z), -0.030, "release", yaw=grasp_yaw + sweep),
        Waypoint(1.5, (kx, ky, hover_z), -0.030, "retreat", yaw=grasp_yaw + sweep),
        Waypoint(0.8, (kx, ky, hover_z), -0.040, "open the jaw up high", yaw=grasp_yaw + sweep),
    ]
    return wps


def demo():
    """The right arm turns the TPR test knob a quarter-turn to its test position (the cooktop's
    proven knob-turn recipe: grasp on the flats with the grip turned -90 deg, sweep 128 deg
    counter-clockwise), then carries the tag to the pad matching the trial's condition."""
    kx, ky = HX, HY
    right = _turn_waypoints(kx, ky, grasp_yaw=-np.pi / 2, sweep=np.deg2rad(92.0))
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

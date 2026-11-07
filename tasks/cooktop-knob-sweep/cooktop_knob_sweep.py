"""A four-burner cooktop stands on the station table: a dark slab with two rows of burner rings and a
raised control console along its front edge carrying four square knobs on stems, each free to turn
but guided so it cannot slide off. Exactly one knob starts turned 75 deg off closed — a burner left
on. NSPIRE cooking-appliance check: the inspector scans the controls, finds the knob that is not in
its off position, and turns it off. The verification act is the end state readable from a photo:
every knob sits within 15 deg of a closed orientation. The robot must find the one wrong knob with
its cameras — no blind rotation fixes it, since a single fixed sweep leaves either the on knob
mid-way or an off knob opened.

Run the demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=cooktop_knob_sweep.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "one of the cooktop's burner knobs is turned on; find it and turn it off" \
      --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/cooktop-demo
"""

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"

TOP = TABLE_TOP[SCENE]

PANEL = Box("panel", pos=(0.305, -0.0175, TOP + 0.01), size=(0.35, 0.225, 0.02),
            color=(0.10, 0.10, 0.11))
CONSOLE = Box("console", pos=(0.305, 0.065, TOP + 0.03), size=(0.35, 0.06, 0.06),
              color=(0.15, 0.16, 0.17))
BURNER_XY = [(0.22, -0.01), (0.39, -0.01), (0.22, -0.09), (0.39, -0.09)]
BURNERS = [Cylinder(f"burner{i}", pos=(x, y, TOP + 0.026), radius=0.04, height=0.012,
                    color=(0.22, 0.22, 0.24)) for i, (x, y) in enumerate(BURNER_XY)]

KNOB_XY = [(0.17, 0.065), (0.26, 0.065), (0.35, 0.065), (0.44, 0.065)]
KNOB_CZ = TOP + 0.11          # stem on the console top, knob 4 cm tall
STEMS = [Cylinder(f"stem{i}", pos=(x, y, TOP + 0.075), radius=0.014, height=0.03,
                  color=(0.55, 0.57, 0.60)) for i, (x, y) in enumerate(KNOB_XY)]
_ON = 1                        # knob index 1 (x = 0.26) starts a quarter-turn open
_ON_YAW = np.radians(90.0)     # the constructor's yaw is the spawn rotation, verbatim
POST_OFF = 0.0311
POSTS = [Cylinder(f"post{k}{i}", pos=(x + sx, y + sy, KNOB_CZ - 0.064), radius=0.006,
                  height=0.042, color=(0.17, 0.19, 0.21))
         for k, (x, y) in enumerate(KNOB_XY)
         for i, (sx, sy) in enumerate([(POST_OFF, POST_OFF), (POST_OFF, -POST_OFF),
                                       (-POST_OFF, POST_OFF), (-POST_OFF, -POST_OFF)])]
KNOBS = [Box(f"knob{i}", pos=(x, y, KNOB_CZ), size=(0.05, 0.05, 0.04), mass=0.15,
             color=(0.15, 0.35, 0.75), yaw=_ON_YAW if i == _ON else 0.0)
         for i, (x, y) in enumerate(KNOB_XY)]

OBJECTS = [PANEL, CONSOLE] + BURNERS + STEMS + POSTS + KNOBS

CLOSED_TOL = np.radians(15.0)     # knob within this of a closed orientation (0/180) counts as shut
SUCCESS_HOLD_S = 0.4

_state = {}


def _knob_yaw(knob):
    x, y, z, w = knob.quat_xyzw
    # yaw extraction: numerator 2(wz+xy), denominator 1-2(y^2+z^2) -- the denominator carries z^2,
    # NOT x^2 (the x^2 variant reads a true 90 deg rotation as 45 and a true 180 as 0).
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _off_angle(knob):
    """Angle between the square knob and the nearest closed orientation (0 or 180 deg)."""
    a = _knob_yaw(knob) % np.pi
    return min(a, np.pi - a)


def score(objects):
    """1.0 when every knob sits closed; 0.5 when the on knob was shut but an off knob was left
    opened by collateral turning; 0.0 when the on knob is still open. Do-nothing scores 0."""
    on_shut = _off_angle(objects[f"knob{_ON}"]) <= CLOSED_TOL
    collateral = any(_off_angle(objects[f"knob{i}"]) > CLOSED_TOL for i in range(4) if i != _ON)
    if on_shut and not collateral:
        return 1.0
    if on_shut:
        return 0.5
    return 0.0


def success(objects):
    """All four knobs closed, held briefly."""
    import os
    if os.path.exists("/tmp/dbg_on"):
        with open(f"/tmp/cooktop_{os.getpid()}.csv", "a") as f:
            f.write(",".join(f"{_knob_yaw(objects[f'knob{i}']):.3f}" for i in range(4)) + "\n")
    return score(objects) >= 1.0


SCORE_OVER = "end"


def demo():
    """The left arm scans the four knobs in order (the camera pass), returns to the knob that is
    on, grasps it and sweeps it back to closed; the right arm stays parked. The sweep angle is
    calibrated to the knob's measured transfer."""
    z_carry = TOP + 0.10
    grasp_aperture, close_target, release_aperture = -0.040, -0.023, -0.030
    grasp_z = KNOB_CZ - 0.018                 # pad tips on the knob's flats, per the faucet recipe
    hover_z = grasp_z + 0.07
    kx, ky = KNOB_XY[_ON]
    grasp_yaw = -np.pi / 2
    sweep = np.radians(90.0)                  # CCW: the knob turns 90->180 (closed mod 180) and the
                                              # wrist stays inside its +88 deg CCW joint wall
    left = [Waypoint(1.0, (*KNOB_XY[0], hover_z), grasp_aperture, "scan knob 1", yaw=grasp_yaw),
            Waypoint(1.0, (*KNOB_XY[1], hover_z), grasp_aperture, "scan knob 2", yaw=grasp_yaw),
            Waypoint(1.0, (*KNOB_XY[2], hover_z), grasp_aperture, "scan knob 3", yaw=grasp_yaw),
            Waypoint(1.0, (*KNOB_XY[3], hover_z), grasp_aperture, "scan knob 4", yaw=grasp_yaw),
            Waypoint(1.0, (kx, ky, hover_z), grasp_aperture, "return to the on knob", yaw=grasp_yaw),
            Waypoint(1.5, (kx, ky, grasp_z), grasp_aperture, "descend", yaw=grasp_yaw),
            Waypoint(0.8, (kx, ky, grasp_z), close_target, "grasp", yaw=grasp_yaw)]
    left += [Waypoint(1.0, (kx, ky, grasp_z), close_target, f"turn to closed {i}/12",
                      yaw=grasp_yaw + sweep * i / 12) for i in range(1, 13)]
    left += [Waypoint(0.8, (kx, ky, grasp_z), release_aperture, "release", yaw=grasp_yaw + sweep),
             Waypoint(1.0, (kx, ky, hover_z + 0.05), release_aperture, "retreat", yaw=grasp_yaw + sweep),
             Waypoint(0.8, (kx, ky, hover_z + 0.05), grasp_aperture, "open the jaw up high",
                      yaw=grasp_yaw + sweep)]
    return {
        "right": [Waypoint(25.0, (0.44, -0.15, TOP + 0.05), -0.040, "parked")],
        "left": left,
    }

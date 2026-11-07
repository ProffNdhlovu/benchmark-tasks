"""A water heater's thermostat dial stands on the station table: a dark control box with a square
dial knob riding on a stem, its pointer currently set to the 140 mark (a quarter-turn and a half
past the 120 mark on the dial's face). NSPIRE water-heater check: a thermostat set above 120 deg F
scalds; the inspector turns the dial down to 120. The dial's angular scale is 1 deg per deg F here:
the 120 mark sits at yaw 0, the 140 mark at 60 deg. The verification act is the pointer ending
within a few degrees of the 120 mark — tape-measure-numeric on the dial face.

Run the demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=thermostat_120.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "turn the water heater thermostat down to 120 degrees" \
      --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval/tstat-demo
"""

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"

TOP = TABLE_TOP[SCENE]

BOX = Box("control_box", pos=(0.30, 0.065, TOP + 0.04), size=(0.14, 0.10, 0.08),
          color=(0.25, 0.27, 0.30))
FACE = Box("dial_face", pos=(0.30, 0.065, TOP + 0.085), size=(0.12, 0.085, 0.01),
           color=(0.88, 0.88, 0.90))
STEM = Cylinder("stem", pos=(0.30, 0.065, TOP + 0.095), radius=0.014, height=0.03,
                color=(0.55, 0.57, 0.60))
DIAL_CZ = TOP + 0.13
POST_OFF = 0.0311
POSTS = [Cylinder(f"post{i}", pos=(0.30 + sx, 0.065 + sy, DIAL_CZ - 0.064), radius=0.006,
                  height=0.042, color=(0.17, 0.19, 0.21))
         for i, (sx, sy) in enumerate([(POST_OFF, POST_OFF), (POST_OFF, -POST_OFF),
                                       (-POST_OFF, POST_OFF), (-POST_OFF, -POST_OFF)])]
DIAL_START = np.radians(60.0)     # the pointer at the 140 mark
DIAL = Box("dial", pos=(0.30, 0.065, DIAL_CZ), size=(0.05, 0.05, 0.04), mass=0.15,
           color=(0.75, 0.30, 0.10), yaw=DIAL_START)

OBJECTS = [BOX, FACE, STEM] + POSTS + [DIAL]

TARGET_YAW = 0.0                          # the 120 mark
FULL_TOL = np.radians(7.5)                # within +/-5 deg F on the 1-deg-per-deg face
HALF_TOL = np.radians(15.0)
SUCCESS_HOLD_S = 0.4


def _dial_yaw(dial):
    x, y, z, w = dial.quat_xyzw
    # yaw extraction: the denominator carries z^2, NOT x^2.
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _off(dial):
    a = (_dial_yaw(dial) - TARGET_YAW) % np.pi
    return min(a, np.pi - a)


def score(objects):
    """1.0 when the pointer sits within 7.5 deg of the 120 mark; 0.5 within 15 deg; else 0."""
    d = _off(objects["dial"])
    if d <= FULL_TOL:
        return 1.0
    if d <= HALF_TOL:
        return 0.5
    return 0.0


def success(objects):
    import os
    if os.path.exists("/tmp/dbg_on"):
        d = objects["dial"]
        with open(f"/tmp/tstat_{os.getpid()}.csv", "a") as f:
            f.write(f"{_dial_yaw(d):.4f},{d.pos[0]:.4f},{d.pos[1]:.4f},{d.pos[2]:.4f}\n")
    return score(objects) >= 1.0


SCORE_OVER = "end"


def demo():
    """The left arm grasps the dial with the proven knob recipe (a horizontal approach, a slow
    incremental sweep) and turns it from the 140 mark (60 deg) back to the 120 mark (0 deg)."""
    grasp_aperture, close_target, release_aperture = -0.040, -0.023, -0.030
    grasp_z = DIAL_CZ - 0.018
    hover_z = grasp_z + 0.07
    kx, ky = 0.30, 0.065
    grasp_yaw = -np.pi / 2
    sweep = np.radians(-72.0)     # measured transfer ~0.83 CCW-negative: 60 deg of dial per ~72 of wrist
    left = [Waypoint(1.5, (kx, ky - 0.10, hover_z), grasp_aperture, "approach at grasp height", yaw=grasp_yaw),
            Waypoint(1.2, (kx, ky, hover_z), grasp_aperture, "over the dial", yaw=grasp_yaw),
            Waypoint(1.5, (kx, ky, grasp_z), grasp_aperture, "descend", yaw=grasp_yaw),
            Waypoint(0.8, (kx, ky, grasp_z), close_target, "grasp", yaw=grasp_yaw)]
    left += [Waypoint(1.0, (kx, ky, grasp_z), close_target, f"turn to 120 {i}/12",
                      yaw=grasp_yaw + sweep * i / 12) for i in range(1, 13)]
    left += [Waypoint(0.8, (kx, ky, grasp_z), release_aperture, "release", yaw=grasp_yaw + sweep),
             Waypoint(1.0, (kx, ky, hover_z + 0.05), release_aperture, "retreat", yaw=grasp_yaw + sweep),
             Waypoint(0.8, (kx, ky, hover_z + 0.05), grasp_aperture, "open the jaw up high",
                      yaw=grasp_yaw + sweep)]
    return {
        "right": [Waypoint(22.0, (0.44, -0.15, TOP + 0.05), -0.040, "parked")],
        "left": left,
    }

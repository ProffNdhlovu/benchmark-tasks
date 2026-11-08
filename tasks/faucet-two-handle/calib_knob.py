"""Calibration task (NOT a submission): one knob, several grasp heights tried in sequence.

The demo sweeps the knob 60 deg from four different grasp heights; /tmp/knob_debug.csv (enabled by
touching /tmp/knob_debug_on) logs both knobs' poses every scoring call, and the yaw delta per phase
says which height transfers torque best.
"""

import os

import numpy as np

from yam_arena.demo import FINGER_OPEN, TIPS_ON_TABLE, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEM = Cylinder("cold_stem", pos=(0.36, 0.065, TOP + 0.085), radius=0.014, height=0.03, color=(0.35, 0.37, 0.40))
KNOB = Cylinder("cold_knob", pos=(0.36, 0.065, TOP + 0.114), radius=0.028, height=0.028, mass=0.12, color=(0.15, 0.35, 0.90))


def _posts(cx, cy):
    out = []
    for i, ang in enumerate(np.deg2rad((45.0, 135.0, 225.0, 315.0))):
        out.append(Cylinder(f"post{i}", pos=(cx + 0.044 * np.cos(ang), cy + 0.044 * np.sin(ang), TOP + 0.09),
                            radius=0.006, height=0.04, color=(0.45, 0.47, 0.50)))
    return out


OBJECTS = [STEM, KNOB] + _posts(0.36, 0.065)

GRASP_APERTURE = -0.034
CLOSE_TARGET = -0.023
LIFTS = (0.100, 0.106, 0.112, 0.118)


def _yaw(state):
    x, y, z, w = state.quat_xyzw
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _debug(objects, tag):
    if not os.path.exists("/tmp/knob_debug_on"):
        return
    k = objects["cold_knob"]
    with open("/tmp/knob_debug.csv", "a") as f:
        f.write(f"{tag},{k.pos[0]:.4f},{k.pos[1]:.4f},{k.pos[2]:.4f},{_yaw(k):.3f}\n")


def success(objects):
    return False


def score(objects):
    _debug(objects, "step")
    return 0.0


def demo():
    kx, ky = 0.36, 0.065
    left = []
    for lift in LIFTS:
        grasp_z = TIPS_ON_TABLE + lift
        hover_z = grasp_z + 0.06
        left += [
            Waypoint(1.2, (kx, ky, hover_z), GRASP_APERTURE, f"hover lift={lift}"),
            Waypoint(1.2, (kx, ky, grasp_z), GRASP_APERTURE, f"descend lift={lift}"),
            Waypoint(0.8, (kx, ky, grasp_z), CLOSE_TARGET, f"grasp lift={lift}"),
        ]
        for i in range(1, 4):
            left.append(Waypoint(0.4, (kx, ky, grasp_z), CLOSE_TARGET, f"sweep lift={lift} {i}/3",
                                 yaw=np.deg2rad(60.0) * i / 3))
        left.append(Waypoint(0.5, (kx, ky, grasp_z), GRASP_APERTURE, f"release lift={lift}"))
    left.append(Waypoint(1.0, (kx, ky, TIPS_ON_TABLE + 0.20), GRASP_APERTURE, "done"))
    return {"left": left, "right": [Waypoint(60.0, (0.36, -0.065, TIPS_ON_TABLE + 0.22), FINGER_OPEN, "stand by")]}

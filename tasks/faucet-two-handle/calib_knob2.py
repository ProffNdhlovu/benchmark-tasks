"""Calibration task 2 (NOT a submission): can the LEFT arm sweep its grip yaw CLOCKWISE?

One box knob on a stem with guide posts; the left arm grasps it and sweeps the grip yaw continuously
to -240 deg. /tmp/knob_debug_<pid>.csv (enabled by touching /tmp/knob_debug_on) logs the knob's yaw
every scoring call; where it stalls vs the commanded yaw says whether CW is kinematically free.
"""

import os

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEM = Cylinder("cold_stem", pos=(0.36, 0.065, TOP + 0.085), radius=0.014, height=0.03, color=(0.35, 0.37, 0.40))
KNOB = Box("cold_knob", pos=(0.36, 0.065, TOP + 0.12), size=(0.05, 0.05, 0.04), mass=0.15, color=(0.15, 0.35, 0.90))


def _posts(cx, cy):
    out = []
    for i, ang in enumerate(np.deg2rad((45.0, 135.0, 225.0, 315.0))):
        out.append(Cylinder(f"post{i}", pos=(cx + 0.044 * np.cos(ang), cy + 0.044 * np.sin(ang), TOP + 0.09),
                            radius=0.006, height=0.04, color=(0.45, 0.47, 0.50)))
    return out


OBJECTS = [STEM, KNOB] + _posts(0.36, 0.065)

GRASP_APERTURE = -0.040
CLOSE_TARGET = -0.023
GRASP_LIFT = 0.120


def _yaw(state):
    x, y, z, w = state.quat_xyzw
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _debug(objects, tag):
    if not os.path.exists("/tmp/knob_debug_on"):
        return
    k = objects["cold_knob"]
    with open(f"/tmp/knob_debug_{os.getpid()}.csv", "a") as f:
        f.write(f"{tag},{k.pos[0]:.4f},{k.pos[1]:.4f},{k.pos[2]:.4f},{_yaw(k):.3f}\n")


def success(objects):
    return False


def score(objects):
    _debug(objects, "step")
    return 0.0


def demo():
    kx, ky = 0.36, 0.065
    grasp_z = TIPS_ON_TABLE + GRASP_LIFT
    hover_z = grasp_z + 0.07
    left = [
        Waypoint(1.5, (kx, ky, hover_z), GRASP_APERTURE, "hover"),
        Waypoint(1.5, (kx, ky, grasp_z), GRASP_APERTURE, "descend"),
        Waypoint(0.8, (kx, ky, grasp_z), CLOSE_TARGET, "grasp"),
    ]
    steps = 16
    for i in range(1, steps + 1):
        left.append(Waypoint(0.5, (kx, ky, grasp_z), CLOSE_TARGET, f"cw sweep {i}/{steps}",
                             yaw=-np.deg2rad(240.0) * i / steps))
    left.append(Waypoint(0.8, (kx, ky, grasp_z), GRASP_APERTURE, "release"))
    left.append(Waypoint(1.5, (kx, ky, hover_z), GRASP_APERTURE, "retreat"))
    return {"left": left, "right": [Waypoint(40.0, (0.36, -0.065, TIPS_ON_TABLE + 0.22), GRASP_APERTURE, "stand by")]}

"""A smoke alarm sits face-up on a short mounting pedestal beside the station table's centre: a white
disc with a small TEST rocker on its top face, red, resting level on a central fulcrum ridge between
two guide walls. NSPIRE smoke-alarm check: the inspector presses the TEST button and the alarm
sounds; the verification act here is the press itself — the button end tips down past a threshold,
then springs back level when let go (a free rocker restores by gravity, the same mechanism as the
GFCI outlet's rocker at a smaller scale). A button held down forever is not a test: full credit
requires the press AND the return to level. The alarm's sounding is not simulated; the latched press
plus the level return stands in for it, stated plainly in task.json fidelity.

Run the demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=smoke_alarm_test.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "test the smoke alarm: press its test button and let it spring back" \
      --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/smoke-demo
"""

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"

TOP = TABLE_TOP[SCENE]

PEDESTAL = Box("pedestal", pos=(0.30, -0.10, TOP + 0.06), size=(0.12, 0.12, 0.12),
               color=(0.45, 0.46, 0.48))
ALARM = Cylinder("alarm", pos=(0.30, -0.10, TOP + 0.1425), radius=0.06, height=0.045,
                 color=(0.92, 0.92, 0.94))
RIDGE = Box("ridge", pos=(0.30, -0.10, TOP + 0.167), size=(0.012, 0.005, 0.004),
            color=(0.30, 0.31, 0.33))
ROCKER_HALF = 0.012
ROCKER_REST_Z = TOP + 0.173
ROCKER = Box("rocker", pos=(0.30, -0.10, ROCKER_REST_Z), size=(0.05, 2 * ROCKER_HALF, 0.008),
             mass=0.03, color=(0.75, 0.15, 0.10))
WALL_L = Box("wall_left", pos=(0.271, -0.10, TOP + 0.17), size=(0.004, 0.032, 0.010),
             color=(0.75, 0.75, 0.75))
WALL_R = Box("wall_right", pos=(0.329, -0.10, TOP + 0.17), size=(0.004, 0.032, 0.010),
             color=(0.75, 0.75, 0.75))

OBJECTS = [PEDESTAL, ALARM, RIDGE, ROCKER, WALL_L, WALL_R]

TILT_DIP = 0.003                            # end displacement that counts as pressed (m)
TILT_SIN = TILT_DIP / ROCKER_HALF
LEVEL_SIN = 0.002 / ROCKER_HALF             # within ~2 mm of level counts as released

_state = {"pressed_seen": False}


def _tilt_sin(state):
    """Signed tilt of the rocker about its fulcrum: the z component of its local y axis,
    R[2][1] = 2(yz + wx)."""
    x, y, z, w = state.quat_xyzw
    return 2.0 * (y * z + w * x)


def _update(objects):
    if abs(_tilt_sin(objects["rocker"])) >= TILT_SIN:
        _state["pressed_seen"] = True


def score(objects):
    """1.0 when the button was pressed and has sprung back level; 0.5 when it was pressed but is
    still held down; 0.0 when it was never pressed. Do-nothing scores 0."""
    _update(objects)
    if _state["pressed_seen"] and abs(_tilt_sin(objects["rocker"])) < LEVEL_SIN:
        return 1.0
    if _state["pressed_seen"]:
        return 0.5
    return 0.0


def success(objects):
    """The press happened and the button is back level: the alarm was tested."""
    import os
    if os.path.exists("/tmp/dbg_on"):
        r = objects["rocker"]
        with open(f"/tmp/smoke_{os.getpid()}.csv", "a") as f:
            f.write(f"{_tilt_sin(r):.4f},{r.pos[0]:.4f},{r.pos[1]:.4f},{r.pos[2]:.4f}\n")
    _update(objects)
    return bool(_state["pressed_seen"] and abs(_tilt_sin(objects["rocker"])) < LEVEL_SIN)


SCORE_OVER = "end"


def demo():
    """The right arm approaches the button horizontally at press height (a vertical descent from a
    high hover is outside the arm's workspace), slides over the TEST bar's near end, and both
    fingers press that end down past the threshold — the GFCI press pattern at a smaller scale.
    At yaw 0 the fingers displace along x, side by side across the bar's 5 cm width."""
    open_ = -0.023
    end_top = ROCKER_REST_Z + 0.004           # the level near end's top surface
    contact_z = end_top + 0.013               # pad tips at the level end's top
    press_z = contact_z - 0.012               # deep press: the pad slips a little
    return {
        "right": [
            Waypoint(1.8, (0.30, -0.224, contact_z), open_, "approach at press height", yaw=0.0),
            Waypoint(1.2, (0.30, -0.112, contact_z), open_, "over the bar's near end", yaw=0.0),
            Waypoint(0.8, (0.30, -0.112, press_z), open_, "press", yaw=0.0),
            Waypoint(1.4, (0.30, -0.112, press_z), open_, "hold the press", yaw=0.0),
            Waypoint(1.0, (0.30, -0.224, contact_z), open_, "release and back off", yaw=0.0),
        ],
        "left": [Waypoint(8.0, (0.20, 0.20, TOP + 0.08), -0.040, "parked")],
    }

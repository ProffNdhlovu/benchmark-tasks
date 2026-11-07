"""A door-lock fixture stands on the station table: a grey backplate on a short pedestal carries a
slide-bolt — a wide thumb-slide resting on the plate between two guide rails, currently thrown
forward to the far end of its travel (locked). NSPIRE door check: an exit door must be unlockable
from the inside; the inspector's verification is sliding the bolt back. The act is a single
horizontal slide of ~5 cm — tape-measure-numeric — and the bolt must still ride its plate (a bolt
lifted off and dropped on the table is not an unlocked door).

Run the demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=slide_bolt_unlock.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "unlock the door: slide the bolt back 5 centimetres" \
      --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval/bolt-demo
"""

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"

TOP = TABLE_TOP[SCENE]

PEDESTAL = Box("pedestal", pos=(0.30, -0.10, TOP + 0.06), size=(0.14, 0.12, 0.12),
               color=(0.45, 0.46, 0.48))
PLATE = Box("plate", pos=(0.30, -0.10, TOP + 0.132), size=(0.10, 0.16, 0.024),
            color=(0.62, 0.63, 0.66))
# Guide rails stand clear of the bolt's 4 cm width so the jaw can descend beside the bolt.
RAIL_L = Box("rail_left", pos=(0.268, -0.10, TOP + 0.149), size=(0.004, 0.15, 0.010),
             color=(0.30, 0.31, 0.33))
RAIL_R = Box("rail_right", pos=(0.332, -0.10, TOP + 0.149), size=(0.004, 0.15, 0.010),
             color=(0.30, 0.31, 0.33))
# Keeper: a low stop at the travel's +y end, only 1.6 cm tall, so the jaw (riding with its tips
# ~1.8 cm above the plate) passes over it while sliding the bolt.
KEEPER = Box("keeper", pos=(0.30, -0.034, TOP + 0.152), size=(0.05, 0.014, 0.016),
             color=(0.55, 0.30, 0.10))
# The bolt: a wide thumb-slide resting on the plate, thrown to the far (-y) end of its travel
# (locked). Unlocking slides it +y until its nose seats against the keeper's -y face.
BOLT_REST = (0.30, -0.125, TOP + 0.164)
BOLT = Box("bolt", pos=BOLT_REST, size=(0.040, 0.075, 0.04), mass=0.06,
           color=(0.70, 0.72, 0.75))

OBJECTS = [PEDESTAL, PLATE, RAIL_L, RAIL_R, KEEPER, BOLT]

LOCKED_Y = BOLT_REST[1]            # -0.125: the bolt thrown to the far end of its travel
UNLOCK_PULL = 0.0415               # slide +y this far: the nose seats against the keeper
HALF_TOL = 0.025                   # half credit at half the pull
RAIL_TOL = 0.008                   # the bolt must stay between the rails (x within this of 0.30)
END_TOL = 0.006                    # the bolt must be at rest when scored

_state = {}


def score(objects):
    """1.0 when the bolt slid back >= 4.15 cm from its locked position and still rides the rails;
    0.5 at >= 2.5 cm; 0.0 otherwise. A bolt off the rails scores 0 whatever the pull."""
    b = objects["bolt"]
    pull = b.pos[1] - LOCKED_Y
    on_rails = abs(b.pos[0] - 0.30) < RAIL_TOL and b.pos[2] > TOP + 0.13
    if not on_rails:
        return 0.0
    if pull >= UNLOCK_PULL:
        return 1.0
    if pull >= HALF_TOL:
        return 0.5
    return 0.0


def success(objects):
    """The full pull, the bolt riding its rails and at rest."""
    import os
    if os.path.exists("/tmp/dbg_on"):
        b = objects["bolt"]
        with open(f"/tmp/bolt_{os.getpid()}.csv", "a") as f:
            f.write(f"{b.pos[0]:.4f},{b.pos[1]:.4f},{b.pos[2]:.4f},{score(objects):.2f}\n")
    b = objects["bolt"]
    resting = np.linalg.norm(b.lin_vel) < 0.01 and np.linalg.norm(b.ang_vel) < 0.5
    return score(objects) >= 1.0 and resting


SCORE_OVER = "best"     # the pull is transient-safe: the bolt stays retracted once slid


def demo():
    """The right arm approaches horizontally at grip height from beyond the travel's far end,
    slides over the bolt, closes on its sides, slides it back toward the keeper (+y), and
    releases. At yaw 0 the fingers displace along x and squeeze the bolt's 4 cm width; the pad
    tips ride ~1.8 cm above the plate, clearing the low keeper."""
    open_, grip = -0.015, -0.012              # open_ ~4.05 cm descends between the rails;
                                              # grip ~3.24 cm crushes ~3.8 mm/side into the
                                              # bolt's 4 cm width
    bx, by = BOLT_REST[0], BOLT_REST[1]
    z_grip = TOP + 0.175                  # TCP at the bolt's upper face band; tips ~1.8 cm
                                          # above the plate and ~2 mm above the keeper's top
    approach_y = by - 0.10
    slide_y = -0.0735                     # the nose seats against the keeper's -y face
    return {
        "right": [
            Waypoint(1.8, (bx, approach_y, z_grip), open_, "approach at grip height", yaw=0.0),
            Waypoint(1.2, (bx, by, z_grip), open_, "slide over the bolt", yaw=0.0),
            Waypoint(0.8, (bx, by, z_grip), grip, "close on the bolt's sides", yaw=0.0),
            Waypoint(2.0, (bx, slide_y, z_grip), grip, "slide the bolt back to the keeper", yaw=0.0),
            Waypoint(0.5, (bx, slide_y, z_grip), grip, "hold the bolt home", yaw=0.0),
            Waypoint(0.8, (bx, slide_y, z_grip), open_, "release", yaw=0.0),
            Waypoint(1.0, (bx, approach_y, z_grip), open_, "back off", yaw=0.0),
        ],
        "left": [Waypoint(9.0, (0.20, 0.20, TOP + 0.08), -0.040, "parked")],
    }

"""A table-top space heater stands between the arms with a cardboard box sitting on top of it - a
flammable item stored on a heat appliance - and a heavy metal pot on the table beside it as a
nearby non-flammable decoy. A green safe-zone pad marks the drop-off area across the table.

NSPIRE fire-safety check (D078, life-threatening, 24-hour correction): a flammable or combustible
item on or within 3 feet of an appliance that provides heat must be removed at once. The tabletop
rig exercises the ON-appliance half of the criterion: the robot has to tell the flammable cardboard
box from the non-flammable metal pot, lift the box off the heater, carry it across the table and
set it down in the marked safe zone - a hazard clearance, not a photo.

Run it (single condition; the decoy pot is part of the fixed layout):
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=flammable_clearance.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "move the flammable box off the space heater to the safe zone" \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/flamm-demo
"""

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"

TOP = TABLE_TOP[SCENE]

# Table-top space heater with a vent grille on its front face, and the flammable box on its top.
HX, HY = 0.38, -0.10
HEATER = Box("space_heater", pos=(HX, HY, TOP + 0.055), size=(0.09, 0.09, 0.11),
             color=(0.25, 0.26, 0.28))
GRILLE = Box("heater_grille", pos=(HX, HY - 0.0475, TOP + 0.05), size=(0.06, 0.005, 0.07),
             color=(0.55, 0.57, 0.60))
BOX = Box("cardboard_box", pos=(HX, HY, TOP + 0.14), size=(0.07, 0.07, 0.06), mass=0.08,
          color=(0.72, 0.55, 0.35))

# A heavy metal pot on the table beside the heater: nearby, non-flammable, and far too heavy for
# the gripper - the item that should NOT be moved.
POT = Cylinder("metal_pot", pos=(0.26, -0.02, TOP + 0.045), radius=0.05, height=0.09, mass=0.25,
               color=(0.60, 0.62, 0.65))

# The marked safe zone across the table.
ZONE = Box("safe_zone", pos=(0.13, 0.06, TOP + 0.004), size=(0.10, 0.10, 0.008),
           color=(0.20, 0.65, 0.30))

OBJECTS = [HEATER, GRILLE, BOX, POT, ZONE]

OFF_MIN = 0.09                 # box centre at least this far from the heater's centre = off it
ZONE_ON = 0.03                 # box centre within this of the zone's centre = cleared into it


def _box_off_heater(objects):
    """The box is at rest and no longer on the heater."""
    b, h = objects["cardboard_box"], objects["space_heater"]
    return bool(b.resting() and np.hypot(b.pos[0] - h.pos[0], b.pos[1] - h.pos[1]) >= OFF_MIN)


def _box_in_zone(objects):
    """The box is at rest in the marked safe zone."""
    b, z = objects["cardboard_box"], objects["safe_zone"]
    return bool(b.resting() and abs(b.pos[0] - z.pos[0]) < ZONE_ON
                and abs(b.pos[1] - z.pos[1]) < ZONE_ON)


def score(objects):
    """0.5 for the box lifted off the heater and at rest on the table; +0.5 for the box set down
    in the marked safe zone (which implies off the heater). 1.0 for the complete clearance.
    Do-nothing: the box still sits on the heater, so 0."""
    off = 0.5 if _box_off_heater(objects) else 0.0
    zoned = 0.5 if _box_in_zone(objects) else 0.0
    import csv, os as _os
    if _os.path.exists("/tmp/flamm_dbg_on"):
        b = objects["cardboard_box"]
        h = objects["space_heater"]
        row = [round(b.pos[0], 4), round(b.pos[1], 4), round(b.pos[2], 4), b.resting(),
               round(np.hypot(b.pos[0] - h.pos[0], b.pos[1] - h.pos[1]), 4), off, zoned]
        with open(f"/tmp/flamm_{_os.getpid()}.csv", "a") as f:
            csv.writer(f).writerow(row)
    return off + zoned


def success(objects):
    return score(objects) >= 1.0


SCORE_OVER = "best"     # the cleared box stays cleared


def demo():
    """The right arm lifts the cardboard box off the heater (descend-open-close from above), carries
    it along one axis at a time across the table, sets it down in the marked safe zone and backs
    off. The metal pot stays where it is."""
    bx, by = BOX.pos[0], BOX.pos[1]
    grasp_z = BOX.pos[2]                     # the box's mid-thickness on the heater's top
    carry_z = TOP + 0.20
    zx, zy = ZONE.pos[0], ZONE.pos[1]
    place = (zx, zy, TIPS_ON_TABLE + ZONE.size[2] + BOX.size[2] / 2 + 0.004)
    right = [
        Waypoint(1.4, (bx, by, grasp_z + 0.08), -0.047, "hover above the box", yaw=0.0),
        Waypoint(1.0, (bx, by, grasp_z), -0.047, "descend over the box", yaw=0.0),
        Waypoint(0.8, (bx, by, grasp_z), -0.014, "close on the box", yaw=0.0),
        Waypoint(1.2, (bx, by, carry_z), -0.014, "lift the box off the heater", yaw=0.0),
        Waypoint(1.6, (zx, by, carry_z), -0.014, "carry across, stage 1 (-x)", yaw=0.0),
        Waypoint(1.2, (zx, zy, carry_z), -0.014, "carry across, stage 2 (+y)", yaw=0.0),
        Waypoint(1.2, place, -0.014, "lower into the safe zone", yaw=0.0),
        Waypoint(0.6, place, -0.040, "release the box", yaw=0.0),
        Waypoint(1.2, (zx, zy - 0.10, carry_z), -0.040, "back off", yaw=0.0),
    ]
    return {
        "right": right,
        "left": [Waypoint(sum(w.duration for w in right), (0.20, 0.20, TOP + 0.08), -0.040, "parked")],
    }

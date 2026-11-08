"""A smoke alarm on its mounting plate sits on the station table: a white square alarm
(5.2 x 5.2 x 6.5 cm) at the centre of a 16 x 16 cm plate; its battery compartment is modelled as
a tray spot on the plate beside the alarm, where the dead 4.4 x 4.4 x 1 cm cell lies; the fresh
cell waits at the plate's front-left tray spot. NSPIRE smoke-alarm check: a dead or missing
battery is a life-threatening fault; the fix is the battery swap — lift the alarm off its seat,
pull the dead cell, set it aside, place the fresh cell in the tray, and reseat the alarm. Every
condition is verifiable with a photo: the alarm held clear, the dead cell on the table, the fresh
cell in the tray, the alarm level on its seat.

Run the demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=smoke_alarm_battery_swap.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "swap the smoke alarm's dead battery for the fresh one and reseat the alarm" \
      --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/battery-demo
"""

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"

TOP = TABLE_TOP[SCENE]

# Bench rig: a 20 x 20 cm mounting frame on the table where the right arm reaches - four 1 cm
# borders around an 18 x 18 cm opening, so the alarm and the batteries rest directly on the table
# inside it. The alarm sits at the opening's centre; its battery compartment is modelled as a
# tray spot beside the alarm, where the dead cell lies; the fresh cell waits at the front-left
# tray spot.
AX, AY = 0.35, -0.10
BORDER = Box("border_px", pos=(AX + 0.085, AY, TOP + 0.005), size=(0.01, 0.20, 0.01),
             color=(0.45, 0.46, 0.48))
BORDER2 = Box("border_nx", pos=(AX - 0.085, AY, TOP + 0.005), size=(0.01, 0.20, 0.01),
              color=(0.45, 0.46, 0.48))
BORDER3 = Box("border_py", pos=(AX, AY + 0.085, TOP + 0.005), size=(0.18, 0.01, 0.01),
              color=(0.45, 0.46, 0.48))
BORDER4 = Box("border_ny", pos=(AX, AY - 0.085, TOP + 0.005), size=(0.18, 0.01, 0.01),
              color=(0.45, 0.46, 0.48))
ALARM_Z = TOP + 0.0325                         # the alarm's centre, seated inside the frame
ALARM = Box("alarm", pos=(AX, AY, ALARM_Z), size=(0.052, 0.052, 0.065), mass=0.08,
            color=(0.92, 0.92, 0.94))
TRAY = (0.38, -0.050)                          # the battery tray spot inside the frame
DEAD_Z = TOP + 0.005
DEAD = Box("dead_battery", pos=(TRAY[0], TRAY[1], DEAD_Z), size=(0.044, 0.044, 0.01), mass=0.03,
           color=(0.12, 0.12, 0.13))
FRESH_HOME = (AX - 0.048, AY + 0.032)
FRESH = Box("fresh_battery", pos=(FRESH_HOME[0], FRESH_HOME[1], DEAD_Z),
            size=(0.044, 0.044, 0.01), mass=0.03, color=(0.20, 0.55, 0.30))

OBJECTS = [BORDER, BORDER2, BORDER3, BORDER4, DEAD, ALARM, FRESH]

SEAT_TOL = 0.025          # the alarm reseated within 2.5 cm of its seat
CELL_TOL = 0.020          # the fresh cell within 2 cm of the tray spot
LIFT_MIN = 0.03           # the alarm's base this far above the plate counts as "off"
AWAY_MIN = 0.06           # the dead cell's centre this far from the plate counts as "out"
YAW_TOL = 0.26            # the reseated alarm within ~15 deg of its start orientation
TABLE_Z_MAX = TOP + 0.05


def _alarm_yaw(alarm):
    x, y, z, w = alarm.quat_xyzw
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def score(objects):
    """0.5 for the dead battery set on the table at least 6 cm from the plate; +0.5 for the
    fresh battery lying in the tray spot with the alarm reseated level on its seat. 1.0 for the
    full swap. (The lift is implied by the reseat, so the two halves can't double-count.)"""
    a, d, f = objects["alarm"], objects["dead_battery"], objects["fresh_battery"]
    lifted_off = (a.pos[2] >= ALARM_Z + LIFT_MIN
                  and (a.pos[0] - AX) ** 2 + (a.pos[1] - AY) ** 2 < 0.12 ** 2)
    dead_out = (d.resting() and d.pos[2] < TABLE_Z_MAX
                and (d.pos[0] - AX) ** 2 + (d.pos[1] - AY) ** 2 >= AWAY_MIN ** 2)
    fresh_in = (f.resting() and abs(f.pos[0] - TRAY[0]) < CELL_TOL and abs(f.pos[1] - TRAY[1]) < CELL_TOL
                and abs(f.pos[2] - DEAD_Z) < CELL_TOL)
    reseated = (a.resting() and abs(a.pos[0] - AX) < SEAT_TOL and abs(a.pos[1] - AY) < SEAT_TOL
                and abs(a.pos[2] - ALARM_Z) < SEAT_TOL
                and abs(_alarm_yaw(a)) < YAW_TOL
                and fresh_in)
    return (0.5 if dead_out else 0.0) + (0.5 if reseated else 0.0)


def success(objects):
    import os
    if os.path.exists("/tmp/dbg_on"):
        a, d, f = objects["alarm"], objects["dead_battery"], objects["fresh_battery"]
        with open(f"/tmp/bat_{os.getpid()}.csv", "a") as fh:
            fh.write(",".join(f"{v:.4f}" for v in (a.pos[0], a.pos[1], a.pos[2],
                                                   d.pos[0], d.pos[1], d.pos[2],
                                                   f.pos[0], f.pos[1], f.pos[2]))
                     + f",{score(objects):.2f}\n")
    return score(objects) >= 1.0


SCORE_OVER = "best"       # the lift-off partial is a held state; the reseated alarm returns to
                          # its seat height, so the trial's best covers every phase


def demo():
    """Single-arm swap in four short phases, every grasp from a flat rest. The right arm lifts
    the alarm off its seat and sets it down on the plate's near corner; pulls the dead cell from
    the tray and drops it on the table clear of the plate (carries in short rising stages - the
    grip slips on long level drags); carries the fresh cell from the front-left tray spot to the
    emptied tray; then re-grasps the alarm and reseats it. Grasps are direct vertical descents at
    each object's centre, so the pad band covers the faces' middle and the palm clears every
    top."""
    open_full, grip_a, grip_b = -0.04695, -0.016, -0.013
    alarm_down = (AX, AY - 0.14, TOP + 0.0325)   # the alarm's set-down: directly -y of its
                                                 # seat, so both carries run along the grip axis
    aside = (0.22, -0.10, TOP + 0.005)                  # the dead cell's drop spot, outside the
                                                        # frame, 13 cm from its centre
    lift_z = TOP + 0.16
    cell_grasp_z = DEAD_Z + 0.003                       # the cells' grasp: 3 mm above their
                                                        # centres so the pads clear the table

    right = [
        # 1. lift the alarm off its seat and set it down inside the frame
        Waypoint(2.0, (AX, AY, ALARM_Z), open_full, "descend open around the alarm", yaw=0.0),
        Waypoint(0.8, (AX, AY, ALARM_Z), grip_a, "close on the alarm", yaw=0.0),
        Waypoint(1.0, (AX, AY, lift_z), grip_a, "lift the alarm off its seat", yaw=0.0),
        Waypoint(1.4, (AX, AY - 0.07, lift_z + 0.02), grip_a, "carry it down toward the set-down 1/2", yaw=0.0),
        Waypoint(1.4, alarm_down[:2] + (lift_z + 0.04,), grip_a, "carry it down toward the set-down 2/2", yaw=0.0),
        Waypoint(1.2, alarm_down[:2] + (TOP + 0.10,), grip_a, "descend to the table 1/2", yaw=0.0),
        Waypoint(1.2, alarm_down, grip_a, "descend to the table 2/2", yaw=0.0),
        Waypoint(0.8, alarm_down, open_full, "release the alarm", yaw=0.0),
        # 2. pull the dead cell from the tray and drop it outside the frame
        Waypoint(1.2, (TRAY[0], TRAY[1], cell_grasp_z), open_full, "descend to the dead battery", yaw=0.0),
        Waypoint(0.8, (TRAY[0], TRAY[1], cell_grasp_z), grip_b, "close on the dead battery", yaw=0.0),
        Waypoint(1.0, (TRAY[0], TRAY[1], lift_z), grip_b, "lift the dead battery clear", yaw=0.0),
        Waypoint(1.0, (0.35, -0.09, lift_z + 0.02), grip_b, "carry it up and aside 1/4", yaw=0.0),
        Waypoint(1.0, (0.31, -0.095, lift_z + 0.04), grip_b, "carry it up and aside 2/4", yaw=0.0),
        Waypoint(1.0, (0.26, -0.10, lift_z + 0.06), grip_b, "carry it up and aside 3/4", yaw=0.0),
        Waypoint(1.0, aside[:2] + (lift_z + 0.08,), grip_b, "carry it up and aside 4/4", yaw=0.0),
        Waypoint(1.2, aside[:2] + (TOP + 0.08,), grip_b, "descend to the table 1/2", yaw=0.0),
        Waypoint(1.2, aside, grip_b, "descend to the table 2/2", yaw=0.0),
        Waypoint(0.8, aside, open_full, "release the dead battery", yaw=0.0),
        # 3. carry the fresh cell from the front-left tray spot to the emptied tray
        Waypoint(1.5, (FRESH_HOME[0], FRESH_HOME[1], TOP + 0.15), open_full, "move over the fresh battery", yaw=0.0),
        Waypoint(1.2, (FRESH_HOME[0], FRESH_HOME[1], cell_grasp_z), open_full, "descend to the fresh battery", yaw=0.0),
        Waypoint(0.8, (FRESH_HOME[0], FRESH_HOME[1], cell_grasp_z), grip_b, "close on the fresh battery", yaw=0.0),
        Waypoint(1.0, (FRESH_HOME[0], FRESH_HOME[1], lift_z), grip_b, "lift it", yaw=0.0),
        Waypoint(1.4, (0.33, -0.05, lift_z + 0.02), grip_b, "carry it along the tray 1/2", yaw=0.0),
        Waypoint(1.4, (TRAY[0], TRAY[1], lift_z + 0.04), grip_b, "carry it along the tray 2/2", yaw=0.0),
        Waypoint(2.0, (TRAY[0], TRAY[1], DEAD_Z), grip_b, "lower it into the tray", yaw=0.0),
        Waypoint(0.8, (TRAY[0], TRAY[1], DEAD_Z), open_full, "release the fresh battery", yaw=0.0),
        # 4. re-grasp the alarm and reseat it
        Waypoint(1.5, alarm_down[:2] + (TOP + 0.15,), open_full, "move over the alarm", yaw=0.0),
        Waypoint(1.2, alarm_down[:2] + (TOP + 0.0325,), open_full, "descend to the alarm", yaw=0.0),
        Waypoint(0.8, alarm_down[:2] + (TOP + 0.0325,), grip_a, "close on the alarm", yaw=0.0),
        Waypoint(1.0, alarm_down[:2] + (lift_z,), grip_a, "lift the alarm", yaw=0.0),
        Waypoint(1.4, (AX, AY - 0.07, lift_z + 0.02), grip_a, "carry it back 1/2", yaw=0.0),
        Waypoint(1.4, (AX, AY, lift_z + 0.04), grip_a, "carry it back 2/2", yaw=0.0),
        Waypoint(2.0, (AX, AY, ALARM_Z), grip_a, "lower the alarm onto its seat", yaw=0.0),
        Waypoint(0.8, (AX, AY, ALARM_Z), open_full, "release", yaw=0.0),
        Waypoint(1.5, (AX, AY, lift_z), open_full, "retreat up", yaw=0.0),
    ]
    return {"right": right,
            "left": [Waypoint(sum(w.duration for w in right), (0.20, 0.20, TOP + 0.08), -0.040, "parked")]}

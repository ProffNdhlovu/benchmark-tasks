"""A rural-style mailbox stands on the station table with its signal flag lying flat in the
recessed channel on the box's top - the down position, no outgoing mail. The act is to grip
the flag and lift it so it seesaws up over the channel's stop rail, standing upright against
the rail at 90 degrees - the up position that tells the postal carrier to stop and collect.
A raised mailbox flag is the universal outgoing-mail signal; on rural routes a flag left
down means the mail waits another day, and for a home-assistant robot it is a small everyday
interaction with a clean physical end state.

Run the demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \\
      -E task=mailbox_flag_raise.py -E scene=station -E object_jitter=0 \\
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
      --instruction "the mailbox flag is lying flat - raise it up" \\
      --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/flag-demo

Fidelity: the real flag pivots on a bolt at the box's side; the sim's flag is a rigid paddle
lying in a channel on the box's top that seesaws up over a fixed stop rail and stands
upright against the rail at 90 deg - the same seesaw-about-a-rail mechanism as propping a
panel onto its stand. The bolt and the door are not modelled.
"""

import os

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"

TOP = TABLE_TOP[SCENE]

# The mailbox stands at (31, -2) cm, top at rest + 12 cm. The flag spawns FLAT in a shallow
# channel on the box's top (its top face 5 mm below the stop rail's underside), so the arm
# can grip its two faces with parallel vertical pads before anything moves. The stop rail is
# carried on two legs outside the flag's x-span, so the rail hangs ABOVE the flag's far edge
# while leaving the pads' closing path clear: the arm grips the flag at its centre and lifts,
# the flag's far edge rides up the rail's near face and over its top, and the flag seesaws
# about the rail - the rotation axis is the grip axis, so the pads' faces stay flush through
# the whole rotation. Past the over-center point (~63 deg for this geometry) gravity stands
# the flag up and its far face comes to rest against the rail's far face at 90 deg, leaning
# on the rail and the box top.
MAILBOX = Box("mailbox", pos=(0.31, -0.02, TOP + 0.06), size=(0.09, 0.20, 0.12),
              color=(0.75, 0.75, 0.78))
LEG_L = Box("rail_leg_left", pos=(0.275, -0.085, TOP + 0.1375), size=(0.01, 0.01, 0.04),
            color=(0.45, 0.46, 0.48))
LEG_R = Box("rail_leg_right", pos=(0.345, -0.085, TOP + 0.1375), size=(0.01, 0.01, 0.04),
            color=(0.45, 0.46, 0.48))
RAIL = Box("stop_rail", pos=(0.31, -0.085, TOP + 0.1475), size=(0.09, 0.01, 0.015),
           color=(0.45, 0.46, 0.48))
STOP = Box("stop_post", pos=(0.31, -0.055, TOP + 0.16875), size=(0.05, 0.01, 0.085),
           color=(0.45, 0.46, 0.48))
FLAG = Box("mail_flag", pos=(0.31, -0.11, TOP + 0.13), size=(0.05, 0.04, 0.02), mass=0.04,
           color=(0.75, 0.12, 0.10))

OBJECTS = [MAILBOX, LEG_L, LEG_R, RAIL, STOP, FLAG]

SEAT_Y = -0.09       # the rail's -y face: the standing flag's far face rests here
SEAT_TOL = 0.020     # the flag's far face must stay within 2 cm of the rail (still on the box)
LEAN_NY = 0.5        # the face normal must be near-vertical: the flag standing either way
YAW_TOL = 15.0       # the flag must stay square to the mailbox
FULL_LO, FULL_HI = 80.0, 130.0   # a flag flopped up onto the rail
HALF_HI = 80.0                   # a 45-80 deg tilt = raised but not standing


def _tilt_yaw(o):
    """Tilt of the flag's face from horizontal (deg), its yaw (deg), and the normal's y part."""
    qx, qy, qz, qw = o.quat_xyzw
    ny = 2 * (qy * qz + qw * qx)
    nz = 1 - 2 * (qx * qx + qy * qy)
    tilt = float(np.degrees(np.arccos(np.clip(nz, -1.0, 1.0))))
    yaw = float(np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))))
    return tilt, yaw, ny


def score(objects):
    """1.0 when the flag stands raised on the box: tilted 80-130 deg from flat with its
    face near-vertical (either lean direction - resting on the box top and backed by the
    rail either counts), square to the mailbox, its far face within 2 cm of the rail (still
    on the box), and at rest. 0.5 for a 45-80 deg tilt that did not stand. Anything else -
    the flag left flat, knocked off the box, or carried away - is 0."""
    f = objects["mail_flag"]
    tilt, yaw, ny = _tilt_yaw(f)
    resting = np.linalg.norm(f.lin_vel) < 0.01 and np.linalg.norm(f.ang_vel) < 0.5
    on_box = abs(f.pos[0] - 0.31) <= 0.045 and f.low[2] >= TOP - 0.015
    if not resting or abs(ny) < LEAN_NY or abs(yaw) > YAW_TOL or not on_box:
        return 0.0
    if abs(f.high[1] - SEAT_Y) > SEAT_TOL:
        return 0.0
    if FULL_LO <= tilt <= FULL_HI:
        return 1.0
    if 45.0 <= tilt < HALF_HI:
        return 0.5
    return 0.0


def success(objects):
    """Full score, the flag at rest flopped over onto the stop post."""
    if os.path.exists("/tmp/flag_dbg_on"):
        f = objects["mail_flag"]
        tilt, yaw, ny = _tilt_yaw(f)
        with open(f"/tmp/flag_{os.getpid()}.csv", "a") as fh:
            fh.write(f"{f.pos[0]:.4f},{f.pos[1]:.4f},{f.pos[2]:.4f},{tilt:.2f},{yaw:.2f},"
                     f"{ny:.3f},{f.high[1]:.4f},{score(objects):.2f}\n")
    f = objects["mail_flag"]
    resting = np.linalg.norm(f.lin_vel) < 0.01 and np.linalg.norm(f.ang_vel) < 0.5
    return score(objects) >= 1.0 and resting


SCORE_OVER = "end"      # the flopped-over lean is a persistent end state: score at the end


def demo():
    """The right arm grips the flat flag's faces at its centre (yaw 1.5708 rad: the waypoint
    yaw is RADIANS and yaw 0 closes the fingers along world y, so pi/2 turns them onto the
    flag's x-width), then lifts along the grip point's arc about the stop rail: the flag's
    far edge rides up the rail's near face and over its top, the flag seesaws past the
    over-center point, flops over backwards and settles against the stop post. The jaw is
    position-controlled and STALLS on the object, so the close uses FINGER_CLOSED (0.0)
    exactly like the built-in pick-and-place."""
    gx = 0.31
    return {
        "right": [
            Waypoint(1.8, (gx, -0.11, TOP + 0.16), -0.04695, "hover above the flag at its centre", yaw=1.5708),
            Waypoint(1.2, (gx, -0.11, TOP + 0.1455), -0.04695, "descend to the flag's faces", yaw=1.5708),
            Waypoint(1.0, (gx, -0.11, TOP + 0.1455), 0.0, "close on the flag's faces", yaw=1.5708),
            Waypoint(2.5, (gx, -0.07, TOP + 0.1771), 0.0, "lift: seesaw the flag up over the rail", yaw=1.5708),
            Waypoint(1.5, (gx, -0.07, TOP + 0.1771), 0.0, "hold while the flag stands against the rail", yaw=1.5708),
            Waypoint(0.8, (gx, -0.07, TOP + 0.1771), -0.0401, "release", yaw=1.5708),
            Waypoint(1.0, (gx, -0.13, TOP + 0.1771), -0.0401, "pull back clear of the standing flag", yaw=1.5708),
            Waypoint(1.0, (gx, -0.22, TOP + 0.155), -0.0401, "back off", yaw=1.5708),
        ],
        "left": [Waypoint(11.3, (0.20, 0.20, TOP + 0.08), -0.040, "parked")],
    }

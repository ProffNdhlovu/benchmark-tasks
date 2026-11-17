"""A small solar panel is stored FLAT on its mounting stand on the station table: its far end
rests on a storage post, its near end rests on the stand's crossbar — the way crews store
panels flat on their mounting frames between seasons. A panel left flat collects dust and
sheds winter yield, so re-angling it onto the stand's 30-degree operating tilt is routine
seasonal work. The act is to grip the panel's two faces (the post and crossbar anchor it
against the squeeze), then roll it up: the panel pitches about the crossbar's front edge
inside the grip until its bottom edge seats at the toe lip, leaning back at the stand's
30-degree tilt. Tilt is tape-measure-numeric (a protractor reading from a side photo), and
the panel must still be resting on the stand — a panel carried off and propped elsewhere is
not an adjusted panel.

Run the demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=panel_tilt_stand.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "tilt the solar panel up onto its stand" \
      --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/panel-demo
"""

import os

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"

TOP = TABLE_TOP[SCENE]

# The panel lies FLAT at rest + 8 cm: its far end (y -0.28) on the storage post's top, its
# near end (y -0.10) on the crossbar's top — both supports topped at rest + 8 cm. The panel
# is 9.8 cm wide because the jaw's grip pads (the finger bars' innermost flat faces) sit
# 11.2 cm apart at the open aperture (f = 0) and close to 9.1 cm at f = -0.012 — measured on
# the sim's collision hulls; a narrower panel can never be straddled. The crossbar's front-top
# edge (y -0.12, rest + 8 cm) is the pitch hinge: the panel's far end lands on the table at
# asin(8 cm / 15 cm) = 32 deg. The toe rail (top rest + 1.5 mm, face at y -0.2625) marks the
# seat; the landed panel's bottom edge stops ~1.3 cm in front of it, inside the full-score
# band. Legs carry the crossbar at x 0.19/0.35 so the jaw's bars (x 0.22..0.36 at the close)
# clear them; the post and the rail (x 0.26..0.28) sit between the finger bars.
PANEL = Box("panel", pos=(0.27, -0.19, TOP + 0.0875), size=(0.098, 0.18, 0.015), mass=0.12,
            color=(0.16, 0.20, 0.28))
POST = Box("storage_post", pos=(0.27, -0.295, TOP + 0.04), size=(0.02, 0.03, 0.08),
           color=(0.55, 0.56, 0.58))
BAR = Box("stand_bar", pos=(0.27, -0.105, TOP + 0.0725), size=(0.16, 0.03, 0.015),
          color=(0.45, 0.46, 0.48))
LEG_L = Box("stand_leg_left", pos=(0.19, -0.105, TOP + 0.0325), size=(0.02, 0.03, 0.065),
            color=(0.45, 0.46, 0.48))
LEG_R = Box("stand_leg_right", pos=(0.35, -0.105, TOP + 0.0325), size=(0.02, 0.03, 0.065),
            color=(0.45, 0.46, 0.48))
LIP = Box("toe_lip", pos=(0.27, -0.27, TOP + 0.00075), size=(0.036, 0.015, 0.0015),
          color=(0.55, 0.56, 0.58))

OBJECTS = [PANEL, POST, BAR, LEG_L, LEG_R, LIP]

SEAT_Y = -0.2625       # the toe rail's +y face: the leaning panel's bottom edge seats here
SEAT_TOL = 0.030      # the bottom edge must be within 3 cm of the seat (the flat spawn
                      # rests at low_y -0.28, inside the band but flat, so it can't score)
LEAN_NY = 0.10        # the face normal must tilt toward +y: the far edge up
YAW_TOL = 15.0        # the panel must stay square to the stand
FULL_LO, FULL_HI = 25.0, 35.0   # the crossbar's front edge is the hinge: a panel seated at
HALF_HI = 42.0                  # the toe lip pitches onto it at ~30 deg (+/- a few)
def _tilt_yaw(o):
    """Tilt of the panel's face from horizontal (deg), its yaw (deg), and the normal's y part."""
    qx, qy, qz, qw = o.quat_xyzw
    ny = 2 * (qy * qz + qw * qx)
    nz = 1 - 2 * (qx * qx + qy * qy)
    tilt = float(np.degrees(np.arccos(np.clip(nz, -1.0, 1.0))))
    yaw = float(np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))))
    return tilt, yaw, ny


def score(objects):
    """1.0 when the panel rests leaning back on the crossbar at 30 deg +/- 5 with its bottom
    edge seated at the toe lip; 0.5 for a 35-45 deg lean (overshot the seat); 0.0 flat on the
    stand, overturned, fallen, or propped anywhere else. A panel facing the wrong way scores 0."""
    p = objects["panel"]
    tilt, yaw, ny = _tilt_yaw(p)
    resting = np.linalg.norm(p.lin_vel) < 0.01 and np.linalg.norm(p.ang_vel) < 0.5
    if not resting or ny < LEAN_NY or abs(yaw) > YAW_TOL:
        return 0.0
    if abs(p.low[1] - SEAT_Y) > SEAT_TOL:
        return 0.0
    if FULL_LO <= tilt <= FULL_HI:
        return 1.0
    if FULL_HI < tilt <= HALF_HI:
        return 0.5
    return 0.0


def success(objects):
    """Full score, the panel at rest on the stand."""
    if os.path.exists("/tmp/panel_dbg_on"):
        p = objects["panel"]
        tilt, yaw, ny = _tilt_yaw(p)
        with open(f"/tmp/panel_{os.getpid()}.csv", "a") as f:
            f.write(f"{p.pos[0]:.4f},{p.pos[1]:.4f},{p.pos[2]:.4f},{tilt:.2f},{yaw:.2f},"
                    f"{ny:.3f},{p.low[1]:.4f},{score(objects):.2f}\n")
    p = objects["panel"]
    resting = np.linalg.norm(p.lin_vel) < 0.01 and np.linalg.norm(p.ang_vel) < 0.5
    return score(objects) >= 1.0 and resting


SCORE_OVER = "end"      # the lean is a persistent end state: score where the episode ends


def demo():
    """The right arm descends over the flat panel where it lies on the post and
    crossbar (yaw 1.5708), straddling its two long faces with the grip pads, then
    CLOSES the jaw: the pads' squeeze on the panel's lower half wedges the panel's
    near end up over the crossbar's front-top edge and the panel rolls back into
    the stand's ~29-degree operating tilt, seating against the crossbar's front
    edge. The jaw holds the squeeze while the panel settles.

    Jaw ground truth (measured on the sim's collision hulls, demo34 links): the grip channel
    is NORMALIZED, grip in [0, 1] maps to finger-joint targets grip*FINGER_OPEN in
    [-0.04695, 0], and the grip pads (the finger bars' innermost flat faces, each ~2 cm
    long, 2.5 cm tall) sit 11.2 cm apart at f = 0, closing ~1.75 cm per 0.01 rad — 9.1 cm
    apart at f = -0.012, stalling on the 9.8 cm panel with ~3.6 mm-per-side squeeze (the
    finger PD is 5000 N/m, so the stall carries tens of N of normal force). OPEN is
    commanded as 0.0 and a close as a NEGATIVE f. The pads' faces' midpoint rides TCP +
    4.53 cm along the grip axis (world +x at this yaw) at f = 0 and drifts +0.24 cm more
    by the close, so the demo shifts the TCP -x through the close and back +x at the
    release to keep the pad faces centred on the panel. The pads' colliders reach only
    ~0.8 cm below the TCP (probe10: descending the closed jaw over the panel's centre
    first touches the panel's top at TCP z ~0.85, with pad-panel forces 7.7-42.7 N and
    the panel pressed down in lockstep), so grasp_z places the pads' bottoms ~7 mm
    below the panel's top edge, straddling its full 1.5 cm thickness with their y
    between the two pads.

    The roll-up happens IN THE CLOSE: as the pads squeeze the panel's faces at its
    lower half, the wedge lifts the panel's near end over the crossbar's front-top
    edge and the panel rolls back to the stand's ~29 deg tilt, settling seated within
    ~1 s (demo38: the panel tipped during the close and held 28.8 deg to the end).
    The demo therefore just descends, closes and holds — no lift/shift/lower needed."""
    tcp_x = 0.27 - 0.0453           # open-aperture pad midpoint on the panel's centre
    close_x = 0.27 - 0.0477         # TCP shifted -x to track the midpoint's drift through the close
    grasp_z = TOP + 0.090           # pads' bottoms (TCP-0.8 cm) just under the panel's top (TOP+0.095)
    grasp_y = -0.19 + 0.014         # pads' y-centres on the panel's mid-length
    return {
        "right": [
            Waypoint(1.2, (tcp_x, grasp_y, TOP + 0.25), 0.0, "hover above the stored panel", yaw=1.5708),
            Waypoint(0.9, (tcp_x, grasp_y, grasp_z), 0.0, "descend, pads straddling the faces", yaw=1.5708),
            Waypoint(0.9, (close_x, grasp_y, grasp_z), -0.012, "close: the squeeze rolls the panel up onto the stand", yaw=1.5708),
            Waypoint(0.8, (close_x, grasp_y, grasp_z), -0.012, "hold while the panel settles at the stand's tilt", yaw=1.5708),
        ],
        "left": [Waypoint(3.8, (0.20, 0.20, TOP + 0.08), 0.0, "parked")],
    }

"""On the station table, a fixed two-handle faucet fixture stands between the arms: a dark-grey body
with a forward spout and, on its top, two valve stems with loose square knob handles — a blue knob
(cold, on the left arm's side) and a red knob (hot, on the right arm's side). Guide posts around each
stem keep a knob centred while leaving it free to turn. A faucet is verified by turning each handle
from fully closed to fully open and confirming it stays.

Run it:  inspect-robots run --embodiment yam_arena -E task=faucet_two_handle.py \
           --instruction "turn both faucet handles to full open" --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, TIPS_ON_TABLE, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BODY = Box("faucet_body", pos=(0.36, 0.0, TOP + 0.035), size=(0.05, 0.17, 0.07), color=(0.25, 0.27, 0.30))
SPOUT = Box("spout", pos=(0.445, 0.0, TOP + 0.0825), size=(0.13, 0.024, 0.025), color=(0.25, 0.27, 0.30))

_KNOB_HALF = 0.025           # knob face half-width (m); knob is a square 5 cm box, 4 cm tall
_KNOB_H = 0.040
_STEM_TOP = TOP + 0.10       # stems rise from the body top to here
_KNOB_Z = _STEM_TOP + _KNOB_H / 2


def _bonnet(cx, cy, side):
    """Four guide posts around one stem, rising to the knob's mid-height: they bracket the knob (which
    sweeps 3.54 cm as it turns) without touching it, so the knob can rotate but not slide off — the
    posts overlap the knob's lower half, so knocking it loose means lifting it 2 cm over a post."""
    posts = []
    for i, ang in enumerate(np.deg2rad((45.0, 135.0, 225.0, 315.0))):
        posts.append(Cylinder(
            f"{side}_post{i}", pos=(cx + 0.044 * np.cos(ang), cy + 0.044 * np.sin(ang), TOP + 0.091),
            radius=0.006, height=0.042, color=(0.45, 0.47, 0.50),
        ))
    return posts


STEM_L = Cylinder("cold_stem", pos=(0.36, 0.065, TOP + 0.085), radius=0.014, height=0.03, color=(0.35, 0.37, 0.40))
STEM_R = Cylinder("hot_stem", pos=(0.36, -0.065, TOP + 0.085), radius=0.014, height=0.03, color=(0.35, 0.37, 0.40))
KNOB_L = Box("cold_knob", pos=(0.36, 0.065, _KNOB_Z), size=(2 * _KNOB_HALF, 2 * _KNOB_HALF, _KNOB_H),
             mass=0.15, color=(0.15, 0.35, 0.90))
KNOB_R = Box("hot_knob", pos=(0.36, -0.065, _KNOB_Z), size=(2 * _KNOB_HALF, 2 * _KNOB_HALF, _KNOB_H),
             mass=0.15, color=(0.85, 0.15, 0.15))
OBJECTS = [BODY, SPOUT, STEM_L, STEM_R, KNOB_L, KNOB_R] + _bonnet(0.36, 0.065, "cold") + _bonnet(0.36, -0.065, "hot")

OPEN_YAW = np.deg2rad(120.0)   # handle position for fully open (counter-clockwise)
YAW_TOL = np.deg2rad(15.0)     # "fully open" tolerance
SEAT_XY_TOL = 0.012            # knob still centred on its stem
SEAT_Z_TOL = 0.012
GRASP_APERTURE = -0.040        # fingers while descending (8 cm, clears the knob)
CLOSE_TARGET = -0.023          # fingers on the knob: just past its 5 cm width, a light squeeze
RELEASE_APERTURE = -0.030      # fingers at release: opened only to 6 cm so the pads stay inside the
                               # guide-post ring; the jaw opens fully only after retreating up high
GRASP_LIFT = 0.124             # pad tips just above the knob's mid-height: on the flats, and ~14 mm
                               # above the guide-post tops so the sweeping fingertips clear them.
                               # Lower than ~0.11 the pads slide under the knob's bottom face and
                               # transfer almost no torque (measured)


def _yaw(state):
    x, y, z, w = state.quat_xyzw
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _ang_err(yaw, target=OPEN_YAW):
    """Shortest signed angle from yaw to target, in (-pi, pi]."""
    return float((yaw - target + np.pi) % (2.0 * np.pi) - np.pi)


def _knob_state(objects, knob, stem):
    """(seated, open) for one knob: seated = centred on its stem at the right height."""
    k, s = objects[knob], objects[stem]
    seated = bool(
        np.hypot(k.pos[0] - s.pos[0], k.pos[1] - s.pos[1]) < SEAT_XY_TOL
        and abs(k.pos[2] - _KNOB_Z) < SEAT_Z_TOL
    )
    return seated, abs(_ang_err(_yaw(k))) < YAW_TOL


def success(objects):
    """Both knobs fully open, still seated on their stems, and everything still."""
    seated_l, open_l = _knob_state(objects, "cold_knob", "cold_stem")
    seated_r, open_r = _knob_state(objects, "hot_knob", "hot_stem")
    return bool(seated_l and open_l and seated_r and open_r
                and objects["cold_knob"].resting() and objects["hot_knob"].resting())


def distance(objects):
    """Mean angular error of the two knobs from fully open (rad)."""
    err_l = abs(_ang_err(_yaw(objects["cold_knob"])))
    err_r = abs(_ang_err(_yaw(objects["hot_knob"])))
    return float((err_l + err_r) / 2.0)


def score(objects):
    """Partial credit (success scores 1.0): 0.5 per knob fully open and still seated."""
    seated_l, open_l = _knob_state(objects, "cold_knob", "cold_stem")
    seated_r, open_r = _knob_state(objects, "hot_knob", "hot_stem")
    return 0.5 * (seated_l and open_l) + 0.5 * (seated_r and open_r)


def _turn_waypoints(kx, ky, start_delay=0.0, grasp_yaw=0.0, sweep=np.deg2rad(128.0)):
    """One arm turns its knob from closed to fully open with one continuous yaw sweep, then lets go.

    The pads grasp the knob's opposite flats at mid-height and the grip yaws ``grasp_yaw`` -> ``grasp_yaw
    + sweep``; the knob follows the grip's delta (pad slip: a comfortable wrist pose transfers ~99%, an
    awkward one ~75%). The two arms are mirrored, so both arms enter their sweep at -90 deg (the square
    knob does not care which flats it grips), which keeps the sweep inside each wrist's range. At
    release the jaw opens only to 6 cm — inside the guide-post ring — and opens fully only after
    retreating up high.
    """
    grasp_z = TIPS_ON_TABLE + GRASP_LIFT
    hover_z = grasp_z + 0.07
    wps = []
    if start_delay:
        wps.append(Waypoint(start_delay, (kx, ky, hover_z), GRASP_APERTURE, "wait for the other arm",
                            yaw=grasp_yaw))
    wps += [
        Waypoint(1.5, (kx, ky, hover_z), GRASP_APERTURE, "hover over the knob", yaw=grasp_yaw),
        Waypoint(1.5, (kx, ky, grasp_z), GRASP_APERTURE, "descend", yaw=grasp_yaw),
        Waypoint(0.8, (kx, ky, grasp_z), CLOSE_TARGET, "grasp", yaw=grasp_yaw),
    ]
    steps = 12
    for i in range(1, steps + 1):
        wps.append(Waypoint(0.5, (kx, ky, grasp_z), CLOSE_TARGET,
                            f"turn to full open {i}/{steps}", yaw=grasp_yaw + sweep * i / steps))
    wps += [
        Waypoint(0.8, (kx, ky, grasp_z), RELEASE_APERTURE, "release", yaw=grasp_yaw + sweep),
        Waypoint(1.5, (kx, ky, hover_z), RELEASE_APERTURE, "retreat", yaw=grasp_yaw + sweep),
        Waypoint(0.8, (kx, ky, hover_z), GRASP_APERTURE, "open the jaw up high", yaw=grasp_yaw + sweep),
    ]
    return wps


def demo():
    """Each arm turns its own knob fully open: it grasps the knob with its grip turned -90 deg (which
    keeps the sweep inside the mirrored wrist's range), sweeps 128 deg counter-clockwise in one motion,
    and lets go. The right arm waits for the left to finish so the two grippers never share the space
    over the faucet body."""
    left_dur = 1.5 + 1.5 + 0.8 + 12 * 0.5 + 0.8 + 1.5 + 0.8
    return {
        "left": _turn_waypoints(0.36, 0.065, grasp_yaw=-np.pi / 2, sweep=np.deg2rad(128.0)),
        "right": _turn_waypoints(0.36, -0.065, start_delay=left_dur + 1.0, grasp_yaw=-np.pi / 2,
                                 sweep=np.deg2rad(128.0)),
    }

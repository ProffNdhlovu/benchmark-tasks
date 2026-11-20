"""Two receptacles stand side by side on the station table. The farther one is
visibly scorched (its pedestal is blackened - a burned contact that can arc),
and a lamp's plug is seated in its well. The robot pulls the plug out of the
scorched socket and seats it in the sound one beside it. The score is a single
final physical state - the plug resting in the sound socket's well - so there
is no report to file and nothing to guess.

Run it:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=receptacle_plug_move.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "One receptacle's pedestal is scorched black - move the lamp's plug from it to the sound socket beside it." \
      --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/plugmove-demo

Fidelity: a rigid-body stand-in of moving a plug to a sound receptacle. Each
socket is a 17 cm pedestal with two guide walls forming an 8 cm-wide well; the
plug is a 4x3.5x3.5 cm grip stand-in (its cord and prongs are not modelled) and
runs 40 g, lighter than a real moulded plug. Scorched faceplates are a real
fire risk and a standard inspection finding; the damage is marked by the
blackened pedestal a camera can see.
"""

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"

TOP = TABLE_TOP[SCENE]

# Farther (scorched) and nearer (sound) sockets, side by side. Pedestals stand
# 17 cm tall so the plug's seat sits inside the arm's working band at the grasp
# reach (~42 cm from the right arm's base); the grip plane is ~18 cm above the
# table, which is where this arm's pads actually reach.
LX, RX, OY = 0.34, 0.22, -0.06
PED_H = 0.17
PED_L = Box("outlet_pedestal_l", pos=(LX, OY, TOP + PED_H / 2), size=(0.12, 0.08, PED_H),
            color=(0.22, 0.17, 0.14))   # scorched: blackened pedestal
WALL_LL = Box("well_wall_ll", pos=(LX - 0.045, OY, TOP + PED_H + 0.006), size=(0.01, 0.06, 0.012),
              color=(0.30, 0.31, 0.33))
WALL_LR = Box("well_wall_lr", pos=(LX + 0.045, OY, TOP + PED_H + 0.006), size=(0.01, 0.06, 0.012),
              color=(0.30, 0.31, 0.33))
PED_R = Box("outlet_pedestal_r", pos=(RX, OY, TOP + PED_H / 2), size=(0.12, 0.08, PED_H),
            color=(0.88, 0.88, 0.90))
WALL_RL = Box("well_wall_rl", pos=(RX - 0.045, OY, TOP + PED_H + 0.006), size=(0.01, 0.06, 0.012),
              color=(0.30, 0.31, 0.33))
WALL_RR = Box("well_wall_rr", pos=(RX + 0.045, OY, TOP + PED_H + 0.006), size=(0.01, 0.06, 0.012),
              color=(0.30, 0.31, 0.33))

# The lamp's plug (a grip stand-in), seated in the scorched socket's well.
PLUG = Box("lamp_plug", pos=(LX, OY, TOP + PED_H + 0.0195), size=(0.04, 0.035, 0.035), mass=0.04,
           color=(0.35, 0.36, 0.38))

OBJECTS = [PED_L, WALL_LL, WALL_LR, PED_R, WALL_RL, WALL_RR, PLUG]

SEAT_Z = TOP + PED_H + 0.0175   # plug centre resting on a well floor (pedestal top + half the plug)
SEAT_TOL = 0.008
ON_WELL = 0.012                 # plug centre within this of the sound well's centre in x and y


def score(objects):
    """1.0 when the plug rests in the sound (nearer) socket's well: its centre within 1.2 cm of
    that well's centre in both x and y and at the seat height within 8 mm. Anything else - the
    plug left in the scorched socket, lying on the table, or knocked off - is 0."""
    p = objects["lamp_plug"]
    seated = (abs(p.pos[0] - RX) < ON_WELL and abs(p.pos[1] - OY) < ON_WELL
              and abs(p.pos[2] - SEAT_Z) < SEAT_TOL and p.resting())
    return 1.0 if seated else 0.0


def success(objects):
    return score(objects) >= 1.0


SCORE_OVER = "end"


def demo():
    """Right arm: descend into the scorched socket's well at the well-fit aperture (the guide
    walls leave ~7 mm per side, the plug ~5.5 mm per side), close on the plug at a 2.5 mm/side
    over-close, lift it 1.8 cm clear of the guide walls, carry it 12 cm to the sound socket in
    two slow stages, lower it into that well, open in place so the pads leave the plug's faces,
    then retreat straight up. One fixed script; the outcome is emergent physics."""
    seat_z = SEAT_Z                        # TCP at the plug's centre: the pads' band covers it
    right = [
        Waypoint(1.0, (LX, OY - 0.13, TOP + 0.21), -0.047, "approach high in front of the scorched socket", yaw=0.0),
        Waypoint(0.6, (LX, OY, TOP + 0.21), -0.025, "slide over the plug at the well-fit aperture", yaw=0.0),
        Waypoint(0.5, (LX, OY, TOP + 0.185), -0.025, "stair down at the well-fit aperture", yaw=0.0),
        Waypoint(0.8, (LX, OY, seat_z), -0.025, "descend into the well beside the plug", yaw=0.0),
        Waypoint(0.6, (LX, OY, seat_z), -0.025, "settle", yaw=0.0),
        Waypoint(0.8, (LX, OY, seat_z), -0.015, "close: 2.5 mm/side over-close grips the plug", yaw=0.0),
        Waypoint(1.0, (LX, OY, seat_z + 0.018), -0.015, "lift the plug clear of the guide walls", yaw=0.0),
        Waypoint(1.2, (LX - 0.06, OY, seat_z + 0.018), -0.015, "carry stage 1 toward the sound socket", yaw=0.0),
        # The harness path lags a 12 cm carry by ~2.3 cm (measured, run 4: the plug
        # ended 2.28 cm short in x and 8 mm short in y, jamming on the well wall),
        # so command the drop past the well's centre by the measured lag.
        Waypoint(1.2, (RX + 0.023, OY + 0.008, seat_z + 0.018), -0.015, "carry stage 2 to the sound socket (lag-compensated)", yaw=0.0),
        Waypoint(1.0, (RX + 0.023, OY + 0.008, seat_z + 0.002), -0.015, "lower the plug into the sound well (lag-compensated)", yaw=0.0),
        Waypoint(0.6, (RX + 0.023, OY + 0.008, seat_z + 0.002), -0.015, "hold at the seat", yaw=0.0),
        Waypoint(0.8, (RX + 0.023, OY + 0.008, seat_z + 0.002), -0.047, "open: the plug settles to its seat", yaw=0.0),
        Waypoint(1.0, (RX + 0.023, OY + 0.008, seat_z + 0.04), -0.047, "retreat straight up", yaw=0.0),
        Waypoint(1.0, (RX + 0.023, OY + 0.13, TOP + 0.21), -0.047, "back off", yaw=0.0),
    ]
    return {
        "right": right,
        # Parked at the left arm's own READY pose: a waypoint out across the
        # table puts the left forearm across the right arm's elbow corridor and
        # the right arm's descent stalls on it (measured on the damper task).
        "left": [Waypoint(5.5, (0.213, 0.3045, TOP + 0.203), -0.047, "parked at ready")],
    }

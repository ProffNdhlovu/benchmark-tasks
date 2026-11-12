"""A pull-chain fixture sits on the station table as a reassembly job: the bulb has fallen out of
its twist-lock socket onto the table and the pull rod has slipped off its pivot saddle onto the
table. NSPIRE lighting check (D125): a permanently installed light fixture that is inoperable is
deficient - the technician's fix is physical: reseat the bulb, re-hook the chain, and the fixture is
whole again. This rig scores the two reseatings directly by final position - the same check a
technician makes by eye - so no credit is possible without doing the work.

Run it (single fixed layout):
    inspect-robots run --embodiment yam_arena --policy yam_scripted \\
      -E task=pullchain_light_fix.py -E scene=station -E object_jitter=0 \\
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
      --instruction "reseat the fallen bulb in its socket and lay the pull rod back on its saddle" \\
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/bulb-demo
"""

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"

TOP = TABLE_TOP[SCENE]

# Fixture on a short post (kept low and close to the arm: the reach ceiling is ~0.94 m and ~0.90 m
# at this x): socket on top of the post, pivot saddle on the table beside it.
FX, FY = 0.38, -0.11
POST = Cylinder("fixture_post", pos=(FX, FY, TOP + 0.02), radius=0.014, height=0.04,
                color=(0.55, 0.57, 0.60))
SOCKET = Cylinder("bulb_socket", pos=(FX, FY, TOP + 0.055), radius=0.012, height=0.03,
                  color=(0.75, 0.76, 0.78))   # sits on the post; narrower than the bulb so the
                  # bulb rests on it instead of dropping inside it
BULB = Box("light_bulb", pos=(0.36, -0.02, TOP + 0.02), size=(0.028, 0.028, 0.04),
           mass=0.05, color=(0.92, 0.88, 0.65))   # fallen out, standing on the table (square-bodied:
           # the sim's flat finger pads cannot hold a cylinder)
SADDLE = Box("pivot_saddle", pos=(FX - 0.10, FY, TOP + 0.005), size=(0.06, 0.04, 0.01),
             color=(0.30, 0.31, 0.33))   # the rod's pivot saddle, on the table beside the post
ROD = Box("pull_rod", pos=(FX - 0.10, FY + 0.12, TOP + 0.01), size=(0.04, 0.02, 0.02), mass=0.04,
          color=(0.65, 0.67, 0.70))   # slipped off its saddle, lying on the table

OBJECTS = [POST, SOCKET, BULB, SADDLE, ROD]

BULB_Z = TOP + 0.09     # seated: the bulb's centre = socket top (+7 cm) + half its 4 cm height
ROD_Z = TOP + 0.02      # re-seated: the rod's centre = saddle top (+1 cm) + half its 2 cm height
SEAT_TOL = 0.005
ROD_XY_TOL = 0.02


def _bulb_seated(objects):
    b = objects["light_bulb"]
    return bool(abs(b.pos[2] - BULB_Z) < SEAT_TOL and abs(b.pos[0] - FX) < 0.008
                and abs(b.pos[1] - FY) < 0.008 and b.resting())


def _rod_seated(objects):
    r = objects["pull_rod"]
    qx, qy, qz, qw = r.quat_xyzw
    tilt = abs(2.0 * (qy * qz + qw * qx)) + abs(2.0 * (qx * qy + qw * qz))
    return bool(abs(r.pos[2] - ROD_Z) < SEAT_TOL and abs(r.pos[0] - (FX - 0.10)) < ROD_XY_TOL
                and abs(r.pos[1] - FY) < ROD_XY_TOL and tilt < 0.12 and r.resting())


def score(objects):
    """0.5 for the bulb resting centred on the socket, 0.5 for the pull rod resting level across
    its saddle, 1.0 for both. Final positions only: a part knocked off later loses its credit.
    Do-nothing: 0."""
    import csv, os as _os
    if _os.path.exists("/tmp/bulb_dbg_on"):
        b = objects["light_bulb"]
        r = objects["pull_rod"]
        row = [round(b.pos[0], 4), round(b.pos[1], 4), round(b.pos[2], 4), _bulb_seated(objects),
               round(r.pos[0], 4), round(r.pos[1], 4), round(r.pos[2], 4), _rod_seated(objects)]
        with open(f"/tmp/pull_{_os.getpid()}.csv", "a") as f:
            csv.writer(f).writerow(row)
    return 0.5 * _bulb_seated(objects) + 0.5 * _rod_seated(objects)


def success(objects):
    return score(objects) >= 1.0


SCORE_OVER = "end"      # final positions are what is scored


def _pick_place(x, y, grasp_z, place_x, place_y, place_z, carry_z):
    """Station-paced top-down pick and place with settle holds (slow segments keep the held part
    from swinging in the grip)."""
    return [
        Waypoint(1.0, (x, y, grasp_z + 0.07), -0.04695, "hover above the part", yaw=0.0),
        Waypoint(2.0, (x, y, grasp_z + 0.07), -0.04695, "wait at the hover", yaw=0.0),
        Waypoint(2.0, (x, y, grasp_z), -0.04695, "descend onto the part", yaw=0.0),
        Waypoint(1.0, (x, y, grasp_z), 0.0, "close on the part", yaw=0.0),
        Waypoint(2.0, (x, y, carry_z), 0.0, "lift the part", yaw=0.0),
        Waypoint(1.0, (x, y, carry_z), 0.0, "hold lifted", yaw=0.0),
        Waypoint(2.0, (place_x, place_y, carry_z), 0.0, "carry", yaw=0.0),
        Waypoint(1.0, (place_x, place_y, carry_z), 0.0, "hold at the target", yaw=0.0),
        Waypoint(2.0, (place_x, place_y, place_z), 0.0, "lower into place", yaw=0.0),
        Waypoint(0.8, (place_x, place_y, place_z), -0.04695, "release", yaw=0.0),
        Waypoint(1.5, (place_x, place_y - 0.09, carry_z), -0.04695, "retreat", yaw=0.0),
    ]


def demo():
    """The right arm lays the pull rod back across its saddle first, then reseats the bulb in its
    socket - the two reassembly moves a technician makes, in a fixed order. All grips are yaw 0."""
    rx, ry = ROD.pos[0], ROD.pos[1]
    rod_place_z = ROD_Z + 0.013                    # release 1.3 cm above the saddle: it drops on
    bx, by = BULB.pos[0], BULB.pos[1]
    bulb_place_z = BULB_Z + 0.005                  # release just above the socket: it drops on
    carry = TOP + 0.09             # high carry: the rod's long body clears the fallen bulb
    right = _pick_place(rx, ry, ROD.pos[2], FX - 0.10, FY, rod_place_z, carry)
    right += _pick_place(bx, by, BULB.pos[2], FX, FY, bulb_place_z, carry)
    return {
        "right": right,
        "left": [Waypoint(sum(w.duration for w in right), (0.20, 0.20, TOP + 0.08), -0.04695, "parked")],
    }

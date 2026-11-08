"""A small return-air grille sits on the station table: a grey frame whose square cartridge slot
opens toward the right arm; a clogged filter cartridge sits in the slot, its face flush with the
frame's mouth. A clean replacement cartridge lies on the table. NSPIRE ventilation check: a return
with a clogged filter fails; the fix is pulling the dirty cartridge out and seating a clean one
flush in the slot — both verifiable with a photo and a tape measure.

Run the demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=hvac_filter_swap.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "swap the dirty air filter for the clean one and seat it flush" \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/hvac-demo
"""

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"

TOP = TABLE_TOP[SCENE]

# Grille frame, sitting on the table: two jambs, a sill and a back stop form a 5.5 cm deep square
# slot (7.5 cm wide inside, open above), mouth facing -y. The cartridge slides along y: out
# through the mouth, in until its back face seats on the stop. The grasp happens at the mouth.
FX = 0.36
JAMB_L = Box("jamb_left", pos=(FX - 0.0425, -0.0475, TOP + 0.0325), size=(0.01, 0.055, 0.065),
             color=(0.45, 0.47, 0.50))
JAMB_R = Box("jamb_right", pos=(FX + 0.0425, -0.0475, TOP + 0.0325), size=(0.01, 0.055, 0.065),
             color=(0.45, 0.47, 0.50))
SILL = Box("sill", pos=(FX, -0.0475, TOP + 0.005), size=(0.095, 0.055, 0.01),
           color=(0.45, 0.47, 0.50))
BACK = Box("slot_back", pos=(FX, -0.0175, TOP + 0.0325), size=(0.095, 0.005, 0.065),
           color=(0.45, 0.47, 0.50))

# The cartridge filters: 5.2 x 5.2 x 6 cm square cassettes (the tub's twin - the jaw's proven
# grasp class), 80 g, resting on the sill. Seated, the back face is flush with the stop and the
# face flush with the mouth.
HOME_Y = -0.0455
FILTER_Z = TOP + 0.04
DIRTY = Box("dirty_filter", pos=(FX, HOME_Y, FILTER_Z), size=(0.052, 0.052, 0.06), mass=0.08,
            color=(0.55, 0.42, 0.28))
CLEAN_HOME = (0.43, -0.14, TOP + 0.032)
CLEAN = Box("clean_filter", pos=CLEAN_HOME, size=(0.052, 0.052, 0.06), mass=0.08,
            color=(0.93, 0.93, 0.95))

OBJECTS = [JAMB_L, JAMB_R, SILL, BACK, DIRTY, CLEAN]

MOUTH_Y = -0.075                # the frame's front plane (the jambs' -y faces)
POCKET_BACK_Y = -0.02           # the stop's -y face: the seated cartridge's back rests here
OUT_Y = MOUTH_Y - 0.026         # "out" = the centre past its half-depth beyond the mouth plane;
                                # the slot opens toward -y, so out is BELOW the mouth line
SEAT_TOL = 0.012
TABLE_Z_MAX = TOP + 0.045


def score(objects):
    """0.5 for the dirty cartridge pulled out and resting on the table in front of the grille;
    +0.5 for the clean cartridge seated flush in the slot; 1.0 for both."""
    d, c = objects["dirty_filter"], objects["clean_filter"]
    dirty_out = (d.pos[1] <= OUT_Y and d.pos[2] < TABLE_Z_MAX
                 and d.resting() and abs(d.pos[0] - FX) < 0.20)
    clean_in = (abs(c.pos[0] - FX) < SEAT_TOL and abs(c.pos[1] - HOME_Y) < SEAT_TOL
                and abs(c.pos[2] - FILTER_Z) < SEAT_TOL and c.resting())
    return (0.5 if dirty_out else 0.0) + (0.5 if clean_in else 0.0)


def success(objects):
    return score(objects) >= 1.0


SCORE_OVER = "best"     # a seated cartridge stays seated; the pulled cartridge stays out


def demo():
    """Vessel-style grasps (direct landing at the object's centre, jaw fully open, close to the
    sweep-proven 5 mm/side crush on the 5.2 cm width). The right arm pulls the dirty cartridge out
    through the mouth, sets it down clear of the insert path; then grasps the clean cartridge,
    carries it to the mouth at seat height and pushes it in until the back stop seats it flush."""
    open_slot, open_full, grip = -0.026, -0.04695, -0.016      # open_ ~5.9 cm: the jaw slides into the 7.5 cm slot between
                                      # the jambs (the full-open jaw is wider than the frame and
                                      # lands on top of it); grip = the sweep-proven 5 mm/side
                                      # crush on the 5.2 cm width
    setdown = (0.24, -0.16, TOP + 0.032)
    mouth_y = -0.11                   # outside the mouth, cassette at seat height, for the insert
    carry_z = TOP + 0.20
    right = [
        # pull the dirty cartridge out of the slot
        Waypoint(2.0, (FX, mouth_y, FILTER_Z), open_slot, "approach the mouth at jaw-fit aperture", yaw=0.0),
        Waypoint(1.2, (FX, HOME_Y, FILTER_Z), open_slot, "slide the jaw in over the cassette", yaw=0.0),
        Waypoint(0.8, (FX, HOME_Y, FILTER_Z), grip, "close on the dirty cartridge", yaw=0.0),
        Waypoint(1.0, (FX, HOME_Y, FILTER_Z + 0.045), grip, "lift it clear of the sill", yaw=0.0),
        Waypoint(1.0, (FX, -0.065, FILTER_Z + 0.065), grip, "draw it out toward the mouth 1/2", yaw=0.0),
        Waypoint(1.0, (FX, -0.105, FILTER_Z + 0.085), grip, "draw it out toward the mouth 2/2", yaw=0.0),
        # carry it clear and set it down on the table
        Waypoint(1.2, (setdown[0], setdown[1], carry_z), grip, "carry it clear of the grille", yaw=0.0),
        Waypoint(1.2, (setdown[0], setdown[1], setdown[2] + 0.04), grip, "over the drop spot", yaw=0.0),
        Waypoint(1.2, setdown, grip, "lower it onto the table", yaw=0.0),
        Waypoint(0.8, setdown, open_full, "release the dirty cartridge", yaw=0.0),
        Waypoint(1.0, (setdown[0], setdown[1], carry_z), open_full, "retreat up", yaw=0.0),
        # pick up the clean cartridge and insert it
        Waypoint(1.5, (CLEAN_HOME[0], CLEAN_HOME[1], carry_z), open_full, "hover over the clean cartridge", yaw=90.0),
        Waypoint(1.5, (CLEAN_HOME[0], CLEAN_HOME[1], CLEAN_HOME[2]), open_full, "descend vertically onto it", yaw=90.0),
        Waypoint(0.8, (CLEAN_HOME[0], CLEAN_HOME[1], CLEAN_HOME[2]), grip, "close on the clean cartridge", yaw=90.0),
        Waypoint(1.5, (CLEAN_HOME[0], CLEAN_HOME[1], carry_z), grip, "lift it", yaw=90.0),
        Waypoint(1.5, (FX, mouth_y, carry_z), grip, "carry it to the mouth", yaw=90.0),
        Waypoint(1.5, (FX, HOME_Y, carry_z), grip, "push it in above the slot opening", yaw=90.0),
        Waypoint(1.2, (FX, HOME_Y, FILTER_Z), grip, "lower it onto the back stop", yaw=90.0),
        Waypoint(0.5, (FX, HOME_Y, FILTER_Z), grip, "hold it home", yaw=90.0),
        Waypoint(0.8, (FX, HOME_Y, FILTER_Z), open_full, "release", yaw=90.0),
        Waypoint(1.0, (FX, mouth_y, FILTER_Z), open_full, "back away", yaw=90.0),
    ]
    return {
        "right": right,
        "left": [Waypoint(sum(w.duration for w in right), (0.20, 0.20, TOP + 0.08), -0.040, "parked")],
    }

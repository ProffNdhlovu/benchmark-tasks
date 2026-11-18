"""Wood-stove slide-gate flue damper: carry the gate aside off the flue opening.

A wood stove sits on the station table. Its top plate carries the flue opening
(a 6 cm square hole framed by the top plate) and the polished-steel slide-gate
damper lies over it, covering the flue. A damper left closed fills the room
with smoke on the first fire of the season; uncovering the flue before lighting
is the check every stove operator does.

The robot straddles the gate with its open jaw, grips the gate's two side
faces (a 2.5 cm-thick gate presents tall faces to the pads), and carries the
gate sideways along the stove's top until the flue opening is uncovered, then
opens the jaw and retreats. The gate is a free rigid body throughout: closed,
it rests on the top plate bridging the opening; open, it rests displaced on
the same surface with the flue exposed.

Scoring (at episode end, the gate at rest on the stove's top): 1.0 when the
gate has moved at least 7.5 cm from its closed position, leaving the flue
opening uncovered; 0.5 for a 3.5-7.5 cm move (the flue partly exposed); 0.0
for less, or for a gate off the stove's top (fallen, carried away). Do-nothing:
0 — the closed spawn is stable and cannot uncover the flue on its own.

Fidelity: the gate's mass is set to 0.6 kg (a real 2.5 cm cast-iron plate this
size is ~1.5 kg; the reduction keeps the carry inside the jaw's hold budget),
and its finish is rendered as polished steel so the plate reads clearly on the
dark stove top. The flue is a framed opening rather than a piped stack, and the
smoke path is not simulated - the gate's displacement off the opening is
exactly what a photo checks.
"""
import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Geometry (cm above the table): the stove body tops at 18, the top plate's
# strips run 18..19.5 and frame a 6 cm square flue opening centred at
# (0.37, -0.16). The gate (7.8 x 9.4 x 2.5 cm) lies on the strips' top covering
# the opening (0.9 cm of bearing per side), its side faces spanning 19.5..22.
# The grasp closes along x on those faces with the TCP commanded 10 cm above
# the table - far below the arm's floor, so the dive stalls at the strips
# (the finger tips land on them at TCP ~0.955) with the pads' band covering
# the FULL 2.5 cm of each face. The grasp sits ~40 cm from the right arm's
# base (inside the 39-46 cm window) and, critically, at y=-0.16: reaching
# shallower (y=-0.11) swings the elbow back through the station rig's camera
# stand (x -0.26..-0.18, z to 1.2) and the descent stalls on it - measured
# probe15/18a vs the working demo5 at this spot. The carry slides the gate
# TOWARD the arm (-x): the gate keeps its centre on the left strip through
# the whole slide, so it stays flat.
FIX = (0.37, -0.16)
BODY_H = 0.18
PLATE_T = 0.015            # the top strips' thickness
HOLE = 0.06                # the flue opening's side
GATE_TOP_Z = BODY_H + PLATE_T          # 16.5: the strips' top, where the gate rides
GATE = (0.078, 0.094, 0.025)           # the gate: 7.8 wide (x), 9.4 deep (y), 2.5 thick
GATE_Z = GATE_TOP_Z + GATE[2] / 2      # 17.75: the gate's centre while riding

STEEL = (0.45, 0.46, 0.48)
POLISHED = (0.62, 0.63, 0.65)
CAST = (0.35, 0.36, 0.38)

def _bx(name, dx, dy, dz, x, y, z, color):
    return Box(name, pos=(x, y, TOP + z), size=(dx, dy, dz), color=color)

STOVE_BODY = _bx("stove_body", 0.20, 0.16, BODY_H, FIX[0], FIX[1], BODY_H / 2, CAST)
# The top plate: four strips framing the flue opening (the border-frame build).
STRIP_L = _bx("top_strip_left", 0.07, 0.16, PLATE_T, FIX[0] - 0.065, FIX[1], BODY_H + PLATE_T / 2, STEEL)
STRIP_R = _bx("top_strip_right", 0.07, 0.16, PLATE_T, FIX[0] + 0.065, FIX[1], BODY_H + PLATE_T / 2, STEEL)
STRIP_F = _bx("top_strip_front", HOLE, 0.05, PLATE_T, FIX[0], FIX[1] - 0.055, BODY_H + PLATE_T / 2, STEEL)
STRIP_B = _bx("top_strip_back", HOLE, 0.05, PLATE_T, FIX[0], FIX[1] + 0.055, BODY_H + PLATE_T / 2, STEEL)

# The slide-gate damper: CLOSED, centred on the flue opening.
DAMPER_GATE = Box("damper_gate", pos=(FIX[0], FIX[1], TOP + GATE_Z),
                  size=GATE, mass=0.6, color=POLISHED)

OBJECTS = [STOVE_BODY, STRIP_L, STRIP_R, STRIP_F, STRIP_B, DAMPER_GATE]

CLOSED_X = FIX[0]
FULL_MOVE = 0.075          # 1.0 at >= 7.5 cm: the flue opening uncovered
HALF_MOVE = 0.035          # 0.5 at >= 3.5 cm
RIDE_Z = GATE_Z            # the gate's centre z while riding the strips


def score(objects):
    """1.0 when the gate rests on the stove's top moved >= 7.5 cm off the flue;
    0.5 for a 3.5-7.5 cm move; 0.0 otherwise (closed, fallen, carried off)."""
    p = objects["damper_gate"]
    if not p.resting():
        return 0.0
    if abs(p.pos[2] - (TOP + RIDE_Z)) > 0.02:   # off the strips: fallen or carried
        return 0.0
    move = abs(p.pos[0] - CLOSED_X)
    if move >= FULL_MOVE:
        return 1.0
    if move >= HALF_MOVE:
        return 0.5
    return 0.0


def success(objects):
    return score(objects) >= 1.0


def demo():
    """Approach the gate from its front (-y) at face height, slide the open
    jaw +y (perpendicular to the closing axis) until the pads sit beside the
    gate's side faces, close to a ~8.7 cm aperture (4.5 mm/side crush on the
    7.8 cm width), carry the gate toward the arm (-x) in eight ~1 cm stages,
    open and retreat straight up. yaw 90 deg: the jaw's closing axis turns to
    world x.

    Two deliberate choices: the jaw slides in perpendicular to its closing
    axis, so the pads never push the gate (an +x slide would run the right pad
    into the gate's left face and shove the gate off the stove); and the whole
    approach stays at face height, because an overhead descent overshoots in
    the runner's camera-render path (the arm sat ~2 cm high at the hover) and
    its finger tips then rested ON the gate's top - the weak wrist PD
    (stiffness 10) cannot push through 0.6 kg of gate, so the plan stalled.
    """
    tcpx = FIX[0]
    tcp_y = FIX[1]
    slide_z = TOP + 0.21                  # tips 2 mm above the strips: no strip contact, so the
                                          # slide can't wedge the gate +y out of the jaw (v12's
                                          # failure); pads' band still covers 2.3 cm of each face
    open_f = -0.047                       # full open: the ~9.0 cm aperture clears the 7.8 cm gate by 6 mm/side,
                                          # past PhysX's 5 mm contact offset - no graze during the slide
    grip_f = -0.035                       # over-close: the pads squeeze the 7.8 cm gate hard (the PD's
                                          # residual force holds through the carry; v15's gentle
                                          # -0.0455 let the gate slide 3.6 cm in the pads)
    move = 0.11                           # delivered move = command - ~3 cm (lag+slip, v16/v17); 11 cm
                                          # -> the gate's centre ~0.29: an ~8 cm move (score 1.0) with
                                          # its CoM still 2 cm inside the left strip's edge (v17's
                                          # longer carry left the CoM 4 mm from the edge and the
                                          # release's nudge tipped it off the stove)
    stages = 8
    yaw = 1.5708                          # Rz(90) turns the jaw's closing axis to world x: the pads press the side faces
    side_y = tcp_y - 0.12                 # in front of the stove, the jaw clear of the gate
    x_comp = tcpx - 0.012                 # the slide's end overshoots +1.2 cm (v12/v13/v14: 0.381/0.381/0.382);
                                          # commanding short lands the jaw centred on the gate
    wps = [
        Waypoint(1.0, (tcpx, side_y, TOP + 0.22), open_f, "approach high in front of the stove", yaw=yaw),
        # Staircase: the runner's IK is warm-started per step and stalls at the
        # wrist pitch limit if a waypoint jumps far below the arm's floor; small
        # steps track. These lower the jaw in front of the stove (over the
        # table) to face height.
        Waypoint(0.5, (tcpx, side_y, TOP + 0.20), open_f, "stair down 1/2", yaw=yaw),
        Waypoint(0.5, (tcpx, side_y, slide_z + 0.01), open_f, "stair down 2/2", yaw=yaw),
        Waypoint(0.8, (tcpx, side_y, slide_z), open_f, "settle at face height in front of the stove", yaw=yaw),
        Waypoint(2.5, (x_comp, tcp_y, slide_z), open_f, "slide +y: the open jaw straddles the gate", yaw=yaw),
        Waypoint(1.0, (x_comp, tcp_y, slide_z), open_f, "settle: the slide's overshoot relaxes before closing", yaw=yaw),
        Waypoint(0.8, (x_comp, tcp_y, slide_z), grip_f, "close hard on the side faces", yaw=yaw),
    ]
    step_t = 6.4 / stages                 # slow stages: less yank, the pads' grip doesn't slide
    for i in range(1, stages + 1):
        x = tcpx - move * i / stages
        wps.append(Waypoint(step_t, (x, tcp_y, slide_z), grip_f,
                            f"carry stage {i}/{stages}: uncover the flue", yaw=yaw))
    end_x = tcpx - move
    wps.append(Waypoint(0.8, (end_x, tcp_y, slide_z), open_f, "release: the flue stands uncovered", yaw=yaw))
    wps.append(Waypoint(1.0, (end_x, tcp_y, TOP + 0.24), open_f, "retreat straight up", yaw=yaw))
    return {
        "right": wps,
        # Parked at the left arm's own READY pose: a waypoint out at
        # (0.20, 0.20) puts the left forearm across the right arm's elbow
        # corridor at the shallower grasp (cross-arm collisions are on), and
        # the right arm's descent stalls on it (probe15 vs probe13).
        "left": [Waypoint(5.5, (0.213, 0.3045, TOP + 0.203), -0.047, "parked at ready")],
    }

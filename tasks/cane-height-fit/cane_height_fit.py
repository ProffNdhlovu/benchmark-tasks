"""A mobility-aid fitting bench: an adjustable square-tube cane lies on two wooden cradle rails on the table with
its lower shaft half retracted, next to a steel ruler marked in centimetres. A client's fitted height has been
transferred from the fitting chart to the bench as a red line across the ruler at 80.0 cm. Fitting the cane means
pulling the lower shaft out of the upper until it stops with the rubber foot on that red line (the button's
shoulder seats against the hole's edge there and the shaft can come no further out), then shoving the shaft once
toward the handle, working it back out to the stop to confirm it still seats, and letting go. The shaft can sit
anywhere along its travel and stays where it is left, so the fit is only right if the fitter lands the foot on the
red line: one hole short (77.5 cm) or past it is a misfit.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (modeled props, mechanisms, gripper states; hooks yam_arena.scene on import)
import guardkit as gk  # noqa: E402  (the gripper's own shape, for the let-go checks)
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("Fit the client's cane on the bench: pull the lower shaft out until it stops with the rubber foot on "
               "the red line, shove it in once and work it back out to the stop, then let go.")


# -- the fitting bench (world frame: x forward, y left, z up; the table top at TOP) -----------
# The cane lies on two cradle rails with its axis along the base frame's +Y; the base frame is the world frame turned
# -30 degrees about z, so the cane runs diagonally across the table: its handle toward the table's front left, its
# foot end toward the left arm's front. The left arm does the fitting: it grips the exposed lower shaft a fixed 5 cm
# back from the rubber foot and pulls it out along the cane's axis, which keeps the grip point inside the left arm's
# comfortable reach for the whole pull (0.43-0.44 m).

YAW = -math.pi / 6                       # the cane's frame: +Y along the cane, turned -30 deg from world +y
UX, UY = -math.sin(YAW), math.cos(YAW)   # the cane's axis in world x, y: (0.5, 0.866), out of the mouth
MX, MY = 0.33, 0.02                      # the mouth's middle in world x, y
CZ = TOP + 0.034                         # the cane's centre height: 2 cm rails + half the 2.8 cm outer tube
OUTER = 0.55                             # the upper shaft, from the mouth back to the handle's end
SLIDE = 0.34                             # the lower shaft's length
STEP = 0.025                             # the button holes every 2.5 cm, 2.5 to 27.5 cm of extension
HOLES = tuple(round(STEP * k, 4) for k in range(1, 12))
MARK = 0.800                             # the red line: 80.0 cm from the handle's end
TARGET = round(MARK - OUTER, 4)          # 0.250 m of extension puts the foot at the line (the end of travel)
START = 0.075                            # the cane ships in its third hole
GRIP_BACK = 0.05                         # the grip sits this far back from the rubber foot
GRIP_YAW = YAW                           # the pads' closing axis across the cane: perpendicular to it


class Detents:
    """Unused: the detent is modelled as the shaft's end of travel (the link's upper limit at the marked hole), the
    way a real button's shoulder seats against the hole's edge. Kept out of the mechanism because the actuator
    path in this sim cannot carry a reliable spring-damper drive on this joint."""

    def __init__(self, holes, stiffness: float = 80.0, damping: float = 4.0):
        self.holes = tuple(holes)
        self.stiffness = float(stiffness)
        self.damping = float(damping)

    def drive(self, key, q: float, qd: float, jq, mem):
        nearest = min(self.holes, key=lambda h: abs(h - q))
        return (q, 0.0, -self.stiffness * (q - nearest) - self.damping * qd, None)


# Looks
ALUM = kit.Look((0.64, 0.66, 0.68), 0.35, metallic=0.85)   # brushed aluminium shafts
RUBBER = kit.Look((0.05, 0.05, 0.05), 0.85)                # the foot, the button, the ruler's ticks
GRIPLOOK = kit.Look((0.10, 0.10, 0.11), 0.6)               # the handle
PLY = kit.Look((0.74, 0.62, 0.45), 0.85)                   # the cradle rails
STEEL = kit.Look((0.75, 0.76, 0.78), 0.3, metallic=0.9)    # the ruler
RED = kit.Look((0.85, 0.10, 0.08), 0.5)                    # the client's line


def _wall_boxes():
    """(center, size, look) for every fixed box of the cane's upper shaft, base frame: the button-side wall with a
    1.4 cm wide, 1.2 cm tall window at every hole (so the button shows through), the far-side wall, the top and
    bottom walls, and the handle. Drives the meshes, the colliders and the scorer's contact shapes alike."""
    xw = -0.0145
    boxes = [((xw, -0.2725, zc), (0.003, 0.555, 0.008), ALUM) for zc in (-0.010, 0.010)]  # strips below/above
    edges = [-0.55] + sorted([round(h - 0.28 - 0.007, 4) for h in HOLES]
                             + [round(h - 0.28 + 0.007, 4) for h in HOLES]) + [0.005]
    boxes += [((xw, (a + b) / 2, 0.0), (0.003, b - a, 0.012), ALUM)
              for a, b in zip(edges[::2], edges[1::2]) if b - a > 1e-4]                    # middles between windows
    boxes += [((0.0145, -0.2725, 0.0), (0.003, 0.555, 0.028), ALUM),    # the far-side wall
              ((0.0, -0.2725, 0.0125), (0.028, 0.555, 0.003), ALUM),    # the top wall
              ((0.0, -0.2725, -0.0125), (0.028, 0.555, 0.003), ALUM),   # the bottom wall
              ((0.0, -0.566, 0.028), (0.11, 0.032, 0.026), GRIPLOOK)]   # the handle, across the shaft's end
    return boxes


WALL_BOXES = _wall_boxes()
WALL_MESHES = tuple(kit.box(c, s, lk) for c, s, lk in WALL_BOXES)
WALL_COLLIDERS = tuple(kit.Collider(c, size=s) for c, s, lk in WALL_BOXES)

CANE = kit.Mechanism(
    name="cane", pos=(MX, MY, CZ), yaw=YAW, size=(0.11, 0.60, 0.07),   # no mass: fixed to the table
    meshes=WALL_MESHES, colliders=WALL_COLLIDERS,
    links=(kit.Link(
        name="lower", kind="prismatic", axis="Y", anchor=(0.0, 0.0, 0.0),
        lower=0.0, upper=TARGET, start=START, mass=0.15,   # the upper limit IS the detent: the button's shoulder
                                                           # seats against the hole's edge at the marked hole
        meshes=(kit.box((0.0, -0.17, 0.0), (0.024, 0.34, 0.024), ALUM),        # the lower shaft
                kit.box((-0.019, -0.28, 0.0), (0.014, 0.012, 0.012), RUBBER),  # the spring button
                kit.box((0.0, -0.0175, 0.0), (0.032, 0.035, 0.028), RUBBER)),  # the rubber foot
        colliders=(kit.Collider((0.0, -0.17, 0.0), size=(0.024, 0.34, 0.024)),
                   kit.Collider((0.0, -0.0175, 0.0), size=(0.032, 0.035, 0.028))),
        damping=0.5,
    ),),
)

CRADLE = kit.Prop(
    name="cradle", pos=(MX, MY, CZ), yaw=YAW, size=(0.10, 0.64, 0.02),  # no mass: fixed
    meshes=(kit.box((0.019, -0.2875, -0.0245), (0.02, 0.585, 0.019), PLY),
            kit.box((-0.019, -0.2875, -0.0245), (0.02, 0.585, 0.019), PLY)),
    colliders=(kit.Collider((0.019, -0.2875, -0.0245), size=(0.02, 0.585, 0.019)),
               kit.Collider((-0.019, -0.2875, -0.0245), size=(0.02, 0.585, 0.019))),
)

_RULER_X = round(MX - 0.14 * UX - 0.045 * math.cos(YAW), 4)   # the ruler sits 4.5 cm out on the button side
_RULER_Y = round(MY - 0.14 * UY - 0.045 * math.sin(YAW), 4)   # of the cane, level with the handle's end at 0
TICKS = tuple(kit.box((0.0, round(-0.425 + 0.05 * k, 4), 0.004), (0.03, 0.002, 0.002), RUBBER)
              for k in range(17))
RULER = kit.Prop(
    name="ruler", pos=(_RULER_X, _RULER_Y, TOP + 0.003), yaw=YAW, size=(0.05, 0.85, 0.01),  # no mass: fixed
    meshes=(kit.box((0.0, 0.0, 0.0), (0.05, 0.85, 0.006), STEEL),) + TICKS
            + (kit.box((0.0, 0.35, 0.004), (0.05, 0.004, 0.002), RED),),   # the client's line at 77.5 cm
    colliders=(kit.Collider((0.0, 0.0, 0.0), size=(0.05, 0.85, 0.006)),),
)

OBJECTS = [CANE, CRADLE, RULER]

# -- scoring ----------------------------------------------------------------------------------

FIT_TOL = 0.004    # m: the shaft in the marked hole (the detent pulls it to the hole's centre)
PUSH = 0.006       # m: the hold test shoves the shaft at least this far off the hole and it must come back
STILL = 0.01       # m/s: the shaft counts as still under this

LOWER_PARTS = (gk.Block((0.0, -0.17, 0.0), (0.024, 0.34, 0.024)), gk.Block((0.0, -0.0175, 0.0), (0.032, 0.035, 0.028)))
CANE_PARTS = tuple(gk.Block(c, s) for c, s, lk in WALL_BOXES)   # guardkit shapes, not kit.Colliders

_state = {"ep": -1, "fitted": False, "max_off": 0.0}


def _trial_reset() -> None:
    ep = kit.episode()
    if ep != _state["ep"]:
        _state.update(ep=ep, fitted=False, max_off=0.0)


def score(objects) -> float:
    """0.5 the fit (the shaft left in the hole that puts the foot on the line), 0.3 the hold test (the shaft was
    shoved at least 6 mm off the hole after the fit and is back inside 4 mm of it at the end), 0.2 the clean let-go
    (no gripper touching the cane and the shaft still at the end). Nothing done scores 0: the cane ships in its
    third hole, 17.5 cm short of the marked one."""
    _trial_reset()
    js = kit.joint(objects, "cane/lower")
    q, off = js.pos, abs(js.pos - TARGET)
    if off <= FIT_TOL:
        _state["fitted"] = True
    if _state["fitted"]:
        _state["max_off"] = max(_state["max_off"], off)
    fit = 0.5 if off <= FIT_TOL else 0.0
    hold = 0.3 if (_state["fitted"] and _state["max_off"] >= PUSH and off <= FIT_TOL) else 0.0
    let_go = 0.0
    if _state["fitted"] and abs(js.vel) < STILL:
        if not gk.touching(objects, "cane", CANE_PARTS) and not gk.touching(objects, "cane/lower", LOWER_PARTS):
            let_go = 0.2
    return fit + hold + let_go


def success(objects) -> bool:
    return score(objects) >= 0.999


def distance(objects) -> float:
    _trial_reset()
    return abs(kit.joint(objects, "cane/lower").pos - TARGET)


# -- the demo ---------------------------------------------------------------------------------

def _tcp(q: float) -> tuple:
    """The grip point on the exposed lower shaft, GRIP_BACK back from the rubber foot, at the shaft's centre."""
    return (round(MX + (q - GRIP_BACK) * UX, 4), round(MY + (q - GRIP_BACK) * UY, 4), round(CZ, 4))


def demo():
    """The left arm grips the exposed lower shaft 5 cm back from the foot, pulls it out in stages to the
    marked hole (the detent holds at each hole as it goes), seats it against the stop, shoves the shaft once
    toward the foot and works it back out to the stop to check the button holds, then lets go and retreats.
    The right arm stays parked."""
    wp = []
    x0, y0, z0 = _tcp(START)
    wp.append(Waypoint(1.2, (x0, y0, round(CZ + 0.07, 4)), -0.020, "hover over the exposed lower shaft", yaw=GRIP_YAW))
    wp.append(Waypoint(0.7, (x0, y0, z0), -0.020, "descend to the shaft", yaw=GRIP_YAW))
    wp.append(Waypoint(0.9, (x0, y0, z0), -0.010, "close on the shaft: 2 mm/side crush", yaw=GRIP_YAW))
    for q in (0.100, 0.125, 0.150, 0.175, 0.200, 0.2575):
        x, y, _ = _tcp(q)
        wp.append(Waypoint(1.2, (x, y, z0), -0.010, f"pull the shaft out to {q * 100:.1f} cm", yaw=GRIP_YAW))
    x, y, _ = _tcp(TARGET - 0.010)
    wp.append(Waypoint(0.8, (x, y, z0), -0.010, "hold test: shove the shaft 1 cm toward the handle", yaw=GRIP_YAW))
    x, y, _ = _tcp(0.2575)
    wp.append(Waypoint(0.9, (x, y, z0), -0.010, "work it back out to the stop", yaw=GRIP_YAW))
    x, y, _ = _tcp(TARGET)
    wp.append(Waypoint(0.9, (x, y, z0), -0.040, "release at the stop", yaw=GRIP_YAW))
    wp.append(Waypoint(0.7, (x, y, z0), -0.040, "let go", yaw=GRIP_YAW))
    wp.append(Waypoint(0.8, (x, y, round(CZ + 0.07, 4)), -0.040, "retreat up clear", yaw=GRIP_YAW))
    total = sum(w.duration for w in wp)
    return {"left": wp,
            "right": [Waypoint(total, (0.20, -0.20, TOP + 0.08), -0.040, "parked")]}

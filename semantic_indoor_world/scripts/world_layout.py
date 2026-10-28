"""Single source of truth for the indoor world and its semantic map.

Every wall, opening, piece of furniture and semantic target is defined here
once. generate_world.py turns this into worlds/my_map.world and
config/semantic_map.yaml, so the two can never disagree. Do not edit the
generated files by hand; change this file and regenerate.

Frame: Gazebo world frame, origin at the building centre, x east (15 m),
y north (20 m), z up. Units: metres, radians. All yaw rotations are about z.
"""

from dataclasses import dataclass
from dataclasses import field
import math

# ----------------------------------------------------------------- building
BUILDING_X = 15.0
BUILDING_Y = 20.0
X_MIN, X_MAX = -BUILDING_X / 2, BUILDING_X / 2          # -7.5 .. 7.5
Y_MIN, Y_MAX = -BUILDING_Y / 2, BUILDING_Y / 2          # -10 .. 10
WALL_T = 0.15                                           # wall thickness
WALL_H = 2.7                                            # wall height
# Outer walls sit inside the 15 x 20 footprint, so the free interior is:
IN_X_MIN, IN_X_MAX = X_MIN + WALL_T, X_MAX - WALL_T     # -7.35 .. 7.35
IN_Y_MIN, IN_Y_MAX = Y_MIN + WALL_T, Y_MAX - WALL_T     # -9.85 .. 9.85

# Openings
DOOR_W = 2.0            # >= 1.2 required; 2.0 keeps a passage with 0.75 m inflation
DOOR_H = 2.1            # lintel above
WINDOW_W = 2.0
WINDOW_SILL = 0.9
WINDOW_H = 1.2
FRAME_BAR = 0.06        # window frame bar width (in the wall plane)
GLASS_T = 0.01

# ------------------------------------------------------------------ rooms
# Interior wall centre lines.
SMALL_EAST_X = -3.5     # small room: x -7.5 .. -3.5 (4 m), y 5 .. 10 (5 m)
SMALL_SOUTH_Y = 5.0
LARGE_WEST_X = 1.5      # large room: x 1.5 .. 7.5 (6 m), y 4 .. 10 (6 m)
LARGE_SOUTH_Y = 4.0

SMALL_DOOR_X = (-6.0, -4.0)         # opening in the small room's south wall
LARGE_DOOR_X = (2.3, 4.3)           # opening in the large room's south wall
SMALL_WINDOW_Y = (6.5, 8.5)         # opening in the west outer wall (small room)
LIVING_WINDOW_Y = (-4.0, -2.0)      # opening in the east outer wall (living room)

# Free floor of each room (inner wall faces).
SMALL_ROOM = (IN_X_MIN, SMALL_EAST_X - WALL_T / 2, SMALL_SOUTH_Y + WALL_T / 2, IN_Y_MAX)
LARGE_ROOM = (LARGE_WEST_X + WALL_T / 2, IN_X_MAX, LARGE_SOUTH_Y + WALL_T / 2, IN_Y_MAX)

# ------------------------------------------------------------ navigation
ROBOT_RADIUS = 0.25
MIN_GOAL_CLEARANCE = 0.5
REQUIRED_CLEARANCE = ROBOT_RADIUS + MIN_GOAL_CLEARANCE      # 0.75 m from collision surfaces
APPROACH = REQUIRED_CLEARANCE + 0.05                        # goal distance from an object face
OBSTACLE_MAX_HEIGHT = 1.0   # collision geometry starting below this blocks the 2D map
GRID_RESOLUTION = 0.05


@dataclass
class Box:
    """One box collision+visual, pose relative to its model."""
    name: str
    size: tuple                 # (sx, sy, sz)
    pose: tuple                 # (x, y, z, yaw) in the model frame
    color: tuple = (0.8, 0.8, 0.8, 1.0)
    collide: bool = True


@dataclass
class Model:
    name: str
    pose: tuple                 # (x, y, z, yaw) in the world frame
    boxes: list = field(default_factory=list)


def _span_box(name, axis, a0, a1, fixed, z0, z1, color, thickness=WALL_T):
    """Wall piece from a0 to a1 along `axis` ('x' or 'y'), centred on `fixed`."""
    length = a1 - a0
    mid = (a0 + a1) / 2
    zc = (z0 + z1) / 2
    if axis == 'x':
        return Box(name, (length, thickness, z1 - z0), (mid, fixed, zc, 0.0), color)
    return Box(name, (thickness, length, z1 - z0), (fixed, mid, zc, 0.0), color)


WALL_COLOR = (0.85, 0.83, 0.78, 1.0)
INNER_WALL_COLOR = (0.78, 0.80, 0.85, 1.0)


def _wall_with_window(prefix, axis, a0, a1, fixed, opening, color):
    """Full-height wall from a0 to a1 with a window opening (sill + head walls)."""
    w0, w1 = opening
    sill_top = WINDOW_SILL
    head_bottom = WINDOW_SILL + WINDOW_H
    return [
        _span_box(f'{prefix}_a', axis, a0, w0, fixed, 0.0, WALL_H, color),
        _span_box(f'{prefix}_b', axis, w1, a1, fixed, 0.0, WALL_H, color),
        _span_box(f'{prefix}_sill', axis, w0, w1, fixed, 0.0, sill_top, color),
        _span_box(f'{prefix}_head', axis, w0, w1, fixed, head_bottom, WALL_H, color),
    ]


def _wall_with_door(prefix, axis, a0, a1, fixed, opening, color):
    d0, d1 = opening
    return [
        _span_box(f'{prefix}_a', axis, a0, d0, fixed, 0.0, WALL_H, color),
        _span_box(f'{prefix}_b', axis, d1, a1, fixed, 0.0, WALL_H, color),
        _span_box(f'{prefix}_lintel', axis, d0, d1, fixed, DOOR_H, WALL_H, color),
    ]


def outer_walls():
    north_y, south_y = Y_MAX - WALL_T / 2, Y_MIN + WALL_T / 2
    west_x, east_x = X_MIN + WALL_T / 2, X_MAX - WALL_T / 2
    boxes = [
        _span_box('north', 'x', X_MIN, X_MAX, north_y, 0.0, WALL_H, WALL_COLOR),
        _span_box('south', 'x', X_MIN, X_MAX, south_y, 0.0, WALL_H, WALL_COLOR),
    ]
    boxes += _wall_with_window('west', 'y', IN_Y_MIN, IN_Y_MAX, west_x, SMALL_WINDOW_Y,
                               WALL_COLOR)
    boxes += _wall_with_window('east', 'y', IN_Y_MIN, IN_Y_MAX, east_x, LIVING_WINDOW_Y,
                               WALL_COLOR)
    return Model('outer_walls', (0.0, 0.0, 0.0, 0.0), boxes)


def small_room_walls():
    half = WALL_T / 2
    boxes = _wall_with_door('south', 'x', IN_X_MIN, SMALL_EAST_X + half, SMALL_SOUTH_Y,
                            SMALL_DOOR_X, INNER_WALL_COLOR)
    boxes.append(_span_box('east', 'y', SMALL_SOUTH_Y + half, IN_Y_MAX, SMALL_EAST_X,
                           0.0, WALL_H, INNER_WALL_COLOR))
    return Model('small_room_walls', (0.0, 0.0, 0.0, 0.0), boxes)


def large_room_walls():
    half = WALL_T / 2
    boxes = _wall_with_door('south', 'x', LARGE_WEST_X - half, IN_X_MAX, LARGE_SOUTH_Y,
                            LARGE_DOOR_X, INNER_WALL_COLOR)
    boxes.append(_span_box('west', 'y', LARGE_SOUTH_Y + half, IN_Y_MAX, LARGE_WEST_X,
                           0.0, WALL_H, INNER_WALL_COLOR))
    return Model('large_room_walls', (0.0, 0.0, 0.0, 0.0), boxes)


FRAME_COLOR = (0.35, 0.25, 0.15, 1.0)
GLASS_COLOR = (0.55, 0.75, 0.95, 0.35)       # alpha < 1 -> semi-transparent


def window(name, centre_xy, yaw):
    """Window model filling a wall opening.

    Model frame: origin at the opening centre on the wall centre line, at the
    opening's mid height; local x runs along the wall, local y through it.
    """
    x, y = centre_xy
    zc = WINDOW_SILL + WINDOW_H / 2
    w, h, b = WINDOW_W, WINDOW_H, FRAME_BAR
    boxes = [
        Box('frame_top', (w, WALL_T, b), (0.0, 0.0, h / 2 - b / 2, 0.0), FRAME_COLOR),
        Box('frame_bottom', (w, WALL_T, b), (0.0, 0.0, -h / 2 + b / 2, 0.0), FRAME_COLOR),
        Box('frame_left', (b, WALL_T, h - 2 * b), (-w / 2 + b / 2, 0.0, 0.0, 0.0), FRAME_COLOR),
        Box('frame_right', (b, WALL_T, h - 2 * b), (w / 2 - b / 2, 0.0, 0.0, 0.0), FRAME_COLOR),
        Box('frame_mullion', (b, WALL_T, h - 2 * b), (0.0, 0.0, 0.0, 0.0), FRAME_COLOR),
        # Glass has collision too: the robot must not pass through the window.
        Box('glass', (w - 2 * b, GLASS_T, h - 2 * b), (0.0, 0.0, 0.0, 0.0), GLASS_COLOR),
    ]
    return Model(name, (x, y, zc, yaw), boxes)


# -------------------------------------------------------------- furniture
BOX_SIZE = 0.7
BOX_XY = (-4.4, 9.0)

BED_XY = (5.4, 8.72)        # model origin = centre of the 1.2 x 2.0 frame
BED_W, BED_L = 1.2, 2.0
HEADBOARD_T = 0.08

TABLE_XY = (0.0, -1.0)
TABLE_L, TABLE_W, TABLE_H = 1.5, 0.8, 0.75
TABLE_TOP_T = 0.04
LEG = 0.06


def small_room_box():
    return Model('small_room_box', (BOX_XY[0], BOX_XY[1], 0.0, 0.0), [
        Box('box', (BOX_SIZE,) * 3, (0.0, 0.0, BOX_SIZE / 2, 0.0), (0.72, 0.52, 0.30, 1.0)),
    ])


def large_room_bed():
    """Headboard on the +y (north) end, against the north wall."""
    return Model('large_room_bed', (BED_XY[0], BED_XY[1], 0.0, 0.0), [
        Box('frame', (BED_W, BED_L, 0.30), (0.0, 0.0, 0.15, 0.0), (0.45, 0.30, 0.18, 1.0)),
        Box('mattress', (BED_W - 0.06, BED_L - 0.10, 0.20), (0.0, -0.03, 0.40, 0.0),
            (0.95, 0.95, 0.92, 1.0)),
        Box('pillow', (0.60, 0.35, 0.12), (0.0, 0.70, 0.56, 0.0), (0.85, 0.88, 1.0, 1.0)),
        Box('headboard', (BED_W, HEADBOARD_T, 0.90),
            (0.0, BED_L / 2 + HEADBOARD_T / 2, 0.45, 0.0), (0.40, 0.26, 0.15, 1.0)),
    ])


def living_room_table():
    leg_h = TABLE_H - TABLE_TOP_T
    lx, ly = TABLE_L / 2 - 0.06, TABLE_W / 2 - 0.06
    boxes = [Box('top', (TABLE_L, TABLE_W, TABLE_TOP_T), (0.0, 0.0, TABLE_H - TABLE_TOP_T / 2, 0.0),
                 (0.55, 0.38, 0.22, 1.0))]
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        boxes.append(Box(f'leg_{i + 1}', (LEG, LEG, leg_h), (sx * lx, sy * ly, leg_h / 2, 0.0),
                         (0.30, 0.22, 0.15, 1.0)))
    return Model('living_room_table', (TABLE_XY[0], TABLE_XY[1], 0.0, 0.0), boxes)


def all_models():
    west_x, east_x = X_MIN + WALL_T / 2, X_MAX - WALL_T / 2
    return [
        outer_walls(),
        small_room_walls(),
        large_room_walls(),
        # Window yaw: local x along the wall (+y world) -> yaw = pi/2 for both.
        window('small_room_window', (west_x, sum(SMALL_WINDOW_Y) / 2), math.pi / 2),
        window('living_room_window', (east_x, sum(LIVING_WINDOW_Y) / 2), math.pi / 2),
        small_room_box(),
        large_room_bed(),
        living_room_table(),
    ]


# ------------------------------------------------------------ semantics
def _rect(x0, x1, y0, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def living_room_polygon():
    """Building interior minus both rooms (incl. their walls), counter-clockwise."""
    half = WALL_T / 2
    return [
        [IN_X_MIN, IN_Y_MIN], [IN_X_MAX, IN_Y_MIN],
        [IN_X_MAX, LARGE_SOUTH_Y - half], [LARGE_WEST_X - half, LARGE_SOUTH_Y - half],
        [LARGE_WEST_X - half, IN_Y_MAX], [SMALL_EAST_X + half, IN_Y_MAX],
        [SMALL_EAST_X + half, SMALL_SOUTH_Y - half], [IN_X_MIN, SMALL_SOUTH_Y - half],
    ]


def polygon_centroid(points):
    area = cx = cy = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
        cross = x0 * y1 - x1 * y0
        area += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    area /= 2
    return cx / (6 * area), cy / (6 * area)


def _yaw_towards(src, dst):
    return math.atan2(dst[1] - src[1], dst[0] - src[0])


def semantic_targets():
    """Rooms and objects with object_pose and a reachable navigation_goal.

    Object goals are placed APPROACH metres in front of the object's nearest
    collision face and turned to face it; room goals are open floor points.
    """
    small_c = ((SMALL_ROOM[0] + SMALL_ROOM[1]) / 2, (SMALL_ROOM[2] + SMALL_ROOM[3]) / 2)
    large_c = ((LARGE_ROOM[0] + LARGE_ROOM[1]) / 2, (LARGE_ROOM[2] + LARGE_ROOM[3]) / 2)
    living_poly = living_room_polygon()
    living_c = polygon_centroid(living_poly)

    box_goal = (BOX_XY[0], BOX_XY[1] - BOX_SIZE / 2 - APPROACH)            # south face
    bed_goal = (BED_XY[0] - BED_W / 2 - APPROACH, BED_XY[1] - 0.2)         # west side
    table_goal = (TABLE_XY[0], TABLE_XY[1] - TABLE_W / 2 - APPROACH)       # south side
    small_win = (X_MIN + WALL_T / 2, sum(SMALL_WINDOW_Y) / 2)
    living_win = (X_MAX - WALL_T / 2, sum(LIVING_WINDOW_Y) / 2)
    # Window goals stand inside, APPROACH + 0.15 m from the inner wall face.
    small_win_goal = (IN_X_MIN + APPROACH + 0.15, small_win[1])
    living_win_goal = (IN_X_MAX - APPROACH - 0.15, living_win[1])

    rooms = {
        'small_room': dict(
            name='Small Room', aliases=['작은 방', '작은방', 'small room'],
            object_pose=(small_c[0], small_c[1], 0.0, 0.0),
            navigation_goal=(-5.4, 6.6, math.pi / 2),
            region=_rect(*SMALL_ROOM)),
        'large_room': dict(
            name='Large Room', aliases=['큰 방', '큰방', 'large room'],
            object_pose=(large_c[0], large_c[1], 0.0, 0.0),
            navigation_goal=(3.8, 6.0, math.pi / 2),
            region=_rect(*LARGE_ROOM)),
        'living_room': dict(
            name='Living Room', aliases=['거실', '중앙 공간', 'living room'],
            object_pose=(living_c[0], living_c[1], 0.0, 0.0),
            navigation_goal=(0.0, -5.5, math.pi / 2),
            region=living_poly),
    }
    objects = {
        'small_room_box': dict(
            name='Box', room='small_room', aliases=['박스', '상자', 'box'],
            object_pose=(BOX_XY[0], BOX_XY[1], 0.0, 0.0),
            navigation_goal=(*box_goal, _yaw_towards(box_goal, BOX_XY))),
        'large_room_bed': dict(
            name='Bed', room='large_room', aliases=['침대', 'bed'],
            object_pose=(BED_XY[0], BED_XY[1], 0.0, 0.0),
            navigation_goal=(*bed_goal, 0.0)),        # face +x, square to the bed side
        'living_room_table': dict(
            name='Table', room='living_room', aliases=['테이블', '탁자', 'table'],
            object_pose=(TABLE_XY[0], TABLE_XY[1], 0.0, 0.0),
            navigation_goal=(*table_goal, _yaw_towards(table_goal, TABLE_XY))),
        'small_room_window': dict(
            name='Small Room Window', room='small_room',
            aliases=['작은 방 창문', '작은방 창문', 'small room window'],
            object_pose=(small_win[0], small_win[1], WINDOW_SILL + WINDOW_H / 2, math.pi / 2),
            navigation_goal=(*small_win_goal, math.pi)),         # face west
        'living_room_window': dict(
            name='Living Room Window', room='living_room',
            aliases=['거실 창문', '거실쪽 창문', 'living room window'],
            object_pose=(living_win[0], living_win[1], WINDOW_SILL + WINDOW_H / 2, math.pi / 2),
            navigation_goal=(*living_win_goal, 0.0)),            # face east
    }
    return rooms, objects


def doors():
    return {
        'small_room_door': dict(rooms=['small_room', 'living_room'], width=DOOR_W,
                                center=(sum(SMALL_DOOR_X) / 2, SMALL_SOUTH_Y),
                                span={'axis': 'x', 'min': SMALL_DOOR_X[0], 'max': SMALL_DOOR_X[1]}),
        'large_room_door': dict(rooms=['large_room', 'living_room'], width=DOOR_W,
                                center=(sum(LARGE_DOOR_X) / 2, LARGE_SOUTH_Y),
                                span={'axis': 'x', 'min': LARGE_DOOR_X[0], 'max': LARGE_DOOR_X[1]}),
    }


# Where a robot can be spawned later (open floor, away from every goal).
ROBOT_SPAWN = (-3.0, -5.5, 0.0)

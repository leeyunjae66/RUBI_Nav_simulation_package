#!/usr/bin/env python3
"""Validate semantic_map.yaml against the Gazebo world it describes.

The checks read the generated .world file itself (not world_layout.py), so they
catch any disagreement between the SDF and the YAML.

    validate_semantic_map.py                       # installed / source files
    validate_semantic_map.py --world W --yaml Y -v
    validate_semantic_map.py --gazebo              # also compare with a running gzserver

Exit code 0 only when every executed check passes.
"""

import argparse
from collections import deque
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

REQUIRED_TARGETS = {
    'rooms': ('small_room', 'large_room', 'living_room'),
    'objects': ('small_room_box', 'large_room_bed', 'living_room_table',
                'small_room_window', 'living_room_window'),
}
POSE_TOL = 1e-3
FACING_TOL_DEG = 30.0
EDGE_SAMPLE = 0.1
EDGE_OFFSET = 0.05
OUTSIDE_MARGIN = 1.0


# ------------------------------------------------------------------ inputs

class _UniqueKeyLoader(yaml.SafeLoader):
    """yaml.safe_load silently keeps the last duplicate key; reject them instead."""


def _construct_mapping(loader, node, deep=False):
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in seen:
            raise yaml.constructor.ConstructorError(
                None, None, f'duplicate key {key!r}', key_node.start_mark)
        seen.add(key)
    return yaml.SafeLoader.construct_mapping(loader, node, deep)


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
                                 _construct_mapping)


def load_yaml(path):
    with open(path, encoding='utf-8') as stream:
        return yaml.load(stream, Loader=_UniqueKeyLoader)


def _pose(element):
    text = element.findtext('pose') if element is not None else None
    values = [float(v) for v in text.split()] if text else [0.0] * 6
    if len(values) != 6:
        raise ValueError(f'bad pose {text!r}')
    return values


def _compose(parent, child):
    """2D compose (x, y, z, yaw); roll/pitch must be zero for this world."""
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, pyaw + cyaw)


class Obstacle:
    """Oriented box footprint in the world frame."""

    def __init__(self, model, name, x, y, yaw, hx, hy, z0, z1):
        self.model, self.name = model, name
        self.x, self.y, self.yaw, self.hx, self.hy = x, y, yaw, hx, hy
        self.z0, self.z1 = z0, z1

    def distance(self, px, py):
        """Distance from (px, py) to the footprint; 0 inside."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx = c * (px - self.x) + s * (py - self.y)
        ly = -s * (px - self.x) + c * (py - self.y)
        return math.hypot(max(abs(lx) - self.hx, 0.0), max(abs(ly) - self.hy, 0.0))

    def contains(self, px, py):
        return self.distance(px, py) == 0.0


def parse_world(path):
    """-> (model poses {name: (x, y, z, yaw)}, [Obstacle], [problems])."""
    root = ET.parse(path).getroot()
    world = root.find('world')
    models, obstacles, problems = {}, [], []
    names = []
    for model in world.findall('model'):
        name = model.get('name')
        names.append(name)
        mp = _pose(model)
        if abs(mp[3]) > 1e-9 or abs(mp[4]) > 1e-9:
            problems.append(f'model {name}: roll/pitch not supported by the 2D checks')
        model_pose = (mp[0], mp[1], mp[2], mp[5])
        models[name] = model_pose
        for link in model.findall('link'):
            lp = _pose(link)
            link_pose = _compose(model_pose, (lp[0], lp[1], lp[2], lp[5]))
            for collision in link.findall('collision'):
                cp = _pose(collision)
                x, y, z, yaw = _compose(link_pose, (cp[0], cp[1], cp[2], cp[5]))
                geometry = collision.find('geometry')
                box, cyl = geometry.find('box'), geometry.find('cylinder')
                if box is not None:
                    sx, sy, sz = (float(v) for v in box.findtext('size').split())
                    obstacles.append(Obstacle(name, collision.get('name'), x, y, yaw,
                                              sx / 2, sy / 2, z - sz / 2, z + sz / 2))
                elif cyl is not None:      # conservative: bounding square
                    r, h = float(cyl.findtext('radius')), float(cyl.findtext('length'))
                    obstacles.append(Obstacle(name, collision.get('name'), x, y, 0.0, r, r,
                                              z - h / 2, z + h / 2))
                elif geometry.find('plane') is None:
                    problems.append(f'{name}/{collision.get("name")}: unsupported geometry')
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        problems.append(f'duplicate model names in world: {", ".join(duplicates)}')
    return models, obstacles, problems


# ---------------------------------------------------------------- geometry

def point_in_polygon(px, py, polygon):
    inside = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y0 > py) != (y1 > py):
            if px < (x1 - x0) * (py - y0) / (y1 - y0) + x0:
                inside = not inside
    return inside


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) \
        and math.isfinite(value)


class Report:
    def __init__(self):
        self.lines = []
        self.ok = True

    def check(self, condition, message):
        self.lines.append(('ok' if condition else 'FAIL', message))
        self.ok &= bool(condition)
        return condition


# ------------------------------------------------------------------ checks

def validate(world_path, yaml_path, gazebo=False, verbose=False):
    data = load_yaml(yaml_path)
    models, obstacles, world_problems = parse_world(world_path)
    nav = data['navigation_config']
    required = float(nav['robot_radius']) + float(nav['min_goal_clearance'])
    z_max = float(nav.get('obstacle_max_height', 1.0))
    blocking = [o for o in obstacles if o.z0 < z_max]
    bounds = data['map']['bounds']

    targets = {}
    for section in ('rooms', 'objects'):
        for target_id, entry in (data.get(section) or {}).items():
            targets[target_id] = dict(entry, section=section)
    regions = {rid: data['rooms'][rid]['region'] for rid in data.get('rooms', {})}

    def room_of(px, py):
        hits = [rid for rid, poly in regions.items() if point_in_polygon(px, py, poly)]
        return hits[0] if len(hits) == 1 else None

    per_target = {}
    clearance_ok, pose_ok = True, True
    details = {}

    # target set
    set_report = Report()
    for section, ids in REQUIRED_TARGETS.items():
        present = set((data.get(section) or {}).keys())
        for target_id in ids:
            set_report.check(target_id in present, f'{section}.{target_id} present')
        set_report.check(not (present - set(ids)), f'no unexpected {section}: '
                         f'{sorted(present - set(ids)) or "none"}')
    for problem in world_problems:
        set_report.check(False, problem)
    pose_ok &= set_report.ok
    details['targets'] = set_report

    for target_id, entry in targets.items():
        rep = Report()
        goal, obj = entry.get('navigation_goal') or {}, entry.get('object_pose') or {}
        numeric = (all(_finite_number(goal.get(k)) for k in ('x', 'y', 'yaw'))
                   and all(_finite_number(obj.get(k)) for k in ('x', 'y', 'z', 'yaw')))
        if not rep.check(numeric, 'all coordinates and yaw are finite numbers'):
            per_target[target_id] = rep
            clearance_ok = False
            continue
        gx, gy, gyaw = goal['x'], goal['y'], goal['yaw']
        rep.check(bounds['x_min'] < gx < bounds['x_max'] and bounds['y_min'] < gy < bounds['y_max'],
                  f'goal ({gx}, {gy}) inside the building bounds')
        inside = [o for o in obstacles if o.contains(gx, gy)]
        rep.check(not inside, 'goal not inside any collision geometry'
                  + (f' (inside {inside[0].model}/{inside[0].name})' if inside else ''))
        nearest = min(blocking, key=lambda o: o.distance(gx, gy))
        distance = nearest.distance(gx, gy)
        ok = rep.check(distance >= required - 1e-9,
                       f'clearance {distance:.3f} m >= {required:.3f} m '
                       f'(nearest {nearest.model}/{nearest.name})')
        clearance_ok &= ok
        home = entry['room'] if entry['section'] == 'objects' else target_id
        located = room_of(gx, gy)
        rep.check(located == home, f'goal lies in {home} (found {located})')

        if entry['section'] == 'rooms':
            rep.check(room_of(obj['x'], obj['y']) == target_id,
                      'object_pose (reference point) lies inside the room')
        else:
            world_pose = models.get(target_id)
            if rep.check(world_pose is not None, f'world has a model named {target_id}'):
                wx, wy, wz, wyaw = world_pose
                match = (abs(wx - obj['x']) <= POSE_TOL and abs(wy - obj['y']) <= POSE_TOL
                         and abs(wz - obj['z']) <= POSE_TOL
                         and angle_diff(wyaw, obj['yaw']) <= POSE_TOL)
                pose_ok &= rep.check(match, f'object_pose matches world ({wx:.4f}, {wy:.4f}, '
                                            f'{wz:.4f}, yaw {wyaw:.4f})')
            else:
                pose_ok = False
            if 'window' in target_id:
                # One side of the window must be the declared room, the other outside.
                nx, ny = -math.sin(obj['yaw']), math.cos(obj['yaw'])
                sides = [(obj['x'] + k * 0.3 * nx, obj['y'] + k * 0.3 * ny) for k in (1, -1)]
                inside_room = [room_of(*p) == entry['room'] for p in sides]
                outside = [not (bounds['x_min'] < p[0] < bounds['x_max']
                                and bounds['y_min'] < p[1] < bounds['y_max']) for p in sides]
                pose_ok &= rep.check(any(i and o2 for i, o2 in zip(inside_room, outside[::-1])),
                                     f'window is on an exterior wall of {entry["room"]}')
            else:
                pose_ok &= rep.check(room_of(obj['x'], obj['y']) == entry['room'],
                                     f'object placed inside {entry["room"]}')
            want = math.atan2(obj['y'] - gy, obj['x'] - gx)
            if 'window' in target_id:
                # face straight at the wall: compare with the wall normal pointing outwards
                nx, ny = -math.sin(obj['yaw']), math.cos(obj['yaw'])
                outward = (nx, ny) if (obj['x'] - gx) * nx + (obj['y'] - gy) * ny > 0 \
                    else (-nx, -ny)
                want = math.atan2(outward[1], outward[0])
            rep.check(math.degrees(angle_diff(gyaw, want)) <= FACING_TOL_DEG,
                      f'goal yaw {gyaw:.3f} faces the object (want {want:.3f}, '
                      f'tol {FACING_TOL_DEG:.0f} deg)')
        per_target[target_id] = rep

    room_geometry = check_room_geometry(data, blocking)
    connectivity = check_connectivity(data, blocking, required)
    runtime = check_gazebo(data, models) if gazebo else ('NOT TESTED', [])

    def status(ok):
        return 'PASS' if ok else 'FAIL'

    print('Semantic Map Validation\n')
    print(f'world: {world_path}\nyaml:  {yaml_path}\n')
    for target_id in [t for ids in REQUIRED_TARGETS.values() for t in ids]:
        rep = per_target.get(target_id)
        print(f'[{status(rep.ok) if rep else "FAIL"}] {target_id}')
        if rep and (verbose or not rep.ok):
            for mark, message in rep.lines:
                print(f'       {mark:<4} {message}')
    if not set_report.ok or verbose:
        for mark, message in set_report.lines:
            print(f'       {mark:<4} {message}')
    print()
    results = {
        'Collision Clearance': status(clearance_ok),
        'World Pose Matching': status(pose_ok),
        'Room Geometry': room_geometry[0],
        'Path Connectivity': connectivity[0],
        'Gazebo Runtime Poses': runtime[0],
    }
    for name, value in results.items():
        print(f'{name}: {value}')
        if verbose or value == 'FAIL':
            for line in (room_geometry if name == 'Room Geometry' else
                         connectivity if name == 'Path Connectivity' else
                         runtime if name == 'Gazebo Runtime Poses' else ('', []))[1]:
                print(f'       {line}')
    all_targets_ok = all(rep.ok for rep in per_target.values()) and len(per_target) == 8
    executed = [v for v in results.values() if v != 'NOT TESTED']
    success = all_targets_ok and set_report.ok and all(v == 'PASS' for v in executed)
    print()
    if success:
        print('All semantic targets validated.')
        not_tested = [k for k, v in results.items() if v == 'NOT TESTED']
        if not_tested:
            print(f'(not tested: {", ".join(not_tested)})')
    else:
        print('Validation FAILED.')
    return success


def check_room_geometry(data, blocking):
    """Each room rectangle must be closed by walls except at its declared doors."""
    lines, ok = [], True
    doors = data.get('doors') or {}
    for room_id in ('small_room', 'large_room'):
        polygon = data['rooms'][room_id]['region']
        room_doors = [d for d in doors.values() if room_id in d['rooms']]
        gaps = 0
        for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
            length = math.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(length / EDGE_SAMPLE))
            # outward normal of a counter-clockwise polygon edge
            nx, ny = (y1 - y0) / length, -(x1 - x0) / length
            for i in range(1, steps):
                t = i / steps
                px, py = x0 + t * (x1 - x0) + EDGE_OFFSET * nx, y0 + t * (y1 - y0) + EDGE_OFFSET * ny
                if any(o.contains(px, py) for o in blocking):
                    continue
                in_door = any(
                    (d['span']['axis'] == 'x' and d['span']['min'] <= px <= d['span']['max']
                     and abs(py - d['center'][1]) < 0.2)
                    or (d['span']['axis'] == 'y' and d['span']['min'] <= py <= d['span']['max']
                        and abs(px - d['center'][0]) < 0.2)
                    for d in room_doors)
                if not in_door:
                    gaps += 1
        state = 'closed by walls except its doors' if not gaps else f'{gaps} open boundary samples'
        lines.append(f'{room_id}: {state}')
        ok &= gaps == 0
    return ('PASS' if ok else 'FAIL'), lines


def check_connectivity(data, blocking, inflation):
    """BFS on an inflated 2D grid from the living room goal to every goal."""
    try:
        import numpy as np
    except ImportError:
        return 'NOT TESTED', ['numpy is not installed']
    res = float(data['navigation_config'].get('grid_resolution', 0.05))
    b = data['map']['bounds']
    x0, y0 = b['x_min'] - OUTSIDE_MARGIN, b['y_min'] - OUTSIDE_MARGIN
    nx = int(round((b['x_max'] - b['x_min'] + 2 * OUTSIDE_MARGIN) / res)) + 1
    ny = int(round((b['y_max'] - b['y_min'] + 2 * OUTSIDE_MARGIN) / res)) + 1
    xs = x0 + np.arange(nx) * res
    ys = y0 + np.arange(ny) * res
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    blocked = np.zeros((nx, ny), dtype=bool)
    for o in blocking:
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        lx = c * (gx - o.x) + s * (gy - o.y)
        ly = -s * (gx - o.x) + c * (gy - o.y)
        dist = np.hypot(np.maximum(np.abs(lx) - o.hx, 0.0), np.maximum(np.abs(ly) - o.hy, 0.0))
        blocked |= dist < inflation - 1e-9
    outside = ~((gx > b['x_min']) & (gx < b['x_max']) & (gy > b['y_min']) & (gy < b['y_max']))

    def cell(x, y):
        return int(round((x - x0) / res)), int(round((y - y0) / res))

    goals = {}
    for section in ('rooms', 'objects'):
        for target_id, entry in data[section].items():
            goals[target_id] = cell(entry['navigation_goal']['x'], entry['navigation_goal']['y'])
    start_id = 'living_room'
    start = goals[start_id]
    lines = [f'grid {nx}x{ny} @ {res} m, obstacles inflated by {inflation:.2f} m, '
             f'start = {start_id} goal']
    if blocked[start]:
        return 'FAIL', lines + ['start cell is blocked']
    seen = np.full((nx, ny), -1, dtype=np.int32)
    seen[start] = 0
    queue = deque([start])
    steps = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    while queue:
        i, j = queue.popleft()
        for di, dj in steps:
            a, c2 = i + di, j + dj
            if 0 <= a < nx and 0 <= c2 < ny and seen[a, c2] < 0 and not blocked[a, c2]:
                seen[a, c2] = seen[i, j] + 1
                queue.append((a, c2))
    ok = True
    for target_id, g in goals.items():
        reached = seen[g] >= 0
        ok &= reached
        lines.append(f'{target_id}: {"reachable" if reached else "UNREACHABLE"}'
                     + (f' ({seen[g]} cells)' if reached else ''))
    leaked = bool((seen >= 0)[outside].any())
    ok &= not leaked
    lines.append('no free path leaves the building (walls and windows are closed)'
                 if not leaked else 'a path reaches OUTSIDE the building')
    return ('PASS' if ok else 'FAIL'), lines


def check_gazebo(data, models):
    """Compare YAML object poses with a running gzserver (`gz model -p`)."""
    lines, ok = [], True
    for target_id, entry in data['objects'].items():
        try:
            out = subprocess.run(['gz', 'model', '-m', target_id, '-p'], capture_output=True,
                                 text=True, timeout=10).stdout.strip()
        except (OSError, subprocess.TimeoutExpired) as error:
            return 'NOT TESTED', [f'gz model failed: {error}']
        values = [float(v) for v in re.findall(r'-?\d+(?:\.\d+)?(?:e-?\d+)?', out)]
        if len(values) != 6:
            return 'NOT TESTED', [f'no running gzserver with {target_id} ({out[:60]!r})']
        x, y, z, _, _, yaw = values
        obj = entry['object_pose']
        match = (abs(x - obj['x']) <= 1e-3 and abs(y - obj['y']) <= 1e-3
                 and abs(z - obj['z']) <= 1e-3 and angle_diff(yaw, obj['yaw']) <= 1e-3)
        ok &= match
        lines.append(f'{target_id}: gazebo ({x:.4f}, {y:.4f}, {z:.4f}, {yaw:.4f}) '
                     f'{"matches" if match else "DIFFERS"}')
    return ('PASS' if ok else 'FAIL'), lines


def plot_floor_plan(world_path, yaml_path, out_path):
    """Top view: collision footprints, inflated clearance, rooms and goals."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as Poly

    data = load_yaml(yaml_path)
    _, obstacles, _ = parse_world(world_path)
    nav = data['navigation_config']
    inflation = float(nav['robot_radius']) + float(nav['min_goal_clearance'])
    z_max = float(nav.get('obstacle_max_height', 1.0))
    fig, ax = plt.subplots(figsize=(8, 10))
    colors = {'small_room': '#cfe3ff', 'large_room': '#d8f5d0', 'living_room': '#fff1cc'}
    for room_id, entry in data['rooms'].items():
        ax.add_patch(Poly(entry['region'], closed=True, color=colors.get(room_id, '#eee'),
                          alpha=0.7, zorder=0))
    for o in obstacles:
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        corners = [(o.x + c * dx - s * dy, o.y + s * dx + c * dy)
                   for dx, dy in ((-o.hx, -o.hy), (o.hx, -o.hy), (o.hx, o.hy), (-o.hx, o.hy))]
        blocking = o.z0 < z_max
        ax.add_patch(Poly(corners, closed=True, zorder=2,
                          color='#444' if blocking else '#aaa', alpha=1.0 if blocking else 0.4))
    # inflated (unreachable) band, drawn from the same grid the BFS uses
    try:
        import numpy as np
        b = data['map']['bounds']
        xs = np.arange(b['x_min'], b['x_max'] + 1e-9, 0.05)
        ys = np.arange(b['y_min'], b['y_max'] + 1e-9, 0.05)
        gx, gy = np.meshgrid(xs, ys, indexing='ij')
        blocked = np.zeros(gx.shape, dtype=bool)
        for o in obstacles:
            if o.z0 >= z_max:
                continue
            c, s = math.cos(o.yaw), math.sin(o.yaw)
            lx = c * (gx - o.x) + s * (gy - o.y)
            ly = -s * (gx - o.x) + c * (gy - o.y)
            blocked |= np.hypot(np.maximum(np.abs(lx) - o.hx, 0), np.maximum(np.abs(ly) - o.hy, 0)) \
                < inflation
        ax.contourf(gx, gy, blocked.astype(float), levels=[0.5, 1.5], colors=['#ff9a9a'],
                    alpha=0.35, zorder=1)
    except ImportError:
        pass
    for section in ('rooms', 'objects'):
        for target_id, entry in data[section].items():
            g = entry['navigation_goal']
            marker = 's' if section == 'rooms' else 'o'
            ax.plot(g['x'], g['y'], marker, color='#d62728', zorder=4)
            ax.arrow(g['x'], g['y'], 0.5 * math.cos(g['yaw']), 0.5 * math.sin(g['yaw']),
                     head_width=0.15, color='#d62728', zorder=4)
            ax.annotate(target_id, (g['x'], g['y']), textcoords='offset points', xytext=(5, -10),
                        fontsize=7, zorder=5)
    ax.set_xlim(data['map']['bounds']['x_min'] - 0.5, data['map']['bounds']['x_max'] + 0.5)
    ax.set_ylim(data['map']['bounds']['y_min'] - 0.5, data['map']['bounds']['y_max'] + 0.5)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'semantic_indoor_world: dark = collision below {z_max} m, '
                 f'red band = {inflation:.2f} m clearance')
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f'floor plan written to {out_path}')


def _default(kind):
    env = os.environ.get('SEMANTIC_WORLD' if kind == 'world' else 'SEMANTIC_MAP_YAML')
    if env:
        return env
    rel = Path('worlds/my_map.world') if kind == 'world' else Path('config/semantic_map.yaml')
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory('semantic_indoor_world')) / rel
        if candidate.is_file():
            return str(candidate)
    except Exception:  # not sourced / not built: fall back to the source tree
        pass
    return str(Path(__file__).resolve().parent.parent / rel)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', default=_default('world'))
    parser.add_argument('--yaml', default=_default('yaml'))
    parser.add_argument('--gazebo', action='store_true',
                        help='also compare object poses with a running gzserver')
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('--plot', default='', help='also write a top-view PNG to this path')
    args = parser.parse_args(argv)
    ok = validate(args.world, args.yaml, gazebo=args.gazebo, verbose=args.verbose)
    if args.plot:
        plot_floor_plan(args.world, args.yaml, args.plot)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

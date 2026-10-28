"""Checks for the generated world, the semantic map, the loader and the validator."""

import copy
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG / 'scripts'))

import generate_world  # noqa: E402
from semantic_map_loader import SemanticMap  # noqa: E402
from semantic_map_loader import SemanticMapError  # noqa: E402
from semantic_map_loader import UnknownTargetError  # noqa: E402
import validate_semantic_map as V  # noqa: E402
import world_layout as L  # noqa: E402

WORLD = PKG / 'worlds' / 'my_map.world'
YAML = PKG / 'config' / 'semantic_map.yaml'


# ------------------------------------------------------- generated files

def test_generated_files_are_up_to_date(tmp_path):
    """worlds/ and config/ must be exactly what world_layout.py produces."""
    w, y = tmp_path / 'w.world', tmp_path / 's.yaml'
    generate_world.main(['--world', str(w), '--yaml', str(y)])
    assert w.read_text() == WORLD.read_text(), 'regenerate: python3 scripts/generate_world.py'
    assert y.read_text() == YAML.read_text(), 'regenerate: python3 scripts/generate_world.py'


def test_yaml_has_eight_targets_and_no_nulls():
    data = yaml.safe_load(YAML.read_text())
    assert list(data['rooms']) == ['small_room', 'large_room', 'living_room']
    assert list(data['objects']) == ['small_room_box', 'large_room_bed', 'living_room_table',
                                     'small_room_window', 'living_room_window']
    assert 'null' not in YAML.read_text()
    assert data['navigation_config']['robot_radius'] == 0.25
    assert data['navigation_config']['min_goal_clearance'] == 0.5


def _models():
    models, obstacles, problems = V.parse_world(str(WORLD))
    assert problems == []
    return models, obstacles


def test_building_is_15_by_20_with_closed_outer_walls():
    _, obstacles = _models()
    walls = [o for o in obstacles if o.model == 'outer_walls']
    xs = [o.x + s * o.hx for o in walls for s in (-1, 1)]
    ys = [o.y + s * o.hy for o in walls for s in (-1, 1)]
    assert min(xs) == pytest.approx(-7.5) and max(xs) == pytest.approx(7.5)
    assert min(ys) == pytest.approx(-10.0) and max(ys) == pytest.approx(10.0)
    assert all(o.z1 == pytest.approx(L.WALL_H) for o in walls if 'sill' not in o.name)
    assert all(min(o.hx, o.hy) * 2 == pytest.approx(0.15) for o in walls)


def test_rooms_differ_in_size_and_doors_are_wide_enough():
    small = (L.SMALL_ROOM[1] - L.SMALL_ROOM[0]) * (L.SMALL_ROOM[3] - L.SMALL_ROOM[2])
    large = (L.LARGE_ROOM[1] - L.LARGE_ROOM[0]) * (L.LARGE_ROOM[3] - L.LARGE_ROOM[2])
    assert small < large
    for door in yaml.safe_load(YAML.read_text())['doors'].values():
        assert door['span']['max'] - door['span']['min'] >= 1.2


def test_windows_are_real_openings_with_glass():
    root = ET.parse(WORLD).getroot()
    for name in ('small_room_window', 'living_room_window'):
        model = root.find(f".//model[@name='{name}']")
        collisions = {c.get('name') for c in model.iter('collision')}
        assert 'glass_collision' in collisions and 'frame_top_collision' in collisions
        visual = model.find(".//visual[@name='glass_visual']")
        assert 0 < float(visual.findtext('transparency')) < 1
    # the outer wall itself has a gap at full height where the window is
    _, obstacles = _models()
    sill = [o for o in obstacles if o.model == 'outer_walls' and o.name.startswith('east_sill')]
    assert sill and sill[0].z1 == pytest.approx(L.WINDOW_SILL)
    width = 2 * max(sill[0].hx, sill[0].hy)
    assert width == pytest.approx(L.WINDOW_W)


def test_every_object_and_wall_has_collision():
    root = ET.parse(WORLD).getroot()
    for model in root.find('world').findall('model'):
        visuals = len(list(model.iter('visual')))
        collisions = len(list(model.iter('collision')))
        assert collisions >= 1 and collisions == visuals, model.get('name')


# ------------------------------------------------------------- validator

def test_shipped_files_validate(capsys):
    assert V.validate(str(WORLD), str(YAML))
    out = capsys.readouterr().out
    for target in ('small_room', 'large_room', 'living_room', 'small_room_box',
                   'large_room_bed', 'living_room_table', 'small_room_window',
                   'living_room_window'):
        assert f'[PASS] {target}' in out
    for line in ('Collision Clearance: PASS', 'World Pose Matching: PASS',
                 'Path Connectivity: PASS', 'Gazebo Runtime Poses: NOT TESTED'):
        assert line in out


def _write_yaml(tmp_path, mutate):
    data = yaml.safe_load(YAML.read_text())
    mutate(data)
    path = tmp_path / 'semantic_map.yaml'
    path.write_text(yaml.safe_dump(data, allow_unicode=True))
    return str(path)


@pytest.mark.parametrize('mutate, expect', [
    # goal right at the table: inside/too close to collision
    (lambda d: d['objects']['living_room_table']['navigation_goal'].update(x=0.0, y=-1.5),
     'Collision Clearance: FAIL'),
    # goal 0.6 m from the box: clear of it but closer than robot_radius + 0.5
    (lambda d: d['objects']['small_room_box']['navigation_goal'].update(y=8.05),
     'Collision Clearance: FAIL'),
    # object pose no longer matches the world
    (lambda d: d['objects']['small_room_box']['object_pose'].update(x=-4.0),
     'World Pose Matching: FAIL'),
    # bed goal moved into the living room
    (lambda d: d['objects']['large_room_bed']['navigation_goal'].update(x=3.3, y=2.0),
     '[FAIL] large_room_bed'),
    # missing target
    (lambda d: d['objects'].pop('living_room_window'), 'World Pose Matching: FAIL'),
    # non-numeric value
    (lambda d: d['rooms']['small_room']['navigation_goal'].update(yaw=None),
     '[FAIL] small_room'),
    # robot facing away from the table
    (lambda d: d['objects']['living_room_table']['navigation_goal'].update(yaw=-1.5708),
     '[FAIL] living_room_table'),
])
def test_validator_catches_bad_yaml(tmp_path, capsys, mutate, expect):
    assert not V.validate(str(WORLD), _write_yaml(tmp_path, mutate))
    assert expect in capsys.readouterr().out


def _write_world(tmp_path, edit):
    tree = ET.parse(WORLD)
    edit(tree.getroot().find('world'))
    path = tmp_path / 'w.world'
    tree.write(path)
    return str(path)


def test_validator_detects_unreachable_room(tmp_path, capsys):
    def block_door(world):
        model = ET.fromstring(
            '<model name="blocker"><static>true</static><pose>3.3 4.0 0.5 0 0 0</pose>'
            '<link name="link"><collision name="c"><geometry><box><size>2.2 0.15 1.0</size>'
            '</box></geometry></collision></link></model>')
        world.append(model)
    assert not V.validate(_write_world(tmp_path, block_door), str(YAML))
    out = capsys.readouterr().out
    assert 'Path Connectivity: FAIL' in out
    assert 'large_room: UNREACHABLE' in out


def test_validator_detects_path_leaving_the_building(tmp_path, capsys):
    def open_window(world):
        for model in list(world.findall('model')):
            if model.get('name') == 'living_room_window':
                world.remove(model)
        link = world.find("model[@name='outer_walls']/link")
        for collision in list(link.findall('collision')):
            if collision.get('name') == 'east_sill_collision':
                link.remove(collision)
    assert not V.validate(_write_world(tmp_path, open_window), str(YAML))
    out = capsys.readouterr().out
    assert 'a path reaches OUTSIDE the building' in out
    assert 'World Pose Matching: FAIL' in out


def test_validator_detects_open_room_boundary(tmp_path, capsys):
    def remove_wall(world):
        link = world.find("model[@name='small_room_walls']/link")
        for collision in list(link.findall('collision')):
            if collision.get('name') == 'east_collision':
                link.remove(collision)
    assert not V.validate(_write_world(tmp_path, remove_wall), str(YAML))
    assert 'Room Geometry: FAIL' in capsys.readouterr().out


def test_duplicate_yaml_keys_are_rejected(tmp_path):
    text = YAML.read_text().replace('objects:\n', 'objects:\n  small_room_box:\n    name: x\n', 1)
    path = tmp_path / 'dup.yaml'
    path.write_text(text)
    with pytest.raises(yaml.constructor.ConstructorError, match='duplicate key'):
        V.load_yaml(path)


# ---------------------------------------------------------------- loader

@pytest.fixture(scope='module')
def semantic_map():
    return SemanticMap(YAML)


def test_get_goal_returns_floats(semantic_map):
    goal = semantic_map.get_goal('large_room_bed')
    assert set(goal) == {'x', 'y', 'yaw'}
    assert all(isinstance(v, float) and math.isfinite(v) for v in goal.values())
    data = yaml.safe_load(YAML.read_text())
    assert goal == {k: float(v) for k, v in
                    data['objects']['large_room_bed']['navigation_goal'].items()}


@pytest.mark.parametrize('text, target', [
    ('작은 방', 'small_room'), ('작은방', 'small_room'), ('SMALL  ROOM', 'small_room'),
    ('큰방', 'large_room'), ('거실', 'living_room'), ('중앙 공간', 'living_room'),
    ('침대', 'large_room_bed'), ('상자', 'small_room_box'), ('탁자', 'living_room_table'),
    ('작은 방 창문', 'small_room_window'), ('작은방 창문', 'small_room_window'),
    ('거실쪽 창문', 'living_room_window'), ('living_room_table', 'living_room_table'),
])
def test_alias_lookup(semantic_map, text, target):
    assert semantic_map.find_target(text) == target


def test_unknown_target(semantic_map):
    with pytest.raises(UnknownTargetError, match='kitchen'):
        semantic_map.get_goal('kitchen')
    with pytest.raises(KeyError):            # UnknownTargetError is also a KeyError
        semantic_map.find_target('부엌')


def test_llm_command_resolution(semantic_map):
    result = semantic_map.resolve_llm_command('{"skill": "NAVIGATE", "target": "large_room_bed"}')
    assert result['target'] == 'large_room_bed' and result['frame_id'] == 'map'
    assert result['goal'] == semantic_map.get_goal('large_room_bed')
    # coordinates invented by the LLM are ignored
    hacked = semantic_map.resolve_llm_command(
        {'skill': 'navigate', 'target': '침대', 'x': 99, 'y': 99})
    assert hacked['goal'] == result['goal']
    for bad in ('not json', '[]', '{"skill": "PICK", "target": "bed"}', '{"skill": "NAVIGATE"}'):
        with pytest.raises(SemanticMapError):
            semantic_map.resolve_llm_command(bad)


def test_cli(capsys):
    from semantic_map_loader import main
    assert main(['--yaml', str(YAML), '--alias', '침대']) == 0
    assert json.loads(capsys.readouterr().out)['target'] == 'large_room_bed'
    assert main(['--yaml', str(YAML), 'nowhere']) == 1


def test_loader_does_not_mutate_yaml(semantic_map):
    before = copy.deepcopy(semantic_map.data)
    semantic_map.get_goal('small_room')
    semantic_map.find_target('박스')
    assert semantic_map.data == before

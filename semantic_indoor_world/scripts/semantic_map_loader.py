#!/usr/bin/env python3
"""Look up semantic navigation targets in semantic_map.yaml.

The LLM only chooses a target id (e.g. {"skill": "NAVIGATE", "target":
"large_room_bed"}); coordinates always come from the YAML through this module,
never from the model.

    from semantic_map_loader import SemanticMap
    semantic_map = SemanticMap()                     # installed / source YAML
    goal = semantic_map.get_goal('large_room_bed')   # {'x': ..., 'y': ..., 'yaw': ...}
    target_id = semantic_map.find_target('침대')     # alias -> 'large_room_bed'

CLI:
    semantic_map_loader.py --list
    semantic_map_loader.py large_room_bed
    semantic_map_loader.py --alias "작은 방 창문"
    semantic_map_loader.py --llm '{"skill": "NAVIGATE", "target": "living_room_table"}'
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys

import yaml

SECTIONS = ('rooms', 'objects')
SUPPORTED_SKILLS = ('NAVIGATE',)


class SemanticMapError(Exception):
    """Base class for lookup errors."""


class UnknownTargetError(SemanticMapError, KeyError):
    def __init__(self, target, known):
        self.target = target
        self.known = sorted(known)
        super().__init__(f'unknown semantic target {target!r}; known: {", ".join(self.known)}')

    def __str__(self):
        return self.args[0]


def default_yaml_path():
    """$SEMANTIC_MAP_YAML, else the installed share dir, else the source tree."""
    env = os.environ.get('SEMANTIC_MAP_YAML')
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('semantic_indoor_world'))
        candidate = installed / 'config' / 'semantic_map.yaml'
        if candidate.is_file():
            return candidate
    except Exception:  # ament not sourced / package not built: use the source tree
        pass
    return Path(__file__).resolve().parent.parent / 'config' / 'semantic_map.yaml'


def normalize(text):
    return ' '.join(str(text).strip().lower().split())


class SemanticMap:

    def __init__(self, path=None):
        self.path = Path(path) if path else default_yaml_path()
        with open(self.path, encoding='utf-8') as stream:
            self.data = yaml.safe_load(stream)
        self.targets = {}
        for section in SECTIONS:
            for target_id, entry in (self.data.get(section) or {}).items():
                if target_id in self.targets:
                    raise SemanticMapError(f'duplicate target id {target_id!r}')
                self.targets[target_id] = dict(entry, kind=section[:-1], id=target_id)
        self._aliases = {}
        for target_id, entry in self.targets.items():
            for key in [target_id, entry.get('name', '')] + list(entry.get('aliases', [])):
                for variant in (normalize(key), normalize(key).replace(' ', '')):
                    if not variant:
                        continue
                    owner = self._aliases.setdefault(variant, target_id)
                    if owner != target_id:
                        raise SemanticMapError(
                            f'alias {key!r} is used by both {owner!r} and {target_id!r}')

    @property
    def frame_id(self):
        return self.data.get('map', {}).get('frame_id', 'map')

    def target_ids(self):
        return list(self.targets)

    def get_target(self, target_id):
        try:
            return self.targets[target_id]
        except KeyError:
            raise UnknownTargetError(target_id, self.targets) from None

    def get_goal(self, target_id):
        goal = self.get_target(target_id)['navigation_goal']
        result = {key: float(goal[key]) for key in ('x', 'y', 'yaw')}
        if not all(math.isfinite(v) for v in result.values()):
            raise SemanticMapError(f'navigation_goal of {target_id!r} is not finite: {goal}')
        return result

    def get_object_pose(self, target_id):
        pose = self.get_target(target_id)['object_pose']
        return {key: float(pose[key]) for key in ('x', 'y', 'z', 'yaw')}

    def find_target(self, text):
        """Exact id / name / alias match (case and spacing insensitive)."""
        key = normalize(text)
        for variant in (key, key.replace(' ', '')):
            if variant in self._aliases:
                return self._aliases[variant]
        raise UnknownTargetError(text, self.targets)

    def resolve(self, target):
        """Accept a target id or an alias and return the target id."""
        if target in self.targets:
            return target
        return self.find_target(target)

    def resolve_llm_command(self, command):
        """{"skill": "NAVIGATE", "target": "<id or alias>"} -> goal for Nav2.

        Returns {"skill", "target", "frame_id", "goal": {"x", "y", "yaw"}}.
        Coordinates in the LLM output, if any, are ignored on purpose.
        """
        if isinstance(command, str):
            try:
                command = json.loads(command)
            except json.JSONDecodeError as error:
                raise SemanticMapError(f'LLM output is not JSON: {error}') from None
        if not isinstance(command, dict):
            raise SemanticMapError('LLM output must be a JSON object')
        skill = str(command.get('skill', '')).upper()
        if skill not in SUPPORTED_SKILLS:
            raise SemanticMapError(f'unsupported skill {command.get("skill")!r}; '
                                   f'supported: {", ".join(SUPPORTED_SKILLS)}')
        if not command.get('target'):
            raise SemanticMapError('LLM output has no "target"')
        target_id = self.resolve(command['target'])
        return {'skill': skill, 'target': target_id, 'frame_id': self.frame_id,
                'goal': self.get_goal(target_id)}


def main(argv=None):
    parser = argparse.ArgumentParser(description='Query semantic_map.yaml')
    parser.add_argument('target', nargs='?', help='target id')
    parser.add_argument('--alias', help='look a target up by alias, e.g. "침대"')
    parser.add_argument('--llm', help='LLM JSON, e.g. \'{"skill":"NAVIGATE","target":"..."}\'')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--yaml', default=None)
    args = parser.parse_args(argv)
    semantic_map = SemanticMap(args.yaml)
    try:
        if args.list or not (args.target or args.alias or args.llm):
            print(f'{semantic_map.path}')
            for target_id in semantic_map.target_ids():
                entry = semantic_map.get_target(target_id)
                goal = semantic_map.get_goal(target_id)
                print(f'  {target_id:<20} {entry["kind"]:<6} goal=({goal["x"]:.3f}, {goal["y"]:.3f}, '
                      f'yaw {goal["yaw"]:.3f})  aliases: {", ".join(entry.get("aliases", []))}')
        elif args.llm:
            print(json.dumps(semantic_map.resolve_llm_command(args.llm), ensure_ascii=False))
        else:
            target_id = semantic_map.resolve(args.alias or args.target)
            print(json.dumps({'target': target_id, 'goal': semantic_map.get_goal(target_id)},
                             ensure_ascii=False))
    except SemanticMapError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

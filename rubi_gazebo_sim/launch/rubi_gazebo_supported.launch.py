import glob
import os
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def _bool(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _cleanup_generated_world(context, generated_paths):
    del context
    for path in generated_paths:
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
    return []


def _launch_setup(context, generated_paths):
    sim_share = get_package_share_directory('rubi_gazebo_sim')
    core_share = get_package_share_directory('rubi_control_core')
    gazebo_share = get_package_share_directory('gazebo_ros')
    plugin_prefix = get_package_prefix('rubi_gazebo_plugins')
    source_world = os.path.join(sim_share, 'worlds', 'rubi_controller_supported.world')

    tree = ET.parse(source_world)
    world = tree.getroot().find('world')
    fixture = world.find("model[@name='rubi_support_fixture']")
    if not _bool(LaunchConfiguration('support_enabled').perform(context)):
        world.remove(fixture)
    else:
        pose = [LaunchConfiguration(name).perform(context) for name in
                ('support_x', 'support_y', 'support_z')]
        size = [LaunchConfiguration(name).perform(context) for name in
                ('support_size_x', 'support_size_y', 'support_size_z')]
        fixture.find('pose').text = ' '.join((*pose, '0', '0', '0'))
        fixture.find('link/collision/geometry/box/size').text = ' '.join(size)
        fixture.find('link/visual/geometry/box/size').text = ' '.join(size)

    controller_variant = LaunchConfiguration('controller_variant').perform(context)
    robot_include = next(
        include for include in world.findall('include')
        if include.findtext('uri') == 'model://RUBI')
    plugin = robot_include.find('plugin')
    if controller_variant == 'legacy_policy':
        plugin.set('name', 'legacy_policy_controller')
        plugin.set('filename', 'librubi_gazebo_legacy_policy_plugin.so')
    elif controller_variant != 'experimental_unified':
        raise RuntimeError(
            'controller_variant must be experimental_unified or legacy_policy')

    with tempfile.NamedTemporaryFile(
            mode='wb', prefix='rubi_controller_supported_', suffix='.world',
            delete=False) as generated:
        tree.write(generated, encoding='utf-8', xml_declaration=True)
        generated_paths.append(generated.name)

    system_model_paths = sorted(glob.glob('/usr/share/gazebo-*/models'))
    model_paths = [os.path.join(sim_share, 'models'), *system_model_paths]
    return [
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH', [os.pathsep.join(model_paths), os.pathsep,
                                  EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH', [os.path.join(plugin_prefix, 'lib'), os.pathsep,
                                   EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')]),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('RUBI_GAZEBO_ENCODER', LaunchConfiguration('encoder')),
        SetEnvironmentVariable('RUBI_GAZEBO_POLICY', LaunchConfiguration('policy')),
        SetEnvironmentVariable('RUBI_GAZEBO_NAMESPACE', LaunchConfiguration('namespace')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_COMMAND_TIMEOUT', LaunchConfiguration('command_timeout')),
        SetEnvironmentVariable('RUBI_GAZEBO_SELF_TEST', LaunchConfiguration('auto_demo')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_POLICY_TEST_TICKS', LaunchConfiguration('policy_test_ticks')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_ENABLED', LaunchConfiguration('diagnostics_enabled')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_CSV', LaunchConfiguration('diagnostics_csv_path')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_EVERY_INFERENCE', 'true'),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_LEGACY_POLICY', LaunchConfiguration('legacy_policy')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_LEGACY_NAMESPACE', LaunchConfiguration('namespace')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_LEGACY_SELF_TEST', LaunchConfiguration('auto_demo')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_LEGACY_POLICY_TEST_TICKS',
            LaunchConfiguration('policy_test_ticks')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': generated.name,
                'pause': LaunchConfiguration('paused'),
                'verbose': LaunchConfiguration('verbose'),
                'server_required': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ]


def generate_launch_description():
    sim_share = get_package_share_directory('rubi_gazebo_sim')
    core_share = get_package_share_directory('rubi_control_core')
    config_path = os.path.join(sim_share, 'config', 'support_fixture.yaml')
    with open(config_path, encoding='utf-8') as stream:
        defaults = yaml.safe_load(stream)['support_fixture']

    generated_paths = []
    actions = [
        DeclareLaunchArgument(
            'controller_variant', default_value='experimental_unified'),
        DeclareLaunchArgument('support_enabled', default_value=str(defaults['enabled']).lower()),
        DeclareLaunchArgument('support_x', default_value=str(defaults['pose']['x'])),
        DeclareLaunchArgument('support_y', default_value=str(defaults['pose']['y'])),
        DeclareLaunchArgument('support_z', default_value=str(defaults['pose']['z'])),
        DeclareLaunchArgument('support_size_x', default_value=str(defaults['size']['x'])),
        DeclareLaunchArgument('support_size_y', default_value=str(defaults['size']['y'])),
        DeclareLaunchArgument('support_size_z', default_value=str(defaults['size']['z'])),
        DeclareLaunchArgument(
            'encoder', default_value=os.path.join(core_share, 'models', 'rubi_plane_onnx', 'encoder.onnx')),
        DeclareLaunchArgument(
            'policy', default_value=os.path.join(core_share, 'models', 'rubi_plane_onnx', 'policy.onnx')),
        DeclareLaunchArgument(
            'legacy_policy', default_value=os.path.join(
                core_share, 'models', 'rubi_gazebo_legacy_policy.onnx')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='/rubi_gazebo_supported'),
        DeclareLaunchArgument('command_timeout', default_value='0.5'),
        DeclareLaunchArgument('auto_demo', default_value='false'),
        DeclareLaunchArgument('policy_test_ticks', default_value='1500'),
        DeclareLaunchArgument('diagnostics_enabled', default_value='false'),
        DeclareLaunchArgument('diagnostics_csv_path', default_value=''),
        RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(
                function=_cleanup_generated_world, args=[generated_paths])])),
        OpaqueFunction(function=_launch_setup, args=[generated_paths]),
    ]
    return LaunchDescription(actions)

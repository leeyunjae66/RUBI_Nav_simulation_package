import glob
import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('rubi_gazebo_sim')
    core_share = get_package_share_directory('rubi_control_core')
    gazebo_share = get_package_share_directory('gazebo_ros')
    plugin_prefix = get_package_prefix('rubi_gazebo_plugins')
    system_model_paths = sorted(glob.glob('/usr/share/gazebo-*/models'))
    model_paths = [os.path.join(sim_share, 'models'), *system_model_paths]
    world = os.path.join(sim_share, 'worlds', 'rubi_controller.world')

    actions = [
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('encoder', default_value=os.path.join(core_share, 'models', 'rubi_plane_onnx', 'encoder.onnx')),
        DeclareLaunchArgument('policy', default_value=os.path.join(core_share, 'models', 'rubi_plane_onnx', 'policy.onnx')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='/rubi_gazebo'),
        DeclareLaunchArgument('command_timeout', default_value='0.5'),
        DeclareLaunchArgument('policy_test_ticks', default_value='500'),
        DeclareLaunchArgument('auto_demo', default_value='false'),
        DeclareLaunchArgument('diagnostics_enabled', default_value='false'),
        DeclareLaunchArgument('diagnostics_csv_path', default_value=''),
        DeclareLaunchArgument('diagnostics_every_inference', default_value='true'),
        DeclareLaunchArgument('diagnostics_stop_on_threshold', default_value='false'),
        DeclareLaunchArgument('diagnostics_thresholds', default_value='10,100,1000,1e6,1e12,1e24'),
        DeclareLaunchArgument('diagnostics_post_window', default_value='5'),
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
        SetEnvironmentVariable('RUBI_GAZEBO_COMMAND_TIMEOUT', LaunchConfiguration('command_timeout')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_POLICY_TEST_TICKS', LaunchConfiguration('policy_test_ticks')),
        SetEnvironmentVariable('RUBI_GAZEBO_SELF_TEST', LaunchConfiguration('auto_demo')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_ENABLED', LaunchConfiguration('diagnostics_enabled')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_CSV', LaunchConfiguration('diagnostics_csv_path')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_EVERY_INFERENCE',
            LaunchConfiguration('diagnostics_every_inference')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_STOP_ON_THRESHOLD',
            LaunchConfiguration('diagnostics_stop_on_threshold')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_THRESHOLDS',
            LaunchConfiguration('diagnostics_thresholds')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_DIAGNOSTICS_POST_WINDOW',
            LaunchConfiguration('diagnostics_post_window')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'pause': LaunchConfiguration('paused'),
                'verbose': LaunchConfiguration('verbose'),
                'server_required': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ]
    return LaunchDescription(actions)

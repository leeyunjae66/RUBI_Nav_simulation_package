import glob
import os
import shutil
import tempfile

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('rubi_gazebo_sim')
    core_share = get_package_share_directory('rubi_control_core')
    gazebo_share = get_package_share_directory('gazebo_ros')
    plugin_prefix = get_package_prefix('rubi_gazebo_plugins')
    system_model_paths = sorted(glob.glob('/usr/share/gazebo-*/models'))
    model_paths = [os.path.join(sim_share, 'models'), *system_model_paths]
    world = os.path.join(sim_share, 'worlds', 'my_map.world')
    # MID-360 LiDAR (RUBI_mid360_imu): Livox plugin, its scan pattern and the
    # roll-180 frame adapter, as in rubi_mid360_imu_gazebo.launch.py.
    description_share = get_package_share_directory('rubi_description')
    livox_prefix = get_package_prefix('ros2_livox_simulation')
    livox_share = get_package_share_directory('ros2_livox_simulation')
    gazebo_plugins_prefix = get_package_prefix('gazebo_plugins')
    # GPU-rendered MID-360 (RUBI_mid360_imu_gpu); optional package
    try:
        gpu_livox_lib = os.path.join(get_package_prefix('gpu_livox_lidar'), 'lib')
    except Exception:
        gpu_livox_lib = ''
    shutil.copyfile(
        os.path.join(livox_share, 'scan_mode', 'mid360.csv'),
        os.path.join(tempfile.gettempdir(), 'rubi_mid360_imu_mid360.csv'))
    robot_description = ParameterValue(
        Command(['xacro', ' ', os.path.join(description_share, 'urdf',
                                            'RUBI_mid360_imu.urdf.xacro')]),
        value_type=str)

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
        DeclareLaunchArgument('lidar', default_value='true',
                              description='start the MID-360 frame adapter and '
                                          'robot_state_publisher (needs RUBI_mid360_imu)'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH', [os.pathsep.join(model_paths), os.pathsep,
                                  EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH', [os.path.join(plugin_prefix, 'lib'), os.pathsep,
                                   os.path.join(livox_prefix, 'lib'), os.pathsep,
                                   gpu_livox_lib, os.pathsep,
                                   os.path.join(gazebo_plugins_prefix, 'lib'), os.pathsep,
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
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace='/rubi_mid360',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('lidar')), output='screen'),
        Node(
            package='rubi_gazebo_sim', executable='mid360_frame_adapter_node',
            name='mid360_frame_adapter',
            parameters=[{
                'apply_roll_180': True,
                'input_lidar_topic': '/livox/lidar_raw',
                'output_lidar_topic': '/livox/lidar',
                'input_pointcloud2_topic': '/livox/lidar_raw_PointCloud2',
                'output_pointcloud2_topic': '/livox/lidar_PointCloud2',
                'input_imu_topic': '/livox/imu_raw',
                'output_imu_topic': '/livox/imu',
                'output_frame_id': 'livox_frame',
                'use_sim_time': True,
            }],
            condition=IfCondition(LaunchConfiguration('lidar')), output='screen'),
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hydrofoil_usv_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'hydrofoil.urdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    world_path = os.path.join(pkg_share, 'world', 'ocean.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from Gazebo as ROS time source')

    # The URDF's mesh uses a package:// URI, which sdformat rewrites to
    # model://hydrofoil_usv_description/meshes/ef12.obj. gz-sim only
    # resolves that against GZ_SIM_RESOURCE_PATH, which sourcing this
    # package's install/setup.bash does NOT populate on its own (unlike
    # AMENT_PREFIX_PATH) -- so prepend this package's share dir explicitly,
    # keeping whatever was already set.
    #
    # Also add asv_wave_sim's model/world assets so ocean.world's
    # `model://ocean_waves` include resolves. That repo carries a
    # COLCON_IGNORE (never colcon-built), so it's referenced straight out of
    # its cloned source tree -- these paths assume it was cloned into
    # ~/ros2_ws/src per the setup steps in bright-wiggling-ripple.md.
    asv_wave_sim_models = os.path.expanduser(
        '~/ros2_ws/src/asv_wave_sim/gz-waves-models')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.dirname(pkg_share),
            os.pathsep,
            os.path.join(asv_wave_sim_models, 'models'),
            os.pathsep,
            os.path.join(asv_wave_sim_models, 'world_models'),
            os.pathsep,
            os.path.join(asv_wave_sim_models, 'worlds'),
            os.pathsep,
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
        ])

    # The WavesModel/WavesVisual system plugins ocean.world's ocean_waves
    # model loads live in the gz-waves1 package built from asv_wave_sim
    # (colcon build --packages-select gz-waves1), not on the default
    # GZ_SIM_SYSTEM_PLUGIN_PATH -- so point gz-sim at its install lib dir.
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.expanduser('~/ros2_ws/install/gz-waves1/lib'),
            os.pathsep,
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
        ])

    # Plain URDF today, but routed through xacro so this scales cleanly if
    # the description ever grows xacro macros.
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }])

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hydrofoil',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'hydrofoil',
            # base_link's origin is centered on the whole mesh (hull through
            # mast-top). The mesh's lowest point (the foils, mesh-local
            # z=-0.05) sits at link-frame z=-3.425 after that centering
            # offset, so spawning at z=3.425 brings the foils to world z=0
            # (ground level) with the hull/mast riding above it.
            '-z', '3.425',
        ],
        parameters=[{'use_sim_time': use_sim_time}])

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': use_sim_time,
        }])

    return LaunchDescription([
        declare_use_sim_time,
        set_gz_resource_path,
        set_gz_plugin_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ])

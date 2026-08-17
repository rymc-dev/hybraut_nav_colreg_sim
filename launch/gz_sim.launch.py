import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, EnvironmentVariable,
                                   LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hybraut_nav_colreg_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'hydrofoil.urdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from Gazebo as ROS time source')

    # Filename only (not a path) -- resolved against this package's world/
    # dir below, so e.g. `world:=colav_scenario.world` switches to the
    # traffic-vessel scenario without needing a full path on the command
    # line. Everything else (ego spawn, bridge, resource paths) is the same
    # regardless of which world is picked.
    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world', default_value='ocean.world',
        description='World file (in world/) to load, e.g. colav_scenario.world')
    world_path = PathJoinSubstitution([pkg_share, 'world', world])

    # Off by default: these are the ROS-controllable counterparts of the
    # *static* traffic vessels world/colav_scenario.world already <include>s
    # as plain SDF models. Turning both on at once in the same world gives
    # you two copies of each vessel (a fixed SDF one from the world file, a
    # drivable URDF one spawned below) under different entity names, which
    # is rarely what you want -- so treat this as an alternative to
    # colav_scenario.world's static obstacles, for scenarios that need
    # traffic you can actually drive, not something layered on top of it.
    spawn_vessels = LaunchConfiguration('spawn_vessels')
    declare_spawn_vessels = DeclareLaunchArgument(
        'spawn_vessels', default_value='false',
        description="Also spawn every urdf/*_vessel.urdf as an independently "
                     "drivable robot (/<name>/cmd_vel, /<name>/odom) -- see "
                     "the comment above for why this isn't meant to be "
                     "combined with world:=colav_scenario.world")

    # Only meaningful alongside spawn_vessels:=true -- vessel_ais_bridge
    # subscribes to each vessel's /<name>/odom, which only exists once
    # they're actually spawned.
    publish_vessel_ais = LaunchConfiguration('publish_vessel_ais')
    declare_publish_vessel_ais = DeclareLaunchArgument(
        'publish_vessel_ais', default_value='false',
        description='Run vessel_ais_bridge, aggregating the spawned traffic '
                     'vessels (mock static particulars from '
                     'config/mock_ais_vessels.yaml + live odom) onto '
                     '/obstacles_state for hybraut_nav/risk_envelope_node')

    # The URDF's mesh uses a package:// URI, which sdformat rewrites to
    # model://hybraut_nav_colreg_sim/models/hydrofoil_vessel/meshes/ef12.obj
    # (package name + the rest of the path, unchanged). gz-sim only resolves
    # that against GZ_SIM_RESOURCE_PATH, which sourcing this package's
    # install/setup.bash does NOT populate on its own (unlike
    # AMENT_PREFIX_PATH) -- so prepend this package's share dir explicitly,
    # keeping whatever was already set. dirname(pkg_share) is what makes that
    # resolve: it puts pkg_share's *parent* on the path, so the URI's
    # "hybraut_nav_colreg_sim" segment lines up with the share dir itself
    # and the rest of the URI is found underneath it.
    #
    # Scenario-obstacle models (models/<vessel>/model.config, e.g.
    # `model://sail_boat_vessel`) are a separate case: a world's <include>
    # uses the bare model name with no package prefix, so it needs
    # pkg_share/models itself on the path -- dirname(pkg_share) alone won't
    # match those.
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
            os.path.join(pkg_share, 'models'),
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
        # world_path is a substitution (its value depends on the 'world'
        # launch arg), not a plain string, so it can't go through an
        # f-string -- concatenate it as a list of substitutions instead,
        # which launch joins at resolve time.
        launch_arguments={'gz_args': ['-r ', world_path]}.items())

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

    # Traffic-vessel fleet, gated behind spawn_vessels (see its
    # DeclareLaunchArgument above). Each entry's urdf name column matches
    # both its urdf/<name>.urdf filename and its <robot name="..."> --
    # spawn_entity's -name has to agree with that for the resulting model's
    # default gz topics (/model/<name>/cmd_vel etc.) to be what
    # config/ros_gz_bridge.yaml's per-vessel entries expect. x/y/yaw here
    # are just a reasonable non-overlapping spread (matching the layout
    # world/colav_scenario.world uses for its static copies of the same
    # vessels, for whatever that consistency is worth); z is chosen per
    # vessel so its hull's lowest point sits at world z=0, the same
    # reasoning as spawn_entity's -z above for the ego.
    vessels = [
        # name,                     x,     y,   z,      yaw
        ('hydrofoil_vessel',       -60,   -30,  3.425,  0.7854),
        ('sail_boat_vessel',        40,    20,  5.136,  3.14159),
        ('small_motor_boat_vessel', -30,   25,  1.1,   -1.5708),
        ('trauler_vessel',          50,   -40,  7.357,  1.5708),
        ('super_tanker_vessel',    150,     0,  14.0,   3.14159),
    ]

    vessel_nodes = []
    for name, x, y, z, yaw in vessels:
        vessel_urdf_path = os.path.join(pkg_share, 'urdf', f'{name}.urdf')
        vessel_robot_description = ParameterValue(
            Command(['xacro ', vessel_urdf_path]), value_type=str)

        vessel_nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=name,
            output='screen',
            condition=IfCondition(spawn_vessels),
            parameters=[{
                'robot_description': vessel_robot_description,
                'use_sim_time': use_sim_time,
            }]))

        vessel_nodes.append(Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            namespace=name,
            output='screen',
            condition=IfCondition(spawn_vessels),
            arguments=[
                # Relative -- resolves to /<namespace>/robot_description,
                # matching this node's own namespace= above, since -topic's
                # value goes through the same relative-name resolution as
                # any other topic a namespaced node subscribes to.
                '-topic', 'robot_description',
                '-name', name,
                '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
            ],
            parameters=[{'use_sim_time': use_sim_time}]))

    vessel_ais_bridge = Node(
        package='hybraut_nav_colreg_sim',
        executable='vessel_ais_bridge',
        name='vessel_ais_bridge',
        output='screen',
        condition=IfCondition(publish_vessel_ais),
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_spawn_vessels,
        declare_publish_vessel_ais,
        set_gz_resource_path,
        set_gz_plugin_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        vessel_ais_bridge,
    ] + vessel_nodes)

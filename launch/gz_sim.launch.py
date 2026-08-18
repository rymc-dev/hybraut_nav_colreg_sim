import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, EnvironmentVariable,
                                   LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from hybraut_nav_colreg_sim import scenario_loader, vessel_catalog


# Default hardcoded traffic-vessel fleet, used whenever scenario_file is
# unset (the default) -- see _build_vessel_fleet below. x/y/yaw here are
# just a reasonable non-overlapping spread (matching the layout
# world/colav_scenario.world uses for its static copies of the same
# vessels, for whatever that consistency is worth); z is chosen per vessel
# so its hull's lowest point sits at world z=0, the same reasoning as
# spawn_entity's -z below for the ego.
DEFAULT_VESSELS = [
    # name,                     x,     y,   z,      yaw
    ('hydrofoil_vessel',       -60,   -30,  3.425,  0.7854),
    ('sail_boat_vessel',        40,    20,  5.136,  3.14159),
    ('small_motor_boat_vessel', -30,   25,  1.1,   -1.5708),
    ('trauler_vessel',          50,   -40,  7.357,  1.5708),
    ('super_tanker_vessel',    150,     0,  14.0,   3.14159),
]


def _bridge_entries(name):
    """Same 3-entry cmd_vel/odom/tf shape as config/ros_gz_bridge.yaml's own
    per-vessel blocks, for a scenario-spawned vessel whose dynamic tag isn't
    in that static file."""
    return [
        {'ros_topic_name': f'/{name}/cmd_vel', 'gz_topic_name': f'/model/{name}/cmd_vel',
         'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'gz.msgs.Twist',
         'direction': 'ROS_TO_GZ'},
        {'ros_topic_name': f'/{name}/odom', 'gz_topic_name': f'/model/{name}/odometry',
         'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': '/tf', 'gz_topic_name': f'/model/{name}/pose',
         'ros_type_name': 'tf2_msgs/msg/TFMessage', 'gz_type_name': 'gz.msgs.Pose_V',
         'direction': 'GZ_TO_ROS'},
    ]


def _write_yaml_tempfile(data, prefix):
    fd, path = tempfile.mkstemp(prefix=f'hybraut_nav_colreg_sim_{prefix}_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(data, f)
    return path


def generate_launch_description():
    pkg_share = get_package_share_directory('hybraut_nav_colreg_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'hydrofoil.urdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    mock_ais_path = os.path.join(pkg_share, 'config', 'mock_ais_vessels.yaml')

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

    # Off by default: drive the traffic-vessel fleet's initial spawn poses
    # and control states from a CommonOcean scenario XML (see scenarios/
    # and hybraut_nav_colreg_sim/scenario_loader.py) instead of the
    # DEFAULT_VESSELS table above. Each ship-like obstacle in the file gets
    # a randomly-assigned vessel model (vessel_catalog.VESSEL_TYPES) spawned
    # at its (re-anchored, see scenario_loader.reanchor) recorded position,
    # and obstacle_vessel_controler replays its full recorded trajectory as
    # cmd_vel commands. Only meaningful alongside spawn_vessels:=true, same
    # as DEFAULT_VESSELS' own fleet.
    scenario_file = LaunchConfiguration('scenario_file')
    declare_scenario_file = DeclareLaunchArgument(
        'scenario_file', default_value='',
        description='Path to a CommonOcean scenario XML (see scenarios/) to '
                     'drive the traffic-vessel fleet from, instead of the '
                     'hardcoded DEFAULT_VESSELS layout. Empty (default) '
                     'keeps today\'s 5-vessel fleet unchanged.')

    scenario_seed = LaunchConfiguration('scenario_seed')
    declare_scenario_seed = DeclareLaunchArgument(
        'scenario_seed', default_value='',
        description="Seed for scenario_file's random vessel-type "
                     'assignment, for reproducible runs. Empty (default) '
                     'assigns randomly each launch.')

    # The URDF's mesh uses a package:// URI, which sdformat rewrites to
    # model://hybraut_nav_colreg_sim/models/hydrofoil_vessel/meshes/hydrofoil.obj
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

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': use_sim_time,
        }])

    def _build_vessel_fleet(context, *args, **kwargs):
        """Everything downstream of "which vessels, where, doing what" has
        to branch on scenario_file's *resolved* runtime value, not just the
        substitution object -- so this whole block runs inside an
        OpaqueFunction rather than at plain launch-description-generation
        time. See scenario_loader.py's module docstring for how this and
        obstacle_vessel_controler.py agree on the same fleet."""
        scenario_path = scenario_file.perform(context)

        extra_actions = []
        ais_config_path = mock_ais_path

        # ego_xy is deliberately ALWAYS vessel_catalog.EGO_SPAWN_XY, never a
        # scenario's own recorded planningProblem position -- CommonOcean/
        # Marine Cadastre scenario files aren't guaranteed to use a small
        # local frame (e.g. scenarios/USA_NYM-1_20190613_T-1.xml's ego sits
        # at real UTM-projected coordinates in the hundreds-of-thousands of
        # metres), and spawning there would put the ego millions of metres
        # from Gazebo's own origin. reanchor() below is what actually uses
        # this: it rigidly shifts every obstacle by
        # (ego_xy - scenario_ego_xy), so each keeps its real recorded
        # distance/bearing from the ego but re-centered near this small
        # anchor instead of out at the scenario's own coordinates -- see
        # that function's docstring. ego_yaw, unlike position, is safe to
        # take from the scenario as-is (angles don't blow up).
        if scenario_path:
            _scenario_for_ego = scenario_loader.parse_scenario(scenario_path)
        else:
            _scenario_for_ego = None
        ego_xy = vessel_catalog.EGO_SPAWN_XY
        if _scenario_for_ego is not None and _scenario_for_ego.ego_initial_xy is not None:
            ego_yaw = _scenario_for_ego.ego_initial_yaw
        else:
            ego_yaw = 0.0

        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_hydrofoil',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'hydrofoil',
                # base_link's origin is centered on the whole mesh (hull
                # through mast-top). The mesh's lowest point (the foils,
                # mesh-local z=-0.05) sits at link-frame z=-3.425 after that
                # centering offset, so spawning at z=3.425 brings the foils
                # to world z=0 (ground level) with the hull/mast riding
                # above it.
                '-x', str(ego_xy[0]), '-y', str(ego_xy[1]), '-z', '3.425',
                '-Y', str(ego_yaw),
            ],
            parameters=[{'use_sim_time': use_sim_time}])

        if not scenario_path:
            # Default path: today's hardcoded 5-vessel fleet, completely
            # unchanged -- no scenario parsing, no generated config, no
            # extra bridge/controller nodes. name == urdf stem here (no
            # scenario-id suffix), unlike the scenario branch below.
            vessel_table = [(name, name, x, y, z, yaw) for name, x, y, z, yaw in DEFAULT_VESSELS]
        else:
            scenario = _scenario_for_ego
            ships, skipped = scenario_loader.ship_obstacles(scenario)
            if skipped:
                print(
                    f'gz_sim.launch.py: not spawning {len(skipped)} non-ship '
                    f'scenario obstacle(s) with no vessel model: '
                    f'{[(o.id, o.raw_type) for o in skipped]}')
            # reanchor()'s actual job (see ego_xy's own comment above):
            # rigidly shift every obstacle from the scenario's own recorded
            # frame onto this small ego_xy anchor, preserving real
            # distances/bearings between the ego and each obstacle.
            ships = scenario_loader.reanchor(
                ships, ego_xy, scenario.ego_initial_xy)

            # Resolve scenario_seed to a concrete value now (even if the
            # user left it empty) and pass that concrete value on to
            # obstacle_vessel_controler below, rather than letting each side
            # draw its own random seed -- otherwise they'd assign different
            # vessel types to the same obstacle ids.
            seed = scenario_loader.resolve_seed(scenario_seed.perform(context))
            assignment = scenario_loader.assign_vessel_types(
                ships, vessel_catalog.VESSEL_TYPES, seed=seed)

            # (tag, urdf_stem, x, y, z, yaw) -- tag carries the scenario id
            # suffix (vessel_tag), urdf_stem doesn't (it's a VESSEL_TYPES
            # entry, used to find urdf/<urdf_stem>.urdf below).
            vessel_table = [
                (scenario_loader.vessel_tag(assignment[o.id], o.id), assignment[o.id],
                 o.initial.x, o.initial.y,
                 vessel_catalog.SPAWN_Z[assignment[o.id]], o.initial.yaw)
                for o in ships
            ]

            bridge_entries = []
            particulars = vessel_catalog.load_particulars(mock_ais_path)
            agent_entry = vessel_catalog.load_agent_entry(mock_ais_path)
            ais_vessels = [agent_entry] if agent_entry else []
            for tag, urdf_stem, _, _, _, _ in vessel_table:
                bridge_entries += _bridge_entries(tag)
                ais_vessels.append({**particulars[urdf_stem], 'tag': tag})

            bridge_extra_path = _write_yaml_tempfile(bridge_entries, 'bridge')
            ais_config_path = _write_yaml_tempfile({'vessels': ais_vessels}, 'ais')

            extra_actions.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge_scenario',
                output='screen',
                condition=IfCondition(spawn_vessels),
                parameters=[{
                    'config_file': bridge_extra_path,
                    'use_sim_time': use_sim_time,
                }]))

            extra_actions.append(Node(
                package='hybraut_nav_colreg_sim',
                executable='obstacle_vessel_controler',
                name='obstacle_vessel_controler',
                output='screen',
                condition=IfCondition(spawn_vessels),
                parameters=[{
                    'scenario_file': scenario_path,
                    'scenario_seed': str(seed),
                    'use_sim_time': use_sim_time,
                }]))

        vessel_nodes = []
        for name, urdf_stem, x, y, z, yaw in vessel_table:
            vessel_urdf_path = os.path.join(pkg_share, 'urdf', f'{urdf_stem}.urdf')
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
                    # matching this node's own namespace= above, since
                    # -topic's value goes through the same relative-name
                    # resolution as any other topic a namespaced node
                    # subscribes to.
                    '-topic', 'robot_description',
                    '-name', name,
                    '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
                ],
                parameters=[{'use_sim_time': use_sim_time}]))

            # <name>/odom (see this vessel's urdf's OdometryPublisher block)
            # is its own separate TF root -- gz-sim-odometry-publisher-system
            # doesn't parent it to anything, and its published poses are
            # already in world coordinates (same convention as the ego's own
            # odom_frame). Without this, <name>/odom and the ego's odom are
            # two disconnected trees, so nothing that depends on TF
            # (RobotModel, Odometry displays, ...) can be placed relative to
            # the other -- rviz reports "Tf has two or more unconnected
            # trees." An identity transform is correct here, not an offset,
            # precisely because both roots already coincide with world
            # origin.
            vessel_nodes.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{name}_odom_to_world',
                output='screen',
                condition=IfCondition(spawn_vessels),
                arguments=['--frame-id', 'odom', '--child-frame-id', f'{name}/odom'],
                parameters=[{'use_sim_time': use_sim_time}]))

        vessel_ais_bridge = Node(
            package='hybraut_nav_colreg_sim',
            executable='vessel_ais_bridge',
            name='vessel_ais_bridge',
            output='screen',
            condition=IfCondition(publish_vessel_ais),
            parameters=[{'use_sim_time': use_sim_time, 'ais_config_path': ais_config_path}])

        # Size /map to actually cover wherever this run's fleet ends up,
        # centered on its bounding box (ego + every spawned vessel + the
        # planningProblem's own goal region, reanchored the same way as
        # scenario_goal_sender.py's own copy of this) with a clearance
        # margin so nobody spawns -- and the goal doesn't land -- right at
        # the map's edge.
        MAP_MARGIN_M = 100.0
        xs = [ego_xy[0]] + [x for _, _, x, _, _, _ in vessel_table]
        ys = [ego_xy[1]] + [y for _, _, _, y, _, _ in vessel_table]
        if _scenario_for_ego is not None and _scenario_for_ego.ego_goal_xy is not None:
            goal_x, goal_y = scenario_loader.reanchor_xy(
                _scenario_for_ego.ego_goal_xy, ego_xy, _scenario_for_ego.ego_initial_xy)
            goal_radius = _scenario_for_ego.ego_goal_radius or 0.0
            xs += [goal_x - goal_radius, goal_x + goal_radius]
            ys += [goal_y - goal_radius, goal_y + goal_radius]
        map_center = ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0)
        map_width_m = (max(xs) - min(xs)) + 2 * MAP_MARGIN_M
        map_height_m = (max(ys) - min(ys)) + 2 * MAP_MARGIN_M

        map_publisher = Node(
            package='hybraut_nav_colreg_sim',
            executable='map_publisher',
            name='map_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_width': map_width_m,
                'map_height': map_height_m,
                'map_origin_position': [map_center[0], map_center[1], 0.0],
            }])

        return [spawn_entity, map_publisher] + vessel_nodes + [vessel_ais_bridge] + extra_actions

    build_vessel_fleet = OpaqueFunction(function=_build_vessel_fleet)

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_spawn_vessels,
        declare_publish_vessel_ais,
        declare_scenario_file,
        declare_scenario_seed,
        set_gz_resource_path,
        set_gz_plugin_path,
        gz_sim,
        robot_state_publisher,
        ros_gz_bridge,
        build_vessel_fleet,
    ])

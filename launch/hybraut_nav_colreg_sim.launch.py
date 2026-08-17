
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
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('hybraut_nav_colreg_sim')

    # Gazebo is the clock source for this demo (gz_sim.launch.py bridges
    # /clock), so default to sim time rather than
    # hybraut_nav_tactical_immediate.launch.py's own 'false' default -
    # otherwise tf/rviz drift from the sim clock, per that launch file's
    # own use_sim_time_arg description.
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from Gazebo as the ROS time source - '
                     'passed through to both gz_sim.launch.py and '
                     'hybraut_nav_tactical_immediate.launch.py.')

    # --- gz_sim.launch.py passthrough (world/spawn_vessels/publish_vessel_ais
    # aren't speed-dependent, so these just mirror that launch file's own
    # defaults). ---
    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world', default_value='ocean.world',
        description='World file (in hybraut_nav_colreg_sim/world/) to load, '
                     'e.g. colav_scenario.world.')

    spawn_vessels = LaunchConfiguration('spawn_vessels')
    declare_spawn_vessels = DeclareLaunchArgument(
        'spawn_vessels', default_value='true',
        description='Also spawn every urdf/*_vessel.urdf as an '
                     'independently drivable robot - see gz_sim.launch.py '
                     'for why this isn\'t meant to be combined with '
                     'world:=colav_scenario.world.')

    publish_vessel_ais = LaunchConfiguration('publish_vessel_ais')
    declare_publish_vessel_ais = DeclareLaunchArgument(
        'publish_vessel_ais', default_value='true',
        description='Run vessel_ais_bridge, aggregating the spawned '
                     'traffic vessels onto /obstacles_state. Only '
                     'meaningful alongside spawn_vessels:=true.')

    # --- hybraut_nav_tactical_immediate.launch.py passthrough, sized for
    # the hydrofoil cruising at ~15 kn (~7.7 m/s) rather than that launch
    # file's TurtleBot3-scale defaults. ---
    rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch rviz alongside tactical_node + immediate_node, '
                     'using this project\'s rviz config (see rviz_config '
                     'below). Set to \'false\' to skip it.')

    acceptance_radius = LaunchConfiguration('acceptance_radius')
    declare_acceptance_radius = DeclareLaunchArgument(
        'acceptance_radius', default_value='5.0',
        description='colav_automaton acceptance_radius (m) - how close a '
                     'waypoint counts as reached. At 15 kn (~7.7 m/s) this '
                     'is under 1s of travel, tight enough to hold a course '
                     'without the hydrofoil visibly overshooting waypoints.')

    safety_radius = LaunchConfiguration('safety_radius')
    declare_safety_radius = DeclareLaunchArgument(
        'safety_radius', default_value='20.0',
        description='colav_automaton safety_radius (m) - obstacle '
                     'standoff distance. ~2.6s of travel at 15 kn, enough '
                     'margin to detect-and-divert around traffic without '
                     'the hard turn a smaller radius would force at speed.')

    los_distance_threshold = LaunchConfiguration('los_distance_threshold')
    declare_los_distance_threshold = DeclareLaunchArgument(
        'los_distance_threshold', default_value='40.0',
        description='colav_automaton los_distance_threshold (m) - LOS '
                     'guidance lookahead. ~5s of travel at 15 kn (and 2x '
                     'safety_radius), giving the line-of-sight controller '
                     'enough forward horizon to smooth its heading '
                     'commands instead of chattering at close range.')

    lateral_offset_distance = LaunchConfiguration('lateral_offset_distance')
    declare_lateral_offset_distance = DeclareLaunchArgument(
        'lateral_offset_distance', default_value='50.0',
        description='colav_automaton lateral_offset_distance (m) - how '
                     'far to the side a detour waypoint is placed. 2.5x '
                     'safety_radius, wide enough to clear an obstacle at a '
                     'shallow, foil-friendly turn angle rather than a tight '
                     'course change at 15 kn.')

    longitudinal_offset_distance = LaunchConfiguration('longitudinal_offset_distance')
    declare_longitudinal_offset_distance = DeclareLaunchArgument(
        'longitudinal_offset_distance', default_value='50.0',
        description='colav_automaton longitudinal_offset_distance (m). '
                     'Not currently read by generate_new_virtual_waypoint, '
                     'kept in step with lateral_offset_distance.')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'spawn_vessels': spawn_vessels,
            'publish_vessel_ais': publish_vessel_ais,
        }.items())

    # Override hybraut_nav_tactical_immediate.launch.py's rviz_config arg
    # (defaults to its own rviz/tactical_immediate.rviz) with this
    # project's rviz config instead.
    tactical_immediate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hybraut_nav'),
                         'launch', 'hybraut_nav_tactical_immediate.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'acceptance_radius': acceptance_radius,
            'safety_radius': safety_radius,
            'los_distance_threshold': los_distance_threshold,
            'lateral_offset_distance': lateral_offset_distance,
            'longitudinal_offset_distance': longitudinal_offset_distance,
            'rviz_config': PathJoinSubstitution(
                [FindPackageShare('hybraut_nav_colreg_sim'), 'rviz',
                 'hybraut_nav_tactical_immediate_colreg_demo.rviz']),
        }.items())

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_spawn_vessels,
        declare_publish_vessel_ais,
        declare_rviz,
        declare_acceptance_radius,
        declare_safety_radius,
        declare_los_distance_threshold,
        declare_lateral_offset_distance,
        declare_longitudinal_offset_distance,
        gz_sim,
        tactical_immediate,
    ])

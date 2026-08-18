
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, EnvironmentVariable,
                                   LaunchConfiguration, PathJoinSubstitution,
                                   PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('hybraut_nav_colreg_sim')

    # Gazebo is the clock source for this demo (gz_sim.launch.py bridges
    # /clock), so default to sim time rather than
    # hybraut_nav_strategic_tactical_immediate.launch.py's own 'false'
    # default - otherwise tf/rviz drift from the sim clock, per that launch
    # file's own use_sim_time_arg description.
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from Gazebo as the ROS time source - '
                     'passed through to gz_sim.launch.py, '
                     'hybraut_nav_strategic_tactical_immediate.launch.py, '
                     'and agent_state_bridge.')

    # --- gz_sim.launch.py passthrough (world/spawn_vessels/publish_vessel_ais
    # aren't speed-dependent, so these just mirror that launch file's own
    # defaults). ---
    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world', default_value='ocean.world',
        description='World file (in hybraut_nav_colreg_sim/world/) to load, '
                     'e.g. colav_scenario.world.')

    headless = LaunchConfiguration('headless')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description="Passed straight through to gz_sim.launch.py's own "
                     "'headless' arg (gz sim server-only, no GUI window) - "
                     'and also skips rviz, regardless of the rviz arg below, '
                     'since a scripted/CI verification run has nothing to '
                     'render to. Set true for log-/topic-based verification '
                     'runs; leave false for an actual demo.')

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

    scenario_file = LaunchConfiguration('scenario_file')
    declare_scenario_file = DeclareLaunchArgument(
        'scenario_file', default_value='',
        description='Path to a CommonOcean scenario XML (see scenarios/) to '
                     'drive the traffic-vessel fleet from, instead of '
                     "gz_sim.launch.py's hardcoded default layout. Empty "
                     '(default) keeps the existing 5-vessel fleet unchanged.')

    scenario_seed = LaunchConfiguration('scenario_seed')
    declare_scenario_seed = DeclareLaunchArgument(
        'scenario_seed', default_value='',
        description="Seed for scenario_file's random vessel-type "
                     'assignment, for reproducible runs.')

    # --- hybraut_nav_strategic_tactical_immediate.launch.py passthrough,
    # sized for the hydrofoil cruising at ~15 kn (~7.7 m/s) rather than that launch
    # file's TurtleBot3-scale defaults. ---
    rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch rviz alongside strategy_node + tactical_node + '
                     'immediate_node, using this project\'s rviz config '
                     '(see rviz_config below). Set to \'false\' to skip it.')

    acceptance_radius = LaunchConfiguration('acceptance_radius')
    declare_acceptance_radius = DeclareLaunchArgument(
        'acceptance_radius', default_value='15.0',
        description='colav_automaton acceptance_radius (m) - how close a '
                     'waypoint counts as reached. At 15 kn (~7.7 m/s) this '
                     'is under 1s of travel, tight enough to hold a course '
                     'without the hydrofoil visibly overshooting waypoints.')

    safety_radius = LaunchConfiguration('safety_radius')
    declare_safety_radius = DeclareLaunchArgument(
        'safety_radius', default_value='10.0',
        description='colav_automaton safety_radius (m) - obstacle '
                     'standoff distance. ~2.6s of travel at 15 kn, enough '
                     'margin to detect-and-divert around traffic without '
                     'the hard turn a smaller radius would force at speed.')

    los_distance_threshold = LaunchConfiguration('los_distance_threshold')
    declare_los_distance_threshold = DeclareLaunchArgument(
        'los_distance_threshold', default_value='150.0',
        description='colav_automaton los_distance_threshold (m) - LOS '
                     'guidance lookahead: how close (along the straight '
                     'line to the current waypoint) the unsafe region has '
                     'to come before a reroute (e3, generate_new_virtual_'
                     'waypoint) is triggered. ~5s of travel at 15 kn and 2x '
                     'safety_radius - was left at a stale 200.0 (~26s of '
                     'travel), which routed the agent around traffic it had '
                     "barely noticed yet, well before any real COLREGs "
                     'encounter, instead of a deliberate, demo-legible '
                     'give-way turn.')

    lateral_offset_distance = LaunchConfiguration('lateral_offset_distance')
    declare_lateral_offset_distance = DeclareLaunchArgument(
        'lateral_offset_distance', default_value='20.0',
        description='colav_automaton lateral_offset_distance (m) - how '
                     'far to the side a detour waypoint is placed. 2.5x '
                     'safety_radius, wide enough to clear an obstacle at a '
                     'shallow, foil-friendly turn angle rather than a tight '
                     'course change at 15 kn.')

    longitudinal_offset_distance = LaunchConfiguration('longitudinal_offset_distance')
    declare_longitudinal_offset_distance = DeclareLaunchArgument(
        'longitudinal_offset_distance', default_value='20.0',
        description='colav_automaton longitudinal_offset_distance (m). '
                     'Not currently read by generate_new_virtual_waypoint, '
                     'kept in step with lateral_offset_distance.')

    # risk_envelope_node's own dsf/time_of_interest - the library defaults
    # (200.0m / 100.0s) were previously scaled *up* to 500.0/100.0 to keep
    # the default 5-vessel spawn_vessels fleet (clustered ~30-150m apart)
    # from folding into one unsafe-region "wall" the agent could never
    # route around. That reasoning doesn't carry over to the COLREG_*.xml
    # scenarios this file is mainly used with: their traffic starts
    # 250-600m out, one encounter at a time, so dsf=500.0 classified the
    # obstacle as "of interest" from the very first tick - the risk
    # envelope (and any avoidance bias) existed before the two vessels had
    # any real converging relationship, well outside los_distance_threshold
    # above. Retuned for the COLREG scenarios instead: still comfortably
    # above los_distance_threshold (so the region is "seen" before the
    # agent is inside it), but small enough that the encounter emerges as
    # the vessels actually close rather than from t=0. If running the
    # default fleet (spawn_vessels:=true, no scenario_file) and seeing
    # Fallback braking for a "wall" again, raise this back toward 500.0.
    dsf = LaunchConfiguration('dsf')
    declare_dsf = DeclareLaunchArgument(
        'dsf', default_value='200.0',
        description='risk_envelope_node Distance Safety Factor (m) - how '
                     'close (or how close to closing) a vessel needs to be '
                     'before riskenv.create_unsafe_set folds it into the '
                     'unsafe region. Kept a bit above los_distance_threshold '
                     'so the region is "seen" (and can be routed around) '
                     'before the agent is already inside it.')

    time_of_interest = LaunchConfiguration('time_of_interest')
    declare_time_of_interest = DeclareLaunchArgument(
        'time_of_interest', default_value='45.0',
        description='risk_envelope_node TCPA horizon (s) for its I3 '
                     '(predicted-closest-approach) filter. Library default '
                     'is 15.0; this file previously used 100.0, which - for '
                     "a ~90-120s COLREG_*.xml scenario - flags a converging "
                     'obstacle as "of interest" for nearly the whole run. '
                     'classify_unsafe_set_obstacles then has no distance-in-'
                     "time to temper a rule's base urgency by (Rule 14 "
                     'head-on always starts at 0.9), so a still-distant '
                     'ship can dominate aggregate_maneuvers well before '
                     'there is a genuine encounter. 45.0 keeps Rule 8\'s '
                     '"early and substantial" give-way action without '
                     'spanning nearly the whole scenario.')

    k_theta = LaunchConfiguration('k_theta')
    declare_k_theta = DeclareLaunchArgument(
        'k_theta', default_value='1.2',
        description="colav_automaton k_theta - LOS heading gain on the "
                     'reference trajectory (see hybraut_nav_strategic_'
                     'tactical_immediate.launch.py\'s own k_theta_arg for '
                     'the general explanation). Nudged up from that file\'s '
                     'library default (1.0) so the reference swings onto a '
                     'freshly-generated COLAV detour waypoint promptly - the '
                     'real turn rate is still bounded downstream by '
                     "immediate_node's controller, this just keeps the "
                     "tactical layer's own reference from being the lagging "
                     'part of that chain.')

    constant_velocity = LaunchConfiguration('constant_velocity')
    declare_constant_velocity = DeclareLaunchArgument(
        'constant_velocity', default_value='7.7',
        description='colav_automaton constant_velocity (m/s) AND '
                     "immediate_node's desired_velocity (m/s) - one launch "
                     "arg driving both, since they're supposed to agree "
                     '(see hybraut_nav_strategic_tactical_immediate.launch.py\'s '
                     'own constant_velocity_arg/desired_velocity_arg for '
                     'why). Was previously two unset params silently at '
                     'their own defaults - tactical_node\'s reference speed '
                     'at 2.0 m/s, immediate_node\'s real commanded speed at '
                     "7.2 m/s - despite this whole file's own framing "
                     'around a 15 kn (~7.7 m/s) hydrofoil cruise. Set to '
                     '7.7 m/s (15 kn) here so both agree with that framing '
                     'and with the COLREG_*.xml scenarios\' own retimed '
                     'obstacle trajectories (see scenarios/*.xml - each '
                     "one's crossing/meeting geometry is timed against this "
                     'exact speed).')

    fallback_timeout = LaunchConfiguration('fallback_timeout')
    declare_fallback_timeout = DeclareLaunchArgument(
        'fallback_timeout', default_value='60.0',
        description='colav_automaton fallback_timeout (s). Was previously '
                     "silently unset (tactical_node didn't even pass it to "
                     'ColavAutomaton) at the library default of 30.0 - live '
                     'verification runs against the COLREG_*.xml scenarios '
                     'showed a fast-closing encounter can occasionally need '
                     "longer than that purely for the *other* vessel to "
                     'clear (Fallback brakes/holds heading and waits, it '
                     "doesn't actively out-manoeuvre anything), not because "
                     'the leg is genuinely stuck - the same scenario has '
                     'been observed to recover from Fallback in as little as '
                     '~3s and, on a slower run, still be waiting at the 30s '
                     'mark. Doubled to 60.0 to give those recoverable cases '
                     'the real time they need rather than aborting the leg '
                     'underneath them. If a scenario still fails with '
                     "'Invariant violated in mode Fallback' at 60s, that's a "
                     'genuine stuck case, not just a timeout that was too '
                     'tight.')

    # risk_envelope_node: the missing wire between vessel_ais_bridge's
    # /obstacles_state (gz_sim.launch.py, gated on publish_vessel_ais) and
    # tactical_node's /hybraut_nav/riskenv + /hybraut_nav/maneuver_bias
    # inputs (see vessel_ais_bridge.py's own docstring - it exists
    # specifically to let this pipeline run end-to-end). Only meaningful
    # alongside publish_vessel_ais:=true, same gate vessel_ais_bridge itself
    # uses - without it there's no /obstacles_state to synchronise against,
    # so this would just idle and warn on timeout.
    risk_envelope_node = Node(
        package='hybraut_nav',
        executable='risk_envelope_node',
        output='screen',
        condition=IfCondition(publish_vessel_ais),
        parameters=[{
            'use_sim_time': use_sim_time,
            # same "how close is safe" quantity as tactical_node's own
            # safety_radius below - reusing that launch arg keeps the
            # agent's own safety domain consistent between the risk
            # envelope that's built around it and the automaton routing
            # around that envelope.
            'safety_radius': safety_radius,
            'dsf': dsf,
            'time_of_interest': time_of_interest,
        }])

    # Republishes the ego's /odom as /agent_state - strategy_node's own
    # required world-state input (see hybraut_nav/scripts/demo/
    # agent_state_bridge.py's own docstring). The ego's /odom is already
    # bare/unnamespaced (config/ros_gz_bridge.yaml), so this needs no remap.
    agent_state_bridge = Node(
        package='hybraut_nav',
        executable='agent_state_bridge',
        name='agent_state_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Sends scenario_file's own planningProblem/goalState (see
    # scenario_loader.py) to strategy_node's navigate_to_goal action as a
    # single-waypoint mission -- see scenario_goal_sender.py's own module
    # docstring. Only meaningful (and only started) alongside a
    # scenario_file that actually has a <planningProblem>/<goalState> --
    # every scenarios/*.xml file does today, including scenarios/COLREG_*.xml.
    scenario_goal_sender = Node(
        package='hybraut_nav_colreg_sim',
        executable='scenario_goal_sender',
        name='scenario_goal_sender',
        output='screen',
        condition=IfCondition(PythonExpression(['"', scenario_file, '" != ""'])),
        parameters=[{
            'scenario_file': scenario_file,
            'use_sim_time': use_sim_time,
        }])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'headless': headless,
            'spawn_vessels': spawn_vessels,
            'publish_vessel_ais': publish_vessel_ais,
            'scenario_file': scenario_file,
            'scenario_seed': scenario_seed,
        }.items())

    # strategy_node checks each scenario goal is reachable (via A*, against
    # map_publisher's own /map above) and drives tactical_node through it -
    # see hybraut_nav_strategic_tactical_immediate.launch.py's own module
    # docstring. Overrides its rviz_config arg (defaults to its own
    # rviz/strategic_tactical_immediate.rviz) with this project's rviz
    # config instead, same as the tactical_immediate pairing it replaces.
    strategic_tactical_immediate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hybraut_nav'),
                         'launch', 'hybraut_nav_strategic_tactical_immediate.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # rviz has nothing to render to during a headless verification
            # run - force it off regardless of the rviz arg's own value.
            'rviz': PythonExpression(
                ['"true" if ("', rviz, '" == "true" and "', headless, '" != "true") else "false"']),
            'acceptance_radius': acceptance_radius,
            'safety_radius': safety_radius,
            'los_distance_threshold': los_distance_threshold,
            'lateral_offset_distance': lateral_offset_distance,
            'longitudinal_offset_distance': longitudinal_offset_distance,
            'k_theta': k_theta,
            'constant_velocity': constant_velocity,
            'desired_velocity': constant_velocity,
            'fallback_timeout': fallback_timeout,
            'rviz_config': PathJoinSubstitution(
                [FindPackageShare('hybraut_nav_colreg_sim'), 'rviz',
                 'hybraut_nav_strategic_tactical_immediate_colreg_demo.rviz']),
        }.items())

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_headless,
        declare_spawn_vessels,
        declare_publish_vessel_ais,
        declare_scenario_file,
        declare_scenario_seed,
        declare_rviz,
        declare_acceptance_radius,
        declare_safety_radius,
        declare_los_distance_threshold,
        declare_lateral_offset_distance,
        declare_longitudinal_offset_distance,
        declare_dsf,
        declare_time_of_interest,
        declare_k_theta,
        declare_constant_velocity,
        declare_fallback_timeout,
        gz_sim,
        risk_envelope_node,
        strategic_tactical_immediate,
        agent_state_bridge,
        scenario_goal_sender,
    ])

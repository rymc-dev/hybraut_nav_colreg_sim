#!/usr/bin/env python3
"""
scenario_goal_sender

Sends a scenario file's <planningProblem>/<goalState> waypoint(s) to
strategy_node's own navigate_to_goal action
(/hybraut_nav/strategy_node/navigate_to_goal, hybraut_nav/action/NavigateToGoal)
as an ordered goal_waypoints list -- one entry per <planningProblem>/
<goalState> in the file, in document order (see
scenario_loader.ScenarioData.ego_goal_waypoints). Most of the COLREG demo
scenarios (scenarios/COLREG_*.xml) have exactly one, so that's still a
single-waypoint mission for them; a scenario authored with several (e.g.
scenarios/COLREG_MULTI_WAYPOINT-1.xml) becomes a real multi-leg route --
strategy_node already dispatches an ordered goal_waypoints list one leg at a
time (see its own module docstring), this is just the first caller in this
package to actually hand it more than one. Routing it through strategy_node
still gets every waypoint checked for reachability (via A*, against
map_publisher's own /map) before tactical_node is ever sent anything,
instead of handing tactical_node an unchecked target directly.

Each goal position is reanchored (scenario_loader.reanchor_xy) by the same
delta gz_sim.launch.py/obstacle_vessel_controler.py use to reanchor the
ego/obstacles onto vessel_catalog.EGO_SPAWN_XY -- without this, a
scenario's own recorded goal coordinates (kilometers away, or UTM-scale for
some real Marine Cadastre files) would send the ego toward a point nothing
in this small demo world is actually near, exactly the "huge values"
problem already fixed for spawn positions.

Only meaningful for a scenario_file with at least one <planningProblem>/
<goalState> -- raises at startup if the given file has none.

Unlike sending straight to tactical_node (which needs neither), strategy_node
rejects a mission outright if it hasn't yet received both /map and
/agent_state -- so this also blocks at startup (wait_for_world_state) until
both have delivered at least one message, closing the launch-order race
between this node's own startup and map_publisher/agent_state_bridge's.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.wait_for_message import wait_for_message
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from hybraut_nav.action import NavigateToGoal
from hybraut_nav.msg import AgentState, Waypoint
from hybraut_nav.qos import map_qos, world_state_qos

from hybraut_nav_colreg_sim import scenario_loader, vessel_catalog


class ScenarioGoalSender(Node):

    def __init__(self):
        super().__init__('scenario_goal_sender')

        self.declare_parameter(
            'scenario_file', '',
            ParameterDescriptor(
                description='Path to the CommonOcean scenario XML whose '
                            'planningProblem/goalState to send (see '
                            'scenario_loader.py). Required.',
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'mission_tag', 'colreg_scenario',
            ParameterDescriptor(
                description='mission_tag stamped on the NavigateToGoal '
                            'goal -- free-form, for correlation/logging on '
                            'the strategy_node/tactical_node side.',
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'acceptance_radius', 5.0,
            ParameterDescriptor(
                description="Fallback Waypoint.acceptance_radius (m) for "
                            "any waypoint whose own goalState is a bare "
                            "point (no region to derive one from). "
                            "Informational "
                            "only on tactical_node's side (strategy_node "
                            "dispatches this waypoint on to tactical_node's "
                            "own execute_mission leg) -- see "
                            'hybraut_nav/action/ExecuteMission.action\'s '
                            'own note that colav_automaton uses a '
                            "fixed startup param instead, not this "
                            'per-leg field.',
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'server_wait_timeout', 30.0,
            ParameterDescriptor(
                description='Seconds to wait for the '
                            'strategy_node/navigate_to_goal action server '
                            'to come up before giving up.',
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'world_state_wait_timeout', 15.0,
            ParameterDescriptor(
                description='Seconds to wait for /map (map_publisher) and '
                            '/agent_state (agent_state_bridge) to each '
                            'deliver at least one message before sending '
                            'the goal -- strategy_node rejects a mission '
                            'outright without both, so this closes the '
                            'launch-order race against those two nodes '
                            'coming up.',
                type=ParameterType.PARAMETER_DOUBLE))

        scenario_path = self.get_parameter('scenario_file').value
        if not scenario_path:
            raise ValueError(
                'scenario_goal_sender requires a scenario_file parameter '
                '(set by hybraut_nav_colreg_sim.launch.py when '
                'scenario_file:=<path> is given)')

        scenario = scenario_loader.parse_scenario(scenario_path)
        if not scenario.ego_goal_waypoints:
            raise ValueError(
                f"scenario_goal_sender: '{scenario_path}' has no "
                'planningProblem/goalState to send')

        # Same anchor gz_sim.launch.py/obstacle_vessel_controler.py spawn
        # the ego at -- see vessel_catalog.EGO_SPAWN_XY's own docstring for
        # why this is always the fixed small anchor, never the scenario's
        # own recorded (possibly kilometers/UTM-scale) ego position. Every
        # waypoint is reanchored by the same delta, so a multi-waypoint
        # route keeps its shape (leg lengths/bearings) relative to wherever
        # the ego actually spawns.
        default_radius = self.get_parameter('acceptance_radius').value
        goal_waypoints = []
        for xy, radius in scenario.ego_goal_waypoints:
            goal_xy = scenario_loader.reanchor_xy(
                xy, vessel_catalog.EGO_SPAWN_XY, scenario.ego_initial_xy)
            goal_waypoints.append(Waypoint(
                position=Point(x=goal_xy[0], y=goal_xy[1], z=0.0),
                acceptance_radius=radius if radius is not None else default_radius))

        self._action_client = ActionClient(
            self, NavigateToGoal, '/hybraut_nav/strategy_node/navigate_to_goal')

        timeout = self.get_parameter('server_wait_timeout').value
        self.get_logger().info(
            f'scenario_goal_sender: waiting up to {timeout}s for '
            'strategy_node/navigate_to_goal...')
        if not self._action_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                'scenario_goal_sender: strategy_node/navigate_to_goal '
                'action server unavailable')

        self._wait_for_world_state()

        goal = NavigateToGoal.Goal()
        goal.stamp = self.get_clock().now().to_msg()
        goal.mission_tag = self.get_parameter('mission_tag').value
        goal.goal_waypoints = goal_waypoints

        waypoints_desc = ', '.join(
            f"({wp.position.x:.1f}, {wp.position.y:.1f}, r={wp.acceptance_radius:.1f}m)"
            for wp in goal_waypoints)
        self.get_logger().info(
            f"scenario_goal_sender: sending {len(goal_waypoints)}-waypoint "
            f"mission [{waypoints_desc}] from '{scenario_path}'")
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _wait_for_world_state(self):
        """Blocks until /map and /agent_state have each delivered at least
        one message -- see module docstring for why this matters here (it
        wouldn't have for a goal sent straight to tactical_node)."""
        timeout = self.get_parameter('world_state_wait_timeout').value

        self.get_logger().info(
            f'scenario_goal_sender: waiting up to {timeout}s for /map...')
        got_map, _ = wait_for_message(
            OccupancyGrid, self, '/map', qos_profile=map_qos, time_to_wait=timeout)
        if not got_map:
            raise RuntimeError(
                'scenario_goal_sender: no /map received - is map_publisher up?')

        self.get_logger().info(
            f'scenario_goal_sender: waiting up to {timeout}s for /agent_state...')
        got_agent_state, _ = wait_for_message(
            AgentState, self, '/agent_state', qos_profile=world_state_qos, time_to_wait=timeout)
        if not got_agent_state:
            raise RuntimeError(
                'scenario_goal_sender: no /agent_state received - is '
                'agent_state_bridge up?')

    def _on_feedback(self, feedback_msg):
        self.get_logger().info(
            f'scenario_goal_sender: automaton_state='
            f'{feedback_msg.feedback.tactical_automaton_state}',
            throttle_duration_sec=5.0)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'scenario_goal_sender: strategy_node rejected the goal')
            return
        self.get_logger().info(
            'scenario_goal_sender: goal accepted, awaiting result')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'scenario_goal_sender: goal reached - {result.message}')
        else:
            self.get_logger().warn(
                f'scenario_goal_sender: goal not reached - {result.message}')


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioGoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
scenario_goal_sender

Sends a scenario file's <planningProblem>/<goalState> straight to
tactical_node's own execute_mission action
(/hybraut_nav/tactical_node/execute_mission, hybraut_nav/action/
ExecuteMission) as a single leg -- for the COLREG demo scenarios
(scenarios/COLREG_*.xml) that's the whole point: one ego, one obstacle, one
goal, no multi-leg route to plan, so there's no need to go through
strategy_node's own NavigateToGoal mission layer (see
hybraut_nav_strategy/strategy_node.py's own module docstring for how that
one decomposes a goal into legs and dispatches them here one at a time --
this node instead sends the single goal directly, the way `ros2 action
send_goal` against that same action does).

The goal position is reanchored (scenario_loader.reanchor_xy) by the same
delta gz_sim.launch.py/obstacle_vessel_controler.py use to reanchor the
ego/obstacles onto vessel_catalog.EGO_SPAWN_XY -- without this, a
scenario's own recorded goal coordinates (kilometers away, or UTM-scale for
some real Marine Cadastre files) would send the ego toward a point nothing
in this small demo world is actually near, exactly the "huge values"
problem already fixed for spawn positions.

Only meaningful for a scenario_file with a <planningProblem>/<goalState> --
raises at startup if the given file has neither.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Point

from hybraut_nav.action import ExecuteMission
from hybraut_nav.msg import Waypoint

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
                description='mission_tag stamped on the ExecuteMission '
                            'goal -- free-form, for correlation/logging on '
                            'the tactical_node side.',
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'acceptance_radius', 5.0,
            ParameterDescriptor(
                description="Fallback Waypoint.acceptance_radius (m) when "
                            "the scenario's goalState is a bare point (no "
                            "region to derive one from). Informational "
                            "only on tactical_node's side -- see "
                            'hybraut_nav/action/ExecuteMission.action\'s '
                            'own note that colav_automaton uses a '
                            "fixed startup param instead, not this "
                            'per-leg field.',
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'server_wait_timeout', 30.0,
            ParameterDescriptor(
                description='Seconds to wait for the '
                            'tactical_node/execute_mission action server '
                            'to come up before giving up.',
                type=ParameterType.PARAMETER_DOUBLE))

        scenario_path = self.get_parameter('scenario_file').value
        if not scenario_path:
            raise ValueError(
                'scenario_goal_sender requires a scenario_file parameter '
                '(set by hybraut_nav_colreg_sim.launch.py when '
                'scenario_file:=<path> is given)')

        scenario = scenario_loader.parse_scenario(scenario_path)
        if scenario.ego_goal_xy is None:
            raise ValueError(
                f"scenario_goal_sender: '{scenario_path}' has no "
                'planningProblem/goalState to send')

        # Same anchor gz_sim.launch.py/obstacle_vessel_controler.py spawn
        # the ego at -- see vessel_catalog.EGO_SPAWN_XY's own docstring for
        # why this is always the fixed small anchor, never the scenario's
        # own recorded (possibly kilometers/UTM-scale) ego position.
        goal_xy = scenario_loader.reanchor_xy(
            scenario.ego_goal_xy, vessel_catalog.EGO_SPAWN_XY, scenario.ego_initial_xy)
        radius = (scenario.ego_goal_radius if scenario.ego_goal_radius is not None
                  else self.get_parameter('acceptance_radius').value)

        self._action_client = ActionClient(
            self, ExecuteMission, '/hybraut_nav/tactical_node/execute_mission')

        timeout = self.get_parameter('server_wait_timeout').value
        self.get_logger().info(
            f'scenario_goal_sender: waiting up to {timeout}s for '
            'tactical_node/execute_mission...')
        if not self._action_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                'scenario_goal_sender: tactical_node/execute_mission action '
                'server unavailable')

        goal = ExecuteMission.Goal()
        goal.stamp = self.get_clock().now().to_msg()
        goal.mission_tag = self.get_parameter('mission_tag').value
        goal.goal_waypoint = Waypoint(
            position=Point(x=goal_xy[0], y=goal_xy[1], z=0.0),
            acceptance_radius=radius)

        self.get_logger().info(
            f"scenario_goal_sender: sending goal ({goal_xy[0]:.1f}, "
            f"{goal_xy[1]:.1f}), acceptance_radius={radius:.1f}m, from "
            f"'{scenario_path}'")
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg):
        self.get_logger().info(
            f'scenario_goal_sender: automaton_state='
            f'{feedback_msg.feedback.automaton_state}',
            throttle_duration_sec=5.0)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'scenario_goal_sender: tactical_node rejected the goal')
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

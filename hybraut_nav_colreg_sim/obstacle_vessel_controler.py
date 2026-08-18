#!/usr/bin/env python3
"""
obstacle_vessel_controler

Replays each scenario-driven traffic vessel's full recorded CommonOcean
trajectory (position/heading/speed over time -- see scenario_loader.py) as
geometry_msgs/Twist commands on its /<tag>/cmd_vel, so it actually moves
along its recorded track instead of sitting static after spawn.

Only started by gz_sim.launch.py when scenario_file:=<path> is given.
scenario_file and scenario_seed here are the exact same values that launch
file resolved and used to spawn the fleet -- this node reruns the identical
parse -> ship_obstacles -> reanchor -> assign_vessel_types pipeline
(scenario_loader.py) itself rather than being handed a pre-built vessel
list, so scenario_seed must not be left empty by the time this node starts
(gz_sim.launch.py always resolves it to a concrete value first, via
scenario_loader.resolve_seed, specifically so both sides agree).
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Twist

from hybraut_nav_colreg_sim import scenario_loader, vessel_catalog


def _lerp(a, b, frac):
    return a + (b - a) * frac


def _angle_diff(a, b):
    """Shortest signed difference b - a, wrapped to (-pi, pi]."""
    return math.atan2(math.sin(b - a), math.cos(b - a))


class ObstacleVesselControler(Node):

    def __init__(self):
        super().__init__('obstacle_vessel_controler')

        self.declare_parameter(
            'scenario_file', '',
            ParameterDescriptor(
                description='Path to the CommonOcean scenario XML to replay '
                            '(see scenario_loader.py). Required -- set by '
                            'gz_sim.launch.py, not meant to be hand-authored.',
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'scenario_seed', '',
            ParameterDescriptor(
                description="The concrete (already-resolved) random seed "
                            "gz_sim.launch.py used to assign vessel types to "
                            "this scenario's obstacles, so this node "
                            "reconstructs the identical fleet. Required.",
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'control_rate', 2.0,
            ParameterDescriptor(
                description='Rate (Hz) to publish interpolated cmd_vel '
                            'commands. (default: 2.0)',
                type=ParameterType.PARAMETER_DOUBLE))

        scenario_path = self.get_parameter('scenario_file').value
        seed_str = self.get_parameter('scenario_seed').value
        if not scenario_path or not seed_str:
            raise ValueError(
                'obstacle_vessel_controler requires both scenario_file and '
                'scenario_seed parameters (set by gz_sim.launch.py when '
                'scenario_file:=<path> is given)')

        scenario = scenario_loader.parse_scenario(scenario_path)
        ships, skipped = scenario_loader.ship_obstacles(scenario)
        if skipped:
            self.get_logger().info(
                f'obstacle_vessel_controler: not replaying {len(skipped)} '
                f'non-ship scenario obstacle(s): '
                f'{[(o.id, o.raw_type) for o in skipped]}')
        ships = scenario_loader.reanchor(
            ships, vessel_catalog.EGO_SPAWN_XY, scenario.ego_initial_xy)
        assignment = scenario_loader.assign_vessel_types(
            ships, vessel_catalog.VESSEL_TYPES, seed=int(seed_str))

        self._time_step_size = scenario.time_step_size
        self._vessels = []  # list of (tag, trajectory, publisher)
        for obstacle in ships:
            tag = scenario_loader.vessel_tag(assignment[obstacle.id], obstacle.id)
            publisher = self.create_publisher(Twist, f'/{tag}/cmd_vel', 10)
            self._vessels.append((tag, obstacle.trajectory, publisher))

        if not self._vessels:
            raise ValueError(
                f"obstacle_vessel_controler: '{scenario_path}' has no "
                'ship-like obstacles to replay')

        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(
            1.0 / self.get_parameter('control_rate').value, self._tick)

        self.get_logger().info(
            f'obstacle_vessel_controler up - replaying '
            f'{[tag for tag, _, _ in self._vessels]} from {scenario_path!r}')

    def _elapsed(self, trajectory, sample):
        """Seconds since replay start for a recorded sample -- trajectory[0]
        (== the obstacle's initialState) is always t=0, regardless of what
        raw scenario timestep it started at."""
        return (sample.t - trajectory[0].t) * self._time_step_size

    def _bracket(self, trajectory, now_s):
        """Return (prev, nxt, frac) bracketing now_s. Before the first
        sample or after the last, holds that edge sample (frac=0.0) rather
        than extrapolating -- see the module docstring's note on holding
        the final velocity once a trajectory finishes."""
        first, last = trajectory[0], trajectory[-1]
        if now_s <= self._elapsed(trajectory, first):
            return first, first, 0.0
        if now_s >= self._elapsed(trajectory, last):
            return last, last, 0.0
        for prev, nxt in zip(trajectory, trajectory[1:]):
            t_prev, t_nxt = self._elapsed(trajectory, prev), self._elapsed(trajectory, nxt)
            if t_prev <= now_s <= t_nxt:
                frac = (now_s - t_prev) / (t_nxt - t_prev) if t_nxt > t_prev else 0.0
                return prev, nxt, frac
        return last, last, 0.0

    def _tick(self):
        now_s = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        for tag, trajectory, publisher in self._vessels:
            prev, nxt, frac = self._bracket(trajectory, now_s)
            v = _lerp(prev.v, nxt.v, frac)

            dt = self._elapsed(trajectory, nxt) - self._elapsed(trajectory, prev)
            yaw_rate = _angle_diff(prev.yaw, nxt.yaw) / dt if dt > 0.0 else 0.0

            twist = Twist()
            # Body-frame cmd_vel -- same convention hybraut_nav's own
            # immediate_node.py uses to drive the real ego (plain forward
            # speed + turn rate). gz-sim-velocity-control-system applies
            # linear.x/y in the model's own local frame (no <odom_frame>
            # override on urdf/*_vessel.urdf's VelocityControl block), NOT
            # world frame -- publishing world-frame-decomposed
            # v*cos(yaw)/v*sin(yaw) here (as this used to) makes the plugin
            # apply the model's own current rotation on top, sending a
            # vessel spawned/turned away from yaw=0 the wrong direction.
            # angular.z steers the model's heading to track the recorded
            # course; recomputing yaw_rate fresh from interpolation every
            # tick (rather than integrating it once) keeps this
            # self-correcting against any drift in the plugin's own
            # orientation integration.
            twist.linear.x = v
            twist.angular.z = yaw_rate
            publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleVesselControler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

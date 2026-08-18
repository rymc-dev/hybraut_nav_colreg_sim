#!/usr/bin/env python3
"""
vessel_ais_bridge

Mocks the "AIS other-traffic" side of hybraut_nav's real risk_envelope_node
pipeline: combines each traffic vessel's static particulars (mocked, from
config/mock_ais_vessels.yaml -- tag/type/mmsi/loa/beam/safety_radius) with
its live dynamic state (real, not mocked -- read straight from /<tag>/odom,
bridged in config/ros_gz_bridge.yaml whenever launch/gz_sim.launch.py is run
with spawn_vessels:=true) into hybraut_nav/msg/ObstaclesState on
/obstacles_state, the input risk_envelope_node actually expects. Pairs with
hybraut_nav's own agent_state_bridge (ego side, republishing /odom as
/agent_state) to let the real risk_envelope_node -> tactical_node pipeline
run end-to-end against these 5 vessel models.

Not a replacement for hybraut_nav's own obstacles_state_bridge -- that one
aggregates arbitrary named movers under one flat beam/loa/safety_radius for
all of them; this one is specific to this package's 5 vessel models, each
keeping its own particulars from the YAML.

static_obstacles is always left empty -- only the 5 dynamic traffic vessels
are modelled here.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ament_index_python.packages import get_package_share_directory

import yaml

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from hybraut_nav.msg import ObstaclesState, DynamicObstacleState, DynamicObstacleGeometry

from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray

class VesselAisBridge(Node):

    def __init__(self):
        super().__init__('vessel_ais_bridge')

        default_config_path = (
            f"{get_package_share_directory('hybraut_nav_colreg_sim')}"
            "/config/mock_ais_vessels.yaml"
        )
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter(
            'ais_config_path', default_config_path,
            ParameterDescriptor(description='Path to the per-vessel mock AIS particulars YAML '
                                             "(see that file's own comments for the field shape). "
                                             f'(default: {default_config_path})',
                                 type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            'publish_rate', 10.0,
            ParameterDescriptor(description='Rate (Hz) to publish the aggregated ObstaclesState. '
                                             '(default: 10.0)',
                                 type=ParameterType.PARAMETER_DOUBLE)
        )

        config_path = self.get_parameter('ais_config_path').value
        self._vessels = self._load_vessels(config_path)
        if not self._vessels:
            raise ValueError(f"vessel_ais_bridge: no vessels found in ais_config_path '{config_path}'")

        # populated as each vessel's /<tag>/odom starts publishing -- a
        # vessel is only included in the aggregated ObstaclesState once its
        # first odom message has arrived, same as hybraut_nav's own
        # obstacles_state_bridge.
        self._latest_odom = {}
        self._subs = [
            self.create_subscription(
                Odometry,
                f"/{vessel['tag']}/odom",
                self._make_odom_cb(vessel['tag']),
                qos_profile_system_default,
            )
            for vessel in self._vessels
        ]
        self._subs.append(
            self.create_subscription(
                Odometry,
                "/odom",
                self._make_odom_cb('agent'),
                qos_profile_system_default
            )
        )

        self._obstacles_state_pub = self.create_publisher(
            ObstaclesState,
            '/obstacles_state',
            qos_profile_system_default,
        )
        self._safety_radius_pub = self.create_publisher(
            MarkerArray,
            "/safety_domain_markers",
            10
        )
        self._timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self._publish_cb,
        )

        self.get_logger().info(
            f"vessel_ais_bridge up - tracking {[v['tag'] for v in self._vessels]} "
            "-> /obstacles_state"
        )
        
    @staticmethod
    def make_cylinder(id_, x, y, z, radius, height, frame="map", rgba=(1.0, 0.65, 0.0, 0.3)):
        m = Marker()
        m.header.frame_id = frame
        m.ns = "safety_domains"
        m.id = id_
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z + (height / 2)
        m.pose.orientation.w = 1.0
        m.scale.x = radius * 2   # diameter, not radius
        m.scale.y = radius * 2
        m.scale.z = height
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime = Duration(seconds=0).to_msg()  # 0 = persists until replaced
        return m
 
    @staticmethod
    def _load_vessels(config_path: str) -> list:
        with open(config_path) as f:
            data = yaml.safe_load(f) or {}
        return data.get('vessels', [])

    def _make_odom_cb(self, tag: str):
        def _cb(msg: Odometry):
            self._latest_odom[tag] = msg
        return _cb

    def _publish_cb(self):
        frame_id = self.get_parameter('frame_id').value

        obstacles_state = ObstaclesState()
        obstacles_state.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=frame_id)
        obstacles_state.static_obstacles = []
        obstacles_state.dynamic_obstacles = []
        safety_domain_markers = MarkerArray()

        for idx, vessel in enumerate(self._vessels):
            odom = self._latest_odom.get(vessel['tag'])
            if odom is None:
                # no odom for this vessel yet -- skip until its first message arrives,
                # same as hybraut_nav's own obstacles_state_bridge.
                continue

            # the agent isn't a traffic obstacle -- its dynamic state reaches
            # risk_envelope_node via hybraut_nav's own agent_state_bridge on
            # /agent_state. Including it here too would make it its own
            # obstacle (zero relative range -> NaN in the risk envelope's
            # convex hull). Still draw its safety-domain marker below.
            if vessel['tag'] != "agent":
                obstacles_state.dynamic_obstacles.append(self._to_dynamic_obstacle(vessel, odom))

            pos = odom.pose.pose.position
            safety_domain_markers.markers.append(
                self.make_cylinder(
                    idx, pos.x, pos.y, 0.0,
                    vessel['safety_radius'], height=5.0, frame=frame_id,
                    rgba=(1.0, 0.65, 0.0, 0.3) if vessel['tag'] == "agent" else (0.0, 1.0, 0.0, 0.3),
                )
            )


        self._obstacles_state_pub.publish(obstacles_state)
        self._safety_radius_pub.publish(safety_domain_markers)

    @staticmethod
    def _to_dynamic_obstacle(vessel: dict, odom: Odometry) -> DynamicObstacleState:
        return DynamicObstacleState(
            tag=vessel['tag'],
            type=vessel['type'],
            pose=odom.pose.pose,
            velocity=math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y),
            acceleration=0.0,
            yaw_rate=odom.twist.twist.angular.z,
            geometry=DynamicObstacleGeometry(loa=vessel['loa'], beam=vessel['beam']),
            safety_radius=vessel['safety_radius'],
        )


def main(args=None):
    rclpy.init(args=args)
    node = VesselAisBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# strategic layer requires map for global planning
# therefore we utilize this node to publish static map information
# about the environment including static world -> map (and world -> odom)
# transforms as well as default map occupancy
#
# The TF tree here is actually rooted at "odom" -- the ego's
# gz-sim-odometry-publisher-system publishes odom -> base_link, and every
# scenario vessel spawned by gz_sim.launch.py is bridged to that same shared
# "odom" via its own identity static transform. Without a world -> odom
# bridge too, "world -> map" would just be a second, disconnected TF tree --
# same problem (and same identity-transform fix, since odom already
# coincides with world origin) as hybraut_nav's own
# scripts/demo/fake_map_publisher.py works around for a bare TurtleBot3 sim.

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid


from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        self.declare_parameter(
            'map_resolution', 1.0,
            ParameterDescriptor(
                description='Metres per grid cell.',
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'map_width', 400.0,
            ParameterDescriptor(
                description='Map width in metres (not cells).',
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'map_height', 400.0,
            ParameterDescriptor(
                description='Map height in metres (not cells).',
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'map_origin_position', [0.0, 0.0, 0.0],
            ParameterDescriptor(
                description='[x, y, z] world-frame position of the centre '
                            'of the map -- the world -> map static '
                            'transform translation.',
                type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter(
            'map_origin_orientation', [0.0, 0.0, 0.0, 1.0],
            ParameterDescriptor(
                description='[x, y, z, w] quaternion for the world -> map '
                            'static transform rotation.',
                type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter(
            'odom_frame_id', 'odom',
            ParameterDescriptor(
                description='child frame_id for the static world -> odom '
                            'bridge transform this node broadcasts, '
                            "connecting the ego/vessels' shared TF root "
                            "into this node's own world -> map tree. Set "
                            "to '' to skip broadcasting it (e.g. a real "
                            'localization source already provides it).',
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'publish_odom_transform', True,
            ParameterDescriptor(
                description='Whether to broadcast the static world -> odom '
                            'transform at all.',
                type=ParameterType.PARAMETER_BOOL))

        self.tf_global_frame_broadcaster: StaticTransformBroadcaster = StaticTransformBroadcaster(self)
        self.occupancy_map_pub = self.create_publisher(
            OccupancyGrid,
            "/map",
            qos_profile=qos_profile_system_default
        )
        self.publish_global_frame()
        self.create_timer(
            1.0,
            self.publish_occupancy_grid,
            callback_group=ReentrantCallbackGroup()
        )

    def publish_global_frame(self):
        transforms = []

        origin_position = self.get_parameter('map_origin_position').value
        origin_orientation = self.get_parameter('map_origin_orientation').value

        world_to_map = TransformStamped()
        world_to_map.header.stamp = self.get_clock().now().to_msg()
        world_to_map.header.frame_id = 'world'
        world_to_map.child_frame_id = 'map'
        world_to_map.transform.translation.x = origin_position[0]
        world_to_map.transform.translation.y = origin_position[1]
        world_to_map.transform.translation.z = origin_position[2]
        world_to_map.transform.rotation.x = origin_orientation[0]
        world_to_map.transform.rotation.y = origin_orientation[1]
        world_to_map.transform.rotation.z = origin_orientation[2]
        world_to_map.transform.rotation.w = origin_orientation[3]
        transforms.append(world_to_map)

        odom_frame_id = self.get_parameter('odom_frame_id').value
        if self.get_parameter('publish_odom_transform').value and odom_frame_id:
            # Identity is correct here, not an offset -- odom already
            # coincides with world origin (see the OdometryPublisher plugin
            # config on urdf/hydrofoil.urdf and gz_sim.launch.py's own
            # {name}_odom_to_world transforms, which rely on the same fact).
            world_to_odom = TransformStamped()
            world_to_odom.header.stamp = self.get_clock().now().to_msg()
            world_to_odom.header.frame_id = 'world'
            world_to_odom.child_frame_id = odom_frame_id
            world_to_odom.transform.rotation.w = 1.0
            transforms.append(world_to_odom)

        self.tf_global_frame_broadcaster.sendTransform(transforms)

    def publish_occupancy_grid(self):
        resolution = self.get_parameter('map_resolution').value
        width_cells = round(self.get_parameter('map_width').value / resolution)
        height_cells = round(self.get_parameter('map_height').value / resolution)

        map = OccupancyGrid()
        map.header.frame_id = "map"
        map.header.stamp = self.get_clock().now().to_msg()

        map.info.resolution = resolution
        map.info.height = height_cells
        map.info.width = width_cells

        # Bottom-left corner of the grid, in the "map" frame -- centres the
        # grid on the map frame's own origin for any width/height/resolution,
        # matching the "publish /map at the centre point of the map" framing
        # world_to_map's translation above already establishes in world.
        map.info.origin.position.x = -(width_cells * resolution) / 2.0
        map.info.origin.position.y = -(height_cells * resolution) / 2.0
        map.info.origin.position.z = 0.0
        map.info.origin.orientation.w = 1.0

        map.data = [0] * (map.info.width * map.info.height)  # we update later

        self.occupancy_map_pub.publish(map)


def main():
    rclpy.init()
    try:
        rclpy.spin(MapPublisher())
    except Exception as e:
        print(f"{e}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()

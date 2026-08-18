
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
from hybraut_nav.qos import map_qos


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
                            'of the map. Baked directly into the published '
                            "OccupancyGrid's own info.origin (see "
                            'publish_occupancy_grid) rather than into a '
                            'non-identity world -> map transform -- see '
                            "world_to_map's own comment in "
                            'publish_global_frame for why: consumers that '
                            'read info.origin directly without walking TF '
                            '(e.g. strategy_node/strategy_node.py\'s own '
                            '_validate_goal cost-map-bounds check, which by '
                            "its own docstring skips frame_id "
                            'cross-checking) need it there, matching the '
                            "same pattern hybraut_nav's own "
                            'scripts/demo/fake_map_publisher.py already '
                            'uses (world x/y baked into info.origin, only '
                            'ever an identity transform between frames).',
                type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter(
            'map_origin_orientation', [0.0, 0.0, 0.0, 1.0],
            ParameterDescriptor(
                description='[x, y, z, w] quaternion for the world -> map '
                            'static transform rotation. Unlike '
                            'map_origin_position above, this still only '
                            'goes into the TF (no launch file in this repo '
                            'sets it non-identity, and OccupancyGrid.info.'
                            'origin.orientation is left identity to match).',
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
            qos_profile=map_qos
        )
        self.publish_global_frame()
        self.create_timer(
            1.0,
            self.publish_occupancy_grid,
            callback_group=ReentrantCallbackGroup()
        )

    def publish_global_frame(self):
        transforms = []

        origin_orientation = self.get_parameter('map_origin_orientation').value

        # Identity translation, not map_origin_position -- same reasoning as
        # world_to_odom below (map already coincides with world origin
        # position-wise): map_origin_position is instead baked directly into
        # the OccupancyGrid's own info.origin (publish_occupancy_grid), so
        # consumers that read info.origin without walking TF still see the
        # right world-frame bounds. See map_origin_position's own
        # declare_parameter description above for why that matters here.
        world_to_map = TransformStamped()
        world_to_map.header.stamp = self.get_clock().now().to_msg()
        world_to_map.header.frame_id = 'world'
        world_to_map.child_frame_id = 'map'
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

        # Bottom-left corner of the grid, in world-frame coordinates --
        # map_origin_position (the world-frame centre) offset by half the
        # grid extent each way. world_to_map above is identity, so this is
        # also the grid's position in the "map" frame; baking it in here
        # (rather than relying on a non-identity TF) keeps a consumer that
        # reads info.origin directly -- no TF lookup -- in agreement with
        # one that does walk TF. See map_origin_position's own
        # declare_parameter description for why that matters here.
        origin_position = self.get_parameter('map_origin_position').value
        map.info.origin.position.x = origin_position[0] - (width_cells * resolution) / 2.0
        map.info.origin.position.y = origin_position[1] - (height_cells * resolution) / 2.0
        map.info.origin.position.z = origin_position[2]
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

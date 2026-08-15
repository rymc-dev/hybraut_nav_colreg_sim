# mesh_viewer.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker

class MeshViewer(Node):
    def __init__(self):
        super().__init__('mesh_viewer')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)  # latched
        self.pub = self.create_publisher(Marker, 'mesh_marker', qos)
        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = "file:///home/you/model.obj"
        m.mesh_use_embedded_materials = True   # if a matching .mtl exists
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.a = 1.0
        self.pub.publish(m)

rclpy.init()
rclpy.spin(MeshViewer())
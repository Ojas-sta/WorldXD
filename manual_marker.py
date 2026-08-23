"""Manual end-effector jog via RViz interactive markers.

Publishes a draggable 3D target marker. While the user drags it, targets are
streamed to /ee_target (PointStamped) and the stacking controller follows them
in its MANUAL state. This keeps joint_states single-writer (the controller).

A context-menu entry toggles the gripper while in MANUAL mode.

Run: pixi run python3 manual_marker.py   (spawned by launch_robot.py)
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, Marker)


class ManualMarker(Node):
    def __init__(self):
        super().__init__('manual_marker')

        self.ee_pub = self.create_publisher(PointStamped, '/ee_target', 10)
        self.grip_pub = self.create_publisher(Bool, '/manual_gripper', 10)
        self.gripper_open = True

        self.server = InteractiveMarkerServer(self, 'ee_marker')
        self.menu = MenuHandler()
        self.menu.insert('Toggle gripper', callback=self.on_gripper_menu)

        self.make_marker(x=0.15, y=0.0, z=0.15)
        self.get_logger().info(
            'Manual marker ready: drag the sphere in RViz to jog the arm.')

    def make_marker(self, x, y, z):
        m = InteractiveMarker()
        m.header.frame_id = 'base_link'
        m.name = 'ee_jog'
        m.description = 'EE jog'
        m.scale = 0.12
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.035
        sphere.color.r = 0.4
        sphere.color.g = 0.82
        sphere.color.b = 1.0
        sphere.color.a = 0.9

        core = InteractiveMarkerControl()
        core.name = 'move_3d'
        core.always_visible = True
        core.interaction_mode = InteractiveMarkerControl.MOVE_3D
        core.markers.append(sphere)
        m.controls.append(core)

        for axis, quat in (('x', (1., 1., 0., 0.)),
                           ('y', (1., 0., 1., 0.)),
                           ('z', (1., 0., 0., 1.))):
            c = InteractiveMarkerControl()
            c.name = f'move_{axis}'
            c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z = quat
            m.controls.append(c)

        self.server.insert(m, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, 'ee_jog')
        self.server.applyChanges()

    def on_feedback(self, feedback):
        if feedback.event_type != feedback.POSE_UPDATE:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = feedback.pose.position.x
        msg.point.y = feedback.pose.position.y
        msg.point.z = feedback.pose.position.z
        self.ee_pub.publish(msg)

    def on_gripper_menu(self, feedback):
        self.gripper_open = not self.gripper_open
        msg = Bool()
        msg.data = self.gripper_open
        self.grip_pub.publish(msg)
        self.get_logger().info(f'Manual gripper -> {"open" if self.gripper_open else "closed"}')


def main(args=None):
    rclpy.init(args=args)
    node = ManualMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

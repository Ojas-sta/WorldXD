"""Manual end-effector jog + block dragging via RViz interactive markers.

EE jog: a draggable 3D target marker streams /ee_target (PointStamped); the
stacking controller follows it in MANUAL state, keeping joint_states
single-writer. Context menu toggles the gripper while in MANUAL mode.

Blocks: one draggable cube marker per simulated block. Drags stream
/block_move (PointStamped, frame_id = 'block_<id>') and workspace_env moves
that block. Markers re-sync to real block positions (~2Hz) so arm-carried
moves, releases and scene resets keep everything consistent; the marker being
dragged is exempt from syncing briefly so it doesn't fight the user's hand.

Run: pixi run python3 manual_marker.py   (spawned by launch_robot.py)
"""
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback,
    Marker, MarkerArray)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

BLOCK_COLORS = {0: (1.0, 0.15, 0.15),   # red
                1: (0.18, 1.0, 0.35),   # green
                2: (0.25, 0.55, 1.0),   # blue
                3: (1.0, 0.85, 0.2)}    # yellow


class ManualMarker(Node):
    def __init__(self):
        super().__init__('manual_marker')

        self.ee_pub = self.create_publisher(PointStamped, '/ee_target', 10)
        self.grip_pub = self.create_publisher(Bool, '/manual_gripper', 10)
        self.block_pub = self.create_publisher(PointStamped, '/block_move', 10)
        self.gripper_open = True

        # Which block marker is being dragged, when its last drag event was,
        # and our own record of marker poses (server has no query API we trust)
        self._dragging_block = None
        self._last_drag_time = 0.0
        self._block_marker_pos = {}

        self.server = InteractiveMarkerServer(self, 'ee_marker')
        self.menu = MenuHandler()
        self.menu.insert('Toggle gripper', callback=self.on_gripper_menu)

        self.make_ee_marker(x=0.15, y=0.0, z=0.15)
        for bid in range(4):
            self.make_block_marker(bid, x=0.15 + 0.05 * (bid % 2),
                                   y=0.10 if bid < 2 else -0.10, z=0.02)

        # Keep block markers glued to real block state (arm carries, resets...)
        self.create_subscription(MarkerArray, 'workspace_blocks',
                                 self.on_blocks_state, 10)
        self.server.applyChanges()
        self.get_logger().info(
            'Manual markers ready: drag the cyan sphere to jog the arm, '
            'drag colored cubes to move blocks.')

    # ------------------------------------------------------------------ EE jog
    def make_ee_marker(self, x, y, z):
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

        self.server.insert(m, feedback_callback=self.on_ee_feedback)
        self.menu.apply(self.server, 'ee_jog')

    def on_ee_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
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

    # ------------------------------------------------------------- block drags
    def make_block_marker(self, bid, x, y, z):
        m = InteractiveMarker()
        m.header.frame_id = 'base_link'
        m.name = f'jog_block_{bid}'
        m.description = f'block {bid}'
        m.scale = 0.08
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = cube.scale.y = cube.scale.z = 0.04
        cube.color.r, cube.color.g, cube.color.b = BLOCK_COLORS[bid]
        cube.color.a = 0.95

        core = InteractiveMarkerControl()
        core.name = f'block_{bid}_move'
        core.always_visible = True
        core.interaction_mode = InteractiveMarkerControl.MOVE_3D
        core.markers.append(cube)
        m.controls.append(core)

        self.server.insert(m, feedback_callback=self.make_block_cb(bid))
        self._block_marker_pos[f'jog_block_{bid}'] = (x, y, z)

    def make_block_cb(self, bid):
        def cb(feedback):
            if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
                return
            self._dragging_block = bid
            self._last_drag_time = time.monotonic()
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'block_{bid}'
            msg.point.x = feedback.pose.position.x
            msg.point.y = max(feedback.pose.position.y, -0.35)
            msg.point.z = max(feedback.pose.position.z, 0.02)  # stay on the table
            self.block_pub.publish(msg)
            self._block_marker_pos[f'jog_block_{bid}'] = (
                msg.point.x, msg.point.y, msg.point.z)
        return cb

    def on_blocks_state(self, msg: MarkerArray):
        """Re-pose block markers from authoritative state, except while dragging."""
        fresh_drag = time.monotonic() - self._last_drag_time < 1.5
        changed = False
        for mkr in msg.markers:
            name = f'jog_block_{mkr.id}'
            if fresh_drag and self._dragging_block == mkr.id:
                continue
            px, py, pz = self._block_marker_pos.get(name, (None, None, None))
            if (px is None or abs(px - mkr.pose.position.x) > 0.005 or
                    abs(py - mkr.pose.position.y) > 0.005 or
                    abs(pz - mkr.pose.position.z) > 0.005):
                self.server.setPose(name, mkr.pose)
                self._block_marker_pos[name] = (
                    mkr.pose.position.x, mkr.pose.position.y, mkr.pose.position.z)
                changed = True
        if changed:
            self.server.applyChanges()


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

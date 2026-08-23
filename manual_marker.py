"""Manual end-effector jog + block dragging via RViz interactive markers.

EE jog: a draggable 3D target marker streams /ee_target (PointStamped); the
stacking controller follows it in MANUAL state, keeping joint_states
single-writer. Context menu toggles the gripper while in MANUAL mode.

Blocks: one draggable cube marker per simulated block. Drags stream
/block_move (PointStamped, frame_id = 'block_<id>') and workspace_env moves
that block. Markers re-sync to real block positions so arm-carried moves,
releases and scene resets keep everything consistent; the marker being dragged
is exempt from syncing briefly so it doesn't fight the user's hand.

P3.3 GUARDRAILS (driven by /fsm_state from stacking_controller):
  While a JEPA/state-machine task is running (any FSM state other than DONE or
  MANUAL), manual control is locked out:
    - EE jog drags are ignored by the controller AND the marker loses its
      axis arrows + becomes non-draggable (sphere stays visible)
    - Block markers become non-draggable and drags are ignored
  Guardrails lift automatically when the task finishes.

Run: pixi run python3 manual_marker.py   (spawned by launch_robot.py)
"""
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback,
    Marker, MarkerArray)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

BLOCK_COLORS = {0: (1.0, 0.15, 0.15),   # red
                1: (0.18, 1.0, 0.35),   # green
                2: (0.25, 0.55, 1.0),   # blue
                3: (1.0, 0.85, 0.2)}    # yellow

# FSM states that allow manual control
FREE_STATES = {'DONE', 'MANUAL'}
NUM_BLOCKS = 4


class ManualMarker(Node):
    def __init__(self):
        super().__init__('manual_marker')

        self.ee_pub = self.create_publisher(PointStamped, '/ee_target', 10)
        self.grip_pub = self.create_publisher(Bool, '/manual_gripper', 10)
        self.block_pub = self.create_publisher(PointStamped, '/block_move', 10)
        self.gripper_open = True

        # Which block marker is being dragged, when its last drag event was,
        # and our own record of marker poses
        self._dragging_block = None
        self._last_drag_time = 0.0
        self._block_marker_pos = {}

        # P3.3 guardrail state; authoritative source is /fsm_state
        self.task_busy = False

        self.server = InteractiveMarkerServer(self, 'ee_marker')
        self.menu = MenuHandler()
        self.menu.insert('Toggle gripper', callback=self.on_gripper_menu)

        self.create_subscription(String, 'fsm_state', self.on_fsm, 10)

        self._rebuild_all()
        self.create_subscription(MarkerArray, 'workspace_blocks',
                                 self.on_blocks_state, 10)
        self.get_logger().info(
            'Manual markers ready: drag the cyan sphere to jog the arm, '
            'drag colored cubes to move blocks. '
            'Guardrails lock manual control while tasks run.')

    # ------------------------------------------------------------- guardrails
    def on_fsm(self, msg):
        busy = msg.data not in FREE_STATES
        if busy != self.task_busy:
            self.task_busy = busy
            self._rebuild_all()
            self.get_logger().info(
                f'Guardrails {"ENGAGED (task running)" if busy else "released"}'
                f' [fsm={msg.data}]')

    def _rebuild_all(self):
        """Re-create all markers with/without dragging enabled."""
        self._build_ee_marker()
        for bid in range(NUM_BLOCKS):
            x, y, z = self._block_marker_pos.get(f'jog_block_{bid}',
                                                 (0.15 + 0.05 * (bid % 2),
                                                  0.10 if bid < 2 else -0.10,
                                                  0.02))
            self._build_block_marker(bid, x, y, z)
        self.menu.apply(self.server, 'ee_jog')
        self.server.applyChanges()

    # ------------------------------------------------------------------ EE jog
    def _build_ee_marker(self):
        m = InteractiveMarker()
        m.header.frame_id = 'base_link'
        m.name = 'ee_jog'
        m.description = 'EE jog' if not self.task_busy else 'EE jog (locked)'
        m.scale = 0.12
        px, py, pz = self._block_marker_pos.get('ee_jog', (0.15, 0.0, 0.15))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, pz

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.035
        sphere.color.r = 0.4
        sphere.color.g = 0.82
        sphere.color.b = 1.0
        sphere.color.a = 0.9 if not self.task_busy else 0.4  # dimmed while locked

        core = InteractiveMarkerControl()
        core.name = 'move_3d'
        core.always_visible = True
        core.interaction_mode = (InteractiveMarkerControl.MOVE_3D
                                 if not self.task_busy else
                                 InteractiveMarkerControl.NONE)
        core.markers.append(sphere)
        m.controls.append(core)

        if not self.task_busy:
            for axis, quat in (('x', (1., 1., 0., 0.)),
                               ('y', (1., 0., 1., 0.)),
                               ('z', (1., 0., 0., 1.))):
                c = InteractiveMarkerControl()
                c.name = f'move_{axis}'
                c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z = quat
                m.controls.append(c)

        self.server.insert(m, feedback_callback=self.on_ee_feedback)

    def on_ee_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        self._block_marker_pos['ee_jog'] = (
            feedback.pose.position.x, feedback.pose.position.y,
            feedback.pose.position.z)
        if self.task_busy:
            # Belt-and-suspenders: even a stale RViz drag must not move the arm
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = feedback.pose.position.x
        msg.point.y = feedback.pose.position.y
        msg.point.z = feedback.pose.position.z
        self.ee_pub.publish(msg)

    def on_gripper_menu(self, feedback):
        if self.task_busy:
            return
        self.gripper_open = not self.gripper_open
        msg = Bool()
        msg.data = self.gripper_open
        self.grip_pub.publish(msg)
        self.get_logger().info(f'Manual gripper -> {"open" if self.gripper_open else "closed"}')

    # ------------------------------------------------------------- block drags
    def _build_block_marker(self, bid, x, y, z):
        m = InteractiveMarker()
        m.header.frame_id = 'base_link'
        m.name = f'jog_block_{bid}'
        m.description = f'block {bid}' if not self.task_busy else ''
        m.scale = 0.08
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = cube.scale.y = cube.scale.z = 0.04
        cube.color.r, cube.color.g, cube.color.b = BLOCK_COLORS[bid]
        cube.color.a = 0.95 if not self.task_busy else 0.5

        core = InteractiveMarkerControl()
        core.name = f'block_{bid}_move'
        core.always_visible = True
        core.interaction_mode = (InteractiveMarkerControl.MOVE_3D
                                 if not self.task_busy else
                                 InteractiveMarkerControl.NONE)
        core.markers.append(cube)
        m.controls.append(core)

        self.server.insert(m, feedback_callback=self.make_block_cb(bid))
        self._block_marker_pos[f'jog_block_{bid}'] = (x, y, z)

    def make_block_cb(self, bid):
        def cb(feedback):
            if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
                return
            if self.task_busy:
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
                pose = mkr.pose
                self.server.setPose(name, pose)
                self._block_marker_pos[name] = (
                    pose.position.x, pose.position.y, pose.position.z)
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

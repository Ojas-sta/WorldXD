"""ROS2 -> Express dashboard relay.

Subscribes to live robot topics and forwards them as JSON to the Node backend:
  /joint_states      -> joint angles (rad + deg)
  /gripper_closed    -> gripper bool
  /fsm_state         -> current state-machine state
  /jepa_telemetry    -> real CEM inference latency (ms)
  /workspace_blocks  -> block positions/colors
  /camera/image_raw  -> base64 JPEG, throttled (dashboard feed)

Run: pixi run python3 dashboard/backend/ros_bridge.py
Requires the Express server on DASHBOARD_URL (default http://localhost:4002).
"""
import base64
import json
import os
import threading
import urllib.request

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Bool, String, Float32
from visualization_msgs.msg import MarkerArray, Marker

DASHBOARD_URL = os.environ.get('DASHBOARD_URL', 'http://localhost:4002')
CAMERA_FPS = 8.0

COLOR_NAMES = {0: 'Red', 1: 'Green', 2: 'Blue', 3: 'Yellow'}


def post(path, payload):
    """Fire-and-forget POST; never raises into ROS callbacks."""
    try:
        req = urllib.request.Request(
            f'{DASHBOARD_URL}{path}',
            data=json.dumps(payload).encode(),
            headers={'Content-Type': 'application/json'},
            method='POST')
        urllib.request.urlopen(req, timeout=1.0)
    except Exception:
        pass


class RosBridge(Node):
    def __init__(self):
        super().__init__('ros_dashboard_bridge')
        self.bridge = CvBridge()
        self._last_cam_send = 0.0
        self._lock = threading.Lock()

        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Bool, '/gripper_closed', self.on_gripper, 10)
        self.create_subscription(String, 'fsm_state', self.on_fsm, 10)
        self.create_subscription(Float32, 'jepa_telemetry', self.on_jepa, 10)
        self.create_subscription(MarkerArray, 'workspace_blocks', self.on_blocks, 10)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.on_camera, qos_sensor)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info, qos_sensor)

        # Latest state snapshot merged into one payload per event.
        # fsm_stamp lets the server reject out-of-order snapshots (joints stream
        # at 30Hz and HTTP posts can arrive reordered).
        self.state = {
            'fsmState': 'DONE',
            'fsmStamp': 0,
            'jointAngles': [0.0, 0.0, 0.0, 0.0],
            'gripperClosed': False,
            'lastInferenceMs': None,
            'blocks': [],
            'imageRes': None,
        }
        self.get_logger().info(f'ROS bridge ready -> {DASHBOARD_URL}')

    def _push(self):
        with self._lock:
            snapshot = dict(self.state)
        threading.Thread(target=post, args=('/ros/state', snapshot), daemon=True).start()

    def on_joints(self, msg):
        if len(msg.position) >= 4:
            with self._lock:
                self.state['jointAngles'] = list(msg.position[:4])
            self._push()

    def on_gripper(self, msg):
        with self._lock:
            self.state['gripperClosed'] = bool(msg.data)
        self._push()

    def on_fsm(self, msg):
        with self._lock:
            self.state['fsmState'] = msg.data
            self.state['fsmStamp'] = self.get_clock().now().nanoseconds
        self._push()

    def on_jepa(self, msg):
        with self._lock:
            self.state['lastInferenceMs'] = round(float(msg.data), 1)
        self._push()

    def on_blocks(self, msg):
        blocks = []
        for m in msg.markers:
            if m.type == Marker.CUBE:
                rgb = [int(c * 255) for c in (m.color.r, m.color.g, m.color.b)]
                blocks.append({
                    'id': m.id,
                    'color': COLOR_NAMES.get(m.id, f'#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}'),
                    'pos': [round(m.pose.position.x, 3),
                            round(m.pose.position.y, 3),
                            round(m.pose.position.z, 3)],
                })
        blocks.sort(key=lambda b: b['id'])
        with self._lock:
            self.state['blocks'] = blocks
        self._push()

    def on_camera_info(self, msg):
        with self._lock:
            self.state['imageRes'] = [msg.width, msg.height]

    def on_camera(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_cam_send < 1.0 / CAMERA_FPS:
            return
        self._last_cam_send = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                return
            b64 = base64.b64encode(buf.tobytes()).decode()
            threading.Thread(
                target=post, args=('/ros/camera', {'jpeg': b64}), daemon=True).start()
        except Exception as e:
            self.get_logger().warn(f'camera relay error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = RosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

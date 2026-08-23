import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class WorkspaceEnv(Node):
    def __init__(self):
        super().__init__('workspace_env')
        
        self.declare_parameter('use_aruco', True)
        self.use_aruco = self.get_parameter('use_aruco').value
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'workspace_blocks', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Subscriber
        self.gripper_sub = self.create_subscription(Bool, '/gripper_closed', self.gripper_callback, 10)
        from std_msgs.msg import String
        from geometry_msgs.msg import PointStamped
        self.prompt_sub = self.create_subscription(String, 'user_prompt', self.prompt_callback, 10)
        # Manual block drags from the RViz interactive markers (manual_marker.py)
        self.block_move_sub = self.create_subscription(
            PointStamped, '/block_move', self.block_move_callback, 10)
        # P3.3 guardrail: track task activity; blocks are fixed while a task runs
        from std_msgs.msg import Float32 as _F32
        self.task_busy = False
        self.fsm_sub = self.create_subscription(String, 'fsm_state', self.fsm_callback, 10)
        
        self.gripper_closed = False
        self.grabbed_block = None
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.bridge = CvBridge()
        
        # Define initial blocks
        self.initial_blocks = [
            {'id': 0, 'color': (1.0, 0.0, 0.0), 'pos': [0.15, 0.1, 0.02]},
            {'id': 1, 'color': (0.0, 1.0, 0.0), 'pos': [0.20, 0.1, 0.02]},
            {'id': 2, 'color': (0.0, 0.0, 1.0), 'pos': [0.15, -0.1, 0.02]},
            {'id': 3, 'color': (1.0, 1.0, 0.0), 'pos': [0.20, -0.1, 0.02]}
        ]
        import copy
        self.blocks = copy.deepcopy(self.initial_blocks)

        # P3.5 physics state: per-block fall velocity + last manual-drag time.
        # A block that was hand-dragged stays kinematic briefly after its last
        # /block_move message, then gravity takes over (P3.5).
        import time as _time
        self._vel = {b['id']: 0.0 for b in self.blocks}
        self._last_move = {b['id']: 0.0 for b in self.blocks}
        self._last_physics = _time.monotonic()
        self.BLOCK_HALF = 0.02
        self.TABLE_Z = 0.0      # table SURFACE height; block centers rest at 0.02
        self.GRAVITY = 2.5      # m/s^2, scaled down so falls read at 30Hz
        self.DRAG_GRACE = 0.35  # s of kinematic hold after a drag message
        # P3.6: support rule + arm collision
        self.MIN_SUPPORT_FRAC = 0.5   # <50% footprint supported => tumble off
        self.TUMBLE_SLIDE = 0.04      # m/s lateral slide while unsupported
        self.GRIPPER_RADIUS = 0.055   # zone around manipulator_link exempt from
                                      # collision: touching there = pickup attempt
        self.ARM_MARGIN = 0.008       # link-vs-block clearance

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # Pre-render markers at a fixed high resolution once; resize per frame.
        # Generating markers at tiny/odd sizes fails intermittently -> visual glitches.
        self._aruco_cache = {}
        for i in range(4):
            m = cv2.aruco.generateImageMarker(self.aruco_dict, i, 128)
            self._aruco_cache[i] = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info(f"Workspace Environment Initialized.")

    def fsm_callback(self, msg):
        self.task_busy = msg.data not in ('DONE', 'MANUAL')

    def block_move_callback(self, msg):
        """Drag updates from manual_marker: relocate the named block.

        Refuses while the arm carries that block (position is owned by
        the gripper TF until released) and during running tasks (P3.3).
        """
        if self.task_busy:
            self.get_logger().info(
                'Block move refused: task in progress.', throttle_duration_sec=5.0)
            return
        try:
            bid = int(msg.header.frame_id.replace('block_', ''))
        except ValueError:
            return
        if self.grabbed_block == bid:
            return
        self.blocks[bid]['pos'] = [
            max(min(msg.point.x, 0.35), 0.05),
            max(min(msg.point.y, 0.30), -0.30),
            max(msg.point.z, 0.02),
        ]
        import time as _time
        self._last_move[bid] = _time.monotonic()   # hold kinematic during drag (P3.5)

    def prompt_callback(self, msg):
        import copy
        text = msg.data.lower()
        if 'reset' in text or 'down' in text or 'separate' in text:
            self.blocks = copy.deepcopy(self.initial_blocks)
            self._vel = {b['id']: 0.0 for b in self.blocks}
            self._last_move = {b['id']: 0.0 for b in self.blocks}
            self.grabbed_block = None
            self.get_logger().info("Physically reset blocks to initial positions.")

    def gripper_callback(self, msg):
        was_closed = self.gripper_closed
        self.gripper_closed = msg.data
        
        if self.gripper_closed and not was_closed:
            # Gripper just closed. Grab the NEAREST block within grasp radius.
            # Blocks sit 5cm apart, so a 6cm "first-match" check could grab a neighbor.
            try:
                tf = self.tf_buffer.lookup_transform('base_link', 'manipulator_link', rclpy.time.Time())
                mx, my, mz = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
                best_id, best_dist = None, 0.04
                for block in self.blocks:
                    bx, by, bz = block['pos'][0], block['pos'][1], block['pos'][2]
                    dist = math.sqrt((mx - bx)**2 + (my - by)**2 + (mz - bz)**2)
                    if dist < best_dist:
                        best_id, best_dist = block['id'], dist
                if best_id is not None:
                    self.grabbed_block = best_id
                    self.get_logger().info(f"Grabbed block {self.grabbed_block}")
            except Exception as e:
                self.get_logger().warn(f"Grasp check failed: {e}", throttle_duration_sec=5.0)
        elif not self.gripper_closed and was_closed:
            # Gripper opened, release block
            if self.grabbed_block is not None:
                self.get_logger().info(f"Released block {self.grabbed_block}")
            self.grabbed_block = None

    # ------------------------------------------------------------- P3.5 physics
    def _support_for(self, block):
        """Best combined support under a block.

        Supporters whose tops sit within tolerance of the highest contact are
        GROUPED (P3.6.1): a block straddling the diagonal corner of two blocks
        has 25% on each — individually unstable, but 50% combined = stable.
        Returns (surface_z, supported_fraction, slide_dir):
        - surface_z: common height of the highest contact group
        - supported_fraction: summed footprint overlap vs block area
        - slide_dir: unit vector away from the support centroid when the
          fraction is below MIN_SUPPORT_FRAC
        """
        H = 2 * self.BLOCK_HALF
        bx, by, bz = block['pos']
        candidates = []  # (top_z, frac, ox, oy)
        for other in self.blocks:
            if other is block:
                continue
            ox, oy, oz = other['pos']
            dx, dy = bx - ox, by - oy
            fx = H - abs(dx)
            fy = H - abs(dy)
            if fx <= 0 or fy <= 0:
                continue
            o_top = oz + self.BLOCK_HALF
            if o_top > bz + 0.005:
                continue
            candidates.append((o_top, (fx * fy) / (H * H), ox, oy))
        if not candidates:
            return (self.TABLE_Z, 1.0, None)

        top = max(c[0] for c in candidates)
        group = [c for c in candidates if abs(c[0] - top) <= 0.003]
        total = min(1.0, sum(c[1] for c in group))

        if total < self.MIN_SUPPORT_FRAC:
            # P3.6.2 fix (B1): guarantee a usable slide direction. A symmetric
            # straddle can place the centroid exactly under our center
            # (n ~= 0), which previously yielded slide=None and let an
            # under-supported block float forever.
            wsum = sum(c[1] for c in group)
            cx = sum(c[1] * c[2] for c in group) / wsum
            cy = sum(c[1] * c[3] for c in group) / wsum
            dx, dy = bx - cx, by - cy
            n = (dx * dx + dy * dy) ** 0.5
            if n > 1e-6:
                slide = (dx / n, dy / n)
            else:
                # fall back to direction away from nearest supporter edge;
                # final fallback: +x so the block always resolves
                nearest = min(group,
                              key=lambda c: (c[2] - bx) ** 2 + (c[3] - by) ** 2)
                dx, dy = bx - nearest[2], by - nearest[3]
                n = (dx * dx + dy * dy) ** 0.5
                slide = ((dx / n, dy / n) if n > 1e-6 else (1.0, 0.0))
            return (top, total, slide)
        return (top, total, None)

    def physics_step(self):
        """Gravity + stacking + tumble: free blocks fall until they rest on a
        surface that supports at least half their footprint; less than that
        and they slide off the edge before dropping."""
        import time as _time
        now = _time.monotonic()
        dt = min(now - self._last_physics, 0.1)
        self._last_physics = now

        for b in self.blocks:
            bid = b['id']
            if self.grabbed_block == bid:
                self._vel[bid] = 0.0
                continue
            if now - self._last_move.get(bid, 0.0) < self.DRAG_GRACE:
                continue  # held by a live hand-drag: kinematic
            sup_z, frac, slide = self._support_for(b)
            rest_z = sup_z + self.BLOCK_HALF
            z = b['pos'][2]
            if frac < self.MIN_SUPPORT_FRAC:
                # P3.6.2: genuinely under-supported -> TIP OFF, don't surf.
                # Slide out while gravity pulls down, so the block drops into
                # the gap (or onto a lower surface) instead of skating across
                # supporter tops onto the neighbouring block.
                self._vel[bid] += self.GRAVITY * dt
                b['pos'][2] = max(self.TABLE_Z + self.BLOCK_HALF,
                                  z - self._vel[bid] * dt)
                if slide is not None:
                    b['pos'][0] += slide[0] * self.TUMBLE_SLIDE * dt
                    b['pos'][1] += slide[1] * self.TUMBLE_SLIDE * dt
            elif z > rest_z + 0.001:
                # airborne above a valid support: accelerate downward
                self._vel[bid] += self.GRAVITY * dt
                b['pos'][2] = max(rest_z, z - self._vel[bid] * dt)
            elif z < rest_z - 0.001:
                # intersecting the support column (dragged into it): push up
                b['pos'][2] = rest_z
                self._vel[bid] = 0.0
            else:
                b['pos'][2] = rest_z
                self._vel[bid] = 0.0

    # ------------------------------------------------- P3.6 arm-block collision
    def _arm_segments(self):
        """Sampled points along the arm links (base -> tip), from TF."""
        chain = ['shoulder_link', 'lower_arm_link', 'upper_arm_link',
                 'manipulator_link']
        pts = []
        prev = (0.0, 0.0, 0.0)  # base_link origin
        for frame in chain:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', frame, rclpy.time.Time())
            except Exception:
                return None
            p = (tf.transform.translation.x,
                 tf.transform.translation.y,
                 tf.transform.translation.z)
            pts.append((prev, p))
            prev = p
        return pts, prev

    def arm_collision_step(self):
        """Shove blocks aside when a non-gripper link sweeps through them.

        The gripper zone around manipulator_link is exempt: contact there is
        the pickup mechanism (proximity grab), not a collision.
        """
        result = self._arm_segments()
        if result is None:
            return
        segments, tip = result
        H = self.BLOCK_HALF + self.ARM_MARGIN

        for seg_a, seg_b in segments:
            for k in range(7):
                t = k / 6.0
                px = seg_a[0] + (seg_b[0] - seg_a[0]) * t
                py = seg_a[1] + (seg_b[1] - seg_a[1]) * t
                pz = seg_a[2] + (seg_b[2] - seg_a[2]) * t
                # gripper-zone exemption
                d_tip_sq = ((px - tip[0]) ** 2 + (py - tip[1]) ** 2 +
                            (pz - tip[2]) ** 2)
                if d_tip_sq < self.GRIPPER_RADIUS ** 2:
                    continue
                for b in self.blocks:
                    if self.grabbed_block == b['id']:
                        continue
                    bx, by, bz = b['pos']
                    # closest point on the block AABB to the sample point
                    cx = max(bx - H, min(px, bx + H))
                    cy = max(by - H, min(py, by + H))
                    cz = max(bz - H, min(pz, bz + H))
                    dx, dy, dz = px - cx, py - cy, pz - cz
                    dist_sq = dx * dx + dy * dy + dz * dz
                    if dist_sq >= self.ARM_MARGIN ** 2:
                        continue
                    # push horizontally away from the link sample point
                    n_xy = (bx - px, by - py)
                    norm = (n_xy[0] ** 2 + n_xy[1] ** 2) ** 0.5
                    if norm < 1e-6:
                        n_xy = (bx, by)
                        norm = (bx * bx + by * by) ** 0.5 or 1.0
                    push = min(0.012, (self.ARM_MARGIN - dist_sq ** 0.5) + 0.002)
                    b['pos'][0] += n_xy[0] / norm * push
                    b['pos'][1] += n_xy[1] / norm * push
                    self.get_logger().info(
                        f"Arm nudged block {b['id']} aside",
                        throttle_duration_sec=3.0)

    def timer_callback(self):
        # Update grabbed block position
        if self.grabbed_block is not None:
            try:
                tf = self.tf_buffer.lookup_transform('base_link', 'manipulator_link', rclpy.time.Time())
                self.blocks[self.grabbed_block]['pos'] = [
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z - 0.02 # hang slightly below
                ]
            except Exception as e:
                self.get_logger().warn(f"Grabbed-block TF lookup failed: {e}", throttle_duration_sec=5.0)

        self.physics_step()
        self.arm_collision_step()
        self.publish_markers_and_tf()
        self.render_synthetic_camera()

    def publish_markers_and_tf(self):
        marker_array = MarkerArray()
        
        # Broadcast camera_link pointing DOWN from manipulator_link
        t_cam = TransformStamped()
        t_cam.header.stamp = self.get_clock().now().to_msg()
        t_cam.header.frame_id = 'manipulator_link'
        t_cam.child_frame_id = 'camera_link'
        t_cam.transform.translation.x = 0.05  # slightly forward
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = -0.02 # slightly below
        # Rotate pitch by 90 degrees down (so Z points down instead of X forward)
        # Quat for pitch = 90 deg (1.5708 rad): qy = sin(45) = 0.707, qw = cos(45) = 0.707
        t_cam.transform.rotation.x = 0.0
        t_cam.transform.rotation.y = 0.7071068
        t_cam.transform.rotation.z = 0.0
        t_cam.transform.rotation.w = 0.7071068
        self.tf_broadcaster.sendTransform(t_cam)
        
        for idx, block in enumerate(self.blocks):
            # Publish TF
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = f'block_{idx}'
            t.transform.translation.x = float(block['pos'][0])
            t.transform.translation.y = float(block['pos'][1])
            t.transform.translation.z = float(block['pos'][2])
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            
            # Publish Marker
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "blocks"
            m.id = block['id']
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(block['pos'][0])
            m.pose.position.y = float(block['pos'][1])
            m.pose.position.z = float(block['pos'][2])
            m.scale.x = 0.04
            m.scale.y = 0.04
            m.scale.z = 0.04
            m.color.r = block['color'][0]
            m.color.g = block['color'][1]
            m.color.b = block['color'][2]
            m.color.a = 1.0
            marker_array.markers.append(m)
            
        self.marker_pub.publish(marker_array)

    def render_synthetic_camera(self):
        try:
            img = np.ones((224, 224, 3), dtype=np.uint8) * 50
            focal_length = 200.0
            cx, cy = 112.0, 112.0
            
            for block in self.blocks:
                try:
                    # Look up from camera_link to block
                    tf = self.tf_buffer.lookup_transform('camera_link', f'block_{block["id"]}', rclpy.time.Time())
                    x = tf.transform.translation.x
                    y = tf.transform.translation.y
                    z = tf.transform.translation.z
                    
                    # Camera is looking along +Z of camera_link
                    if z < 0.01: continue # Behind or too close to camera
                    
                    u = int(cx + focal_length * (x / z)) # map X to U (horizontal right)
                    v = int(cy + focal_length * (y / z)) # map Y to V (vertical down)
                    
                    size = int(focal_length * (0.04 / z))
                    size = max(4, min(size, 224))  # clamp: avoid degenerate sizes near camera
                    if size > 0:
                        color = (int(block['color'][2]*255), int(block['color'][1]*255), int(block['color'][0]*255))
                        
                        # Calculate bounds for cropping
                        x1 = max(0, u - size//2)
                        y1 = max(0, v - size//2)
                        x2 = min(224, u + size//2)
                        y2 = min(224, v + size//2)
                        
                        if x1 < x2 and y1 < y2:
                            cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
                            
                            if self.use_aruco:
                                marker_img = cv2.resize(self._aruco_cache[block['id']], (size, size))
                                
                                # Crop marker image to fit within bounds
                                mx1 = x1 - (u - size//2)
                                my1 = y1 - (v - size//2)
                                mx2 = mx1 + (x2 - x1)
                                my2 = my1 + (y2 - y1)
                                
                                marker_crop = marker_img[my1:my2, mx1:mx2]
                                roi = img[y1:y2, x1:x2]
                                
                                if roi.shape == marker_crop.shape and roi.size > 0:
                                    blended = cv2.addWeighted(roi, 0.3, marker_crop, 0.7, 0)
                                    img[y1:y2, x1:x2] = blended
                except Exception as e:
                    self.get_logger().warn(f"Render error for block {block['id']}: {e}", throttle_duration_sec=5.0)
            
            # Publish Image
            # Backdate stamp slightly: robot_state_publisher TF (driven by joint_states)
            # can be stamped marginally ahead of this node's clock, making RViz's
            # message filter drop frames ("timestamp earlier than transform cache").
            now = self.get_clock().now().to_msg()
            total_ns = now.sec * 1_000_000_000 + now.nanosec - 50_000_000
            stamp = Time()
            stamp.sec = total_ns // 1_000_000_000
            stamp.nanosec = total_ns % 1_000_000_000
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = "camera_link"
            self.image_pub.publish(img_msg)
            
            # Publish CameraInfo
            cam_info = CameraInfo()
            cam_info.header = img_msg.header
            cam_info.height = 224
            cam_info.width = 224
            cam_info.distortion_model = "plumb_bob"
            cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            # Intrinsic camera matrix
            cam_info.k = [focal_length, 0.0, cx,
                          0.0, focal_length, cy,
                          0.0, 0.0, 1.0]
            # Rectification matrix
            cam_info.r = [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
            # Projection matrix
            cam_info.p = [focal_length, 0.0, cx, 0.0,
                          0.0, focal_length, cy, 0.0,
                          0.0, 0.0, 1.0, 0.0]
            self.camera_info_pub.publish(cam_info)
            
        except Exception as e:
            self.get_logger().warn(f"Camera render error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceEnv()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

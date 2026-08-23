import rclpy
from rclpy.node import Node
import math
import re
import threading
import numpy as np
from cv_bridge import CvBridge

import torch
import torchvision.transforms as transforms
from jepa_model import JEPAWorldModel

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

COLOR_TO_ID = {'red': 0, 'green': 1, 'blue': 2, 'yellow': 3}
BLOCK_HOME = {0: [0.15, 0.1, 0.02], 1: [0.20, 0.1, 0.02],
              2: [0.15, -0.1, 0.02], 3: [0.20, -0.1, 0.02]}


class StackingController(Node):
    def __init__(self):
        super().__init__('stacking_controller')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper_closed', 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.prompt_sub = self.create_subscription(String, 'user_prompt', self.prompt_callback, 10)

        self.bridge = CvBridge()

        # JEPAWorldModel manages its own MPS device; do NOT call .to(device) here.
        # Reduced CEM size: full-size planning is far too slow for a live feed.
        self.model = JEPAWorldModel(device='mps', num_samples=64, iterations=2)
        self.model.eval()
        self.device = self.model.device

        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        dummy_goal = np.zeros((224, 224, 3), dtype=np.uint8)
        self.goal_tensor = self.transform(dummy_goal).unsqueeze(0).to(self.device)

        # Background JEPA worker: inference takes seconds and would otherwise starve
        # the single-threaded executor, freezing control_loop ("minute movement" bug).
        self._latest_frame = None
        self._frame_event = threading.Event()
        self.latest_action = None
        threading.Thread(target=self._jepa_worker, daemon=True).start()

        # Kinematics
        self.l1 = 0.134
        self.l2 = 0.120
        self.base_z = 0.078
        self.current_angles = [0.0, 0.0, 0.0, 0.0]
        self.max_velocity = 41.8879  # 400 rpm limit
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # End effector
        self.current_ee = [0.15, 0.0, 0.15]
        self.gripper_open = True

        # State machine
        self.state = 'DONE'
        self.state_ticks = 0
        self.pick_block_id = None
        self.place_block_id = None   # None => fixed stack target
        self.queue = []              # pending tasks for "arrange all"
        self.stack_target = [0.25, 0.0, 0.02]

        # Motion limits
        self.ee_speed = 0.06         # m/s
        self.pos_tol = 0.006         # m arrival tolerance
        self.settle_ticks = 18       # ~0.6s gripper settle pause

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)
        self.get_logger().info("Stacking Controller Initialized.")

    # ------------------------------------------------------------------ prompts
    def _block_pos(self, block_id):
        try:
            tf = self.tf_buffer.lookup_transform('base_link', f'block_{block_id}', rclpy.time.Time())
            return [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
        except Exception:
            return BLOCK_HOME.get(block_id, [0.2, 0.0, 0.02])

    def _start_task(self, pick_id, place_id):
        self.pick_block_id = pick_id
        self.place_block_id = place_id
        self.state = 'MOVE_ABOVE_BLOCK'
        self.state_ticks = 0
        src = self._block_pos(pick_id)
        dest = 'stack' if place_id is None else f'block {place_id}'
        self.get_logger().info(
            f"Task: pick block {pick_id} at {[round(v, 3) for v in src]} -> {dest}")

    def prompt_callback(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f"Received prompt: {text}")

        if 'reset' in text or 'down' in text or 'separate' in text:
            self.queue = []
            self.state = 'DONE'
            self.get_logger().info("Resetting.")
            return

        if 'arrange' in text or ('all' in text and 'on top of' not in text):
            self.queue = [(i, None) for i in range(4)]
            pick, place = self.queue.pop(0)
            self._start_task(pick, place)
            return

        pick = place = None
        m_pick = re.search(r'(?:pick(?:\s+up)?|grab|move)\s+(?:the\s+)?(red|green|blue|yellow)', text)
        if m_pick:
            pick = COLOR_TO_ID[m_pick.group(1)]
        else:
            for name, idx in COLOR_TO_ID.items():
                if name in text:
                    pick = idx
                    break

        m_place = re.search(r'on top of\s+(?:the\s+)?(red|green|blue|yellow)', text)
        if m_place:
            place = COLOR_TO_ID[m_place.group(1)]
            if pick is not None and place == pick:
                self.get_logger().info("Cannot stack a block onto itself.")
                return

        if pick is None:
            self.get_logger().info(
                "Prompt not recognized. Example: 'pick up the green block and place it "
                "on top of the yellow block'.")
            return

        self.queue = []
        self._start_task(pick, place)

    # ------------------------------------------------------------- JEPA worker
    def image_callback(self, msg):
        try:
            self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._frame_event.set()
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")

    def _jepa_worker(self):
        while True:
            self._frame_event.wait()
            self._frame_event.clear()
            frame = self._latest_frame
            if frame is None:
                continue
            try:
                img_tensor = self.transform(frame).unsqueeze(0).to(self.device)
                with torch.no_grad():
                    action = self.model.get_action(img_tensor, self.goal_tensor)
                self.latest_action = action
            except Exception as e:
                self.get_logger().warn(f"JEPA inference error: {e}")
            finally:
                self._latest_frame = None

    # ------------------------------------------------------------------- IK
    def solve_ik(self, x, y, z):
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        z_rel = z - self.base_z
        d_sq = r**2 + z_rel**2
        d = math.sqrt(d_sq)

        if d > (self.l1 + self.l2):
            scale = (self.l1 + self.l2) / d * 0.99
            r *= scale
            z_rel *= scale
            d_sq = r**2 + z_rel**2
            d = math.sqrt(d_sq)

        cos_theta3 = (d_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta3 = max(min(cos_theta3, 1.0), -1.0)
        theta3_math = math.atan2(-math.sqrt(1 - cos_theta3**2), cos_theta3)
        beta = math.atan2(z_rel, r)
        gamma = math.atan2(self.l2 * math.sin(theta3_math), self.l1 + self.l2 * math.cos(theta3_math))

        theta2 = -(beta - gamma)
        theta3 = -theta3_math
        theta4 = -(theta2 + theta3)
        return [theta1, theta2, theta3, theta4]

    # ---------------------------------------------------------- state machine
    def _goto(self, x, y, z):
        """Step current_ee toward target at ee_speed. Returns True when arrived."""
        step = self.ee_speed / 30.0
        dx = x - self.current_ee[0]
        dy = y - self.current_ee[1]
        dz = z - self.current_ee[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < self.pos_tol:
            return True
        s = min(step / dist, 1.0)
        self.current_ee[0] += dx * s
        self.current_ee[1] += dy * s
        self.current_ee[2] += dz * s
        return False

    def _set_gripper(self, open_gripper):
        if self.gripper_open != open_gripper:
            self.gripper_open = open_gripper
            self.get_logger().info("Opening gripper." if open_gripper else "Closing gripper.")

    def control_loop(self):
        self.state_ticks += 1
        hover_z = 0.08   # approach height above targets
        grab_z_off = 0.005

        if self.state == 'DONE':
            pass

        elif self.state == 'MOVE_ABOVE_BLOCK':
            p = self._block_pos(self.pick_block_id)
            if self._goto(p[0], p[1], p[2] + hover_z):
                self.state = 'DESCEND'
                self.state_ticks = 0

        elif self.state == 'DESCEND':
            p = self._block_pos(self.pick_block_id)
            if self._goto(p[0], p[1], max(p[2] + grab_z_off, 0.02)):
                self.state = 'CLOSE_GRIPPER'
                self.state_ticks = 0

        elif self.state == 'CLOSE_GRIPPER':
            self._set_gripper(False)
            if self.state_ticks >= self.settle_ticks:
                self.state = 'LIFT'
                self.state_ticks = 0

        elif self.state == 'LIFT':
            # Capture an absolute lift target once; computing it relative to the
            # moving EE every tick makes the goal recede endlessly.
            if self.state_ticks == 1:
                self._lift_target = min(self.current_ee[2] + 0.06, 0.30)
            if self._goto(self.current_ee[0], self.current_ee[1], self._lift_target):
                self.state = 'MOVE_ABOVE_STACK'
                self.state_ticks = 0

        elif self.state == 'MOVE_ABOVE_STACK':
            if self.place_block_id is None:
                t = list(self.stack_target)
            else:
                t = self._block_pos(self.place_block_id)
            if self._goto(t[0], t[1], t[2] + hover_z):
                self.state = 'PLACE'
                self.state_ticks = 0

        elif self.state == 'PLACE':
            if self.place_block_id is None:
                t = list(self.stack_target)
            else:
                t = self._block_pos(self.place_block_id)
            if self._goto(t[0], t[1], t[2] + 0.045):  # one block-height above surface
                self.state = 'OPEN_GRIPPER'
                self.state_ticks = 0

        elif self.state == 'OPEN_GRIPPER':
            self._set_gripper(True)
            if self.state_ticks >= self.settle_ticks:
                self.get_logger().info(f"Task complete: placed block {self.pick_block_id}.")
                if self.queue:
                    pick, place = self.queue.pop(0)
                    self._start_task(pick, place)
                else:
                    self.state = 'RETREAT'
                    self.state_ticks = 0

        elif self.state == 'RETREAT':
            if self._goto(0.15, 0.0, 0.15):
                self.state = 'DONE'

        # Workspace clamps
        self.current_ee[0] = max(min(self.current_ee[0], 0.30), 0.05)
        self.current_ee[1] = max(min(self.current_ee[1], 0.30), -0.30)
        self.current_ee[2] = max(min(self.current_ee[2], 0.30), 0.02)

        # IK with joint velocity limiting
        target_angles = self.solve_ik(*self.current_ee)
        dt = 1.0 / 30.0
        max_delta = self.max_velocity * dt
        for i in range(4):
            diff = target_angles[i] - self.current_angles[i]
            if abs(diff) > max_delta:
                self.current_angles[i] += math.copysign(max_delta, diff)
            else:
                self.current_angles[i] = target_angles[i]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_angles
        self.joint_pub.publish(msg)

        g_msg = Bool()
        g_msg.data = not self.gripper_open
        self.gripper_pub.publish(g_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StackingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

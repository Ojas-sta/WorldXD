import rclpy
from rclpy.node import Node
import math
import re
import threading
import time
import numpy as np
from cv_bridge import CvBridge

import torch
import torchvision.transforms as transforms
from jepa_model import JEPAWorldModel
import goal_renderer

from sensor_msgs.msg import Image, JointState, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String, Float32
from tf2_ros import Buffer, TransformListener

COLOR_TO_ID = {'red': 0, 'green': 1, 'blue': 2, 'yellow': 3}
BLOCK_HOME = {0: [0.15, 0.1, 0.02], 1: [0.20, 0.1, 0.02],
              2: [0.15, -0.1, 0.02], 3: [0.20, -0.1, 0.02]}

# Prompt verbs recognized for pick tasks (regex-escaped where needed)
_PICK_VERBS = r'(?:pick(?:ing)?(?:\s+up)?|grab(?:b(?:ed|ing))?|mov(?:e|es|ing)|tak(?:e|ing)|lif(?:t|ting)|plac(?:e|es|ing)|put|ke(?:pt|ep)|set(?:ting)?|get|gettings?|mak(?:e|ing))'
_ACTION_VERB = re.compile(
    r'\b(?:pick|grab|move|take|lift|place|put|keep|set|get|make|stack|arrange'
    r'|reset|clear|stop|separate|unstack|split)\b')
_PLACE_PATTERNS = [
    r'on\s*top\s+of\s+(?:the\s+)?',
    r'onto\s+(?:the\s+)?',
    r'on\s+(?:the\s+)?',
    r'over\s+(?:the\s+)?',
    r'above\s+(?:the\s+)?',
]
# P4.5: spatial-under relations. "A under B" => B must END UP on top,
# so the executed task is (pick=B, place=A).
_UNDER_PATTERN = r'(?:under|beneath|below|underneath)\s+(?:the\s+)?(red|green|blue|yellow)'


def c_name(idx):
    return {v: k for k, v in COLOR_TO_ID.items()}[idx]


def parse_prompt(text):
    """Parse a natural-language command into a task dict (pure function).

    Returns one of:
      {'action': 'reset'}
      {'action': 'arrange'}
      {'action': 'task', 'pick': id, 'place': id_or_None}   # place None = stack point
      {'action': None}                                      # unrecognized
    P4.5 spatial relations:
      "A on top of B"  -> task(A, B)
      "A under B"      -> task(B, A)   # B ends up above A
    Descriptive sentences with no action verb are ignored.
    """
    if not text or not text.strip():
        return {'action': None}
    t = text.lower().strip()

    # reset dominates: explicit stop/undo words
    if re.search(r'\b(reset|clear|stop|separate|unstack|split up?)\b', t):
        return {'action': 'reset'}

    # "arrange all" / "stack everything" style. A bare trailing 'all'
    # ("at ALL") must NOT trigger this — require action context.
    if ('arrange' in t or 'everything' in t or t == 'all'
            or re.search(r'all blocks|block together|blocks? together|'
                         r'each other|one tower|a tower', t)):
        return {'action': 'arrange'}

    # P4.5: descriptive sentences (no action verb anywhere) are not commands,
    # EXCEPT when an explicit spatial-relation phrase carries the intent.
    has_relation_ctx = bool(re.search(
        r'on\s*top\s+of|(?:under|beneath|below|underneath)\s+the?', t))
    if not (_ACTION_VERB.search(t) or has_relation_ctx):
        return {'action': None}
    # NOTE: purely descriptive sentences containing a spatial relation
    # ("the red block is under the blue block") ARE executed as commands --
    # deliberate robot-grammar simplification, documented in teste.md.

    def find_subject():
        m = re.search(_PICK_VERBS + r'(?:\s+\w+){0,2}?\s+(?:the\s+)?'
                      r'(red|green|blue|yellow)', t)
        if m:
            return COLOR_TO_ID[m.group(1)]
        for name, idx in COLOR_TO_ID.items():
            if re.search(r'\b' + name + r'\b', t):
                return idx
        return None

    # --- under/beneath/below relation (inverted pair) ---
    mentions_under = re.search(r'under|beneath|below|underneath', t)
    m_under = re.search(_UNDER_PATTERN, t)
    if mentions_under and not m_under:
        # under-intent with an unknown reference color: not executable
        return {'action': None}
    if m_under:
        ref = COLOR_TO_ID[m_under.group(1)]
        subj = find_subject()
        if subj is None:
            return {'action': None}
        if subj == ref:
            # subject fallback collided with the reference; if exactly one
            # OTHER color is mentioned, that one must be the mover
            others = {c for c in COLOR_TO_ID.values()
                      if re.search(r'\b' + c_name(c) + r'\b', t)} - {ref}
            if len(others) == 1:
                subj = others.pop()
            else:
                return {'action': None}      # "blue under blue": meaningless
        return {'action': 'task', 'pick': ref, 'place': subj}

    pick = find_subject()

    place = None
    for pat in _PLACE_PATTERNS:
        m_place = re.search(pat + r'(red|green|blue|yellow)', t)
        if m_place:
            cand = COLOR_TO_ID[m_place.group(1)]
            if cand != pick:
                place = cand
            break

    if pick is None:
        return {'action': None}
    return {'action': 'task', 'pick': pick, 'place': place}


class StackingController(Node):
    def __init__(self):
        super().__init__('stacking_controller')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper_closed', 10)
        self.fsm_pub = self.create_publisher(String, 'fsm_state', 10)
        self.jepa_telemetry_pub = self.create_publisher(Float32, 'jepa_telemetry', 10)
        # P4: RViz visibility into what the planner is planning TOWARD.
        # RViz's Camera display requires a sibling CameraInfo on
        # /jepa/camera_info or the view stays blank.
        self.goal_img_pub = self.create_publisher(Image, '/jepa/goal_image', 10)
        self.goal_info_pub = self.create_publisher(CameraInfo, '/jepa/camera_info', 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.prompt_sub = self.create_subscription(String, 'user_prompt', self.prompt_callback, 10)
        # Manual jog: interactive-marker drags stream /ee_target; while targets
        # arrive the controller tracks them (MANUAL state). Any task prompt exits.
        self.ee_target_sub = self.create_subscription(
            PointStamped, '/ee_target', self.ee_target_callback, 10)
        self.manual_gripper_sub = self.create_subscription(
            Bool, '/manual_gripper', self.manual_gripper_callback, 10)
        self._last_fsm_published = None
        self.last_ee_msg_time = 0.0

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
        self._goal_img_msg = None
        self.create_timer(1.0, self.publish_goal_image)   # RViz goal view feed
        self.get_logger().info("Stacking Controller Initialized.")

    def publish_goal_image(self):
        """Republish last rendered goal at 1Hz for the RViz 'JEPA Goal View'."""
        if self._goal_img_msg is not None:
            self._goal_img_msg.header.stamp = self.get_clock().now().to_msg()
            self.goal_img_pub.publish(self._goal_img_msg)
            info = CameraInfo()
            info.header = self._goal_img_msg.header
            info.height, info.width = 224, 224
            info.distortion_model = 'plumb_bob'
            f, c = 200.0, 112.0     # matches workspace_env projection
            # NOTE: every element must be a float literal — ROS2 msg validation
            # rejects ints ("each value of type 'float'")
            info.k = [f, 0.0, c, 0.0, f, c, 0.0, 0.0, 1.0]
            info.p = [f, 0.0, c, 0.0, 0.0, f, c, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.goal_info_pub.publish(info)

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
        # P4: render a REAL goal image — the desired end state with the picked
        # block stacked on its destination (replaces the black placeholder).
        try:
            blocks = [{'id': i, 'pos': list(self._block_pos(i))} for i in range(4)]
            base_pos = (list(self._block_pos(place_id))
                        if place_id is not None else list(self.stack_target))
            goal_blocks = [b for b in blocks if b['id'] != pick_id]
            level = 1 + sum(1 for b in goal_blocks
                            if abs(b['pos'][0] - base_pos[0]) < 0.02 and
                            abs(b['pos'][1] - base_pos[1]) < 0.02)
            goal_blocks.append({'id': pick_id,
                                'pos': [base_pos[0], base_pos[1],
                                        base_pos[2] + 0.04 * max(level - 1, 0)]})
            goal_img = goal_renderer.render_goal(goal_blocks)
            self.goal_tensor = self.transform(goal_img).unsqueeze(0).to(self.device)
            self._goal_img_msg = self.bridge.cv2_to_imgmsg(goal_img, encoding='bgr8')
            self._goal_img_msg.header.frame_id = 'camera_link'
            self._goal_img_msg.header.stamp = self.get_clock().now().to_msg()
            self.goal_img_pub.publish(self._goal_img_msg)
            self.get_logger().info("Goal image rendered for JEPA planner.")
        except Exception as e:
            self.get_logger().warn(f"Goal render failed (keeping previous): {e}")

    def prompt_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received prompt: {text[:120]}")
        task = parse_prompt(text)

        if task['action'] == 'reset':
            self.queue = []
            self.state = 'DONE'
            self.get_logger().info("Resetting.")
            return

        if task['action'] == 'arrange':
            self.queue = [(i, None) for i in range(4)]
            pick, place = self.queue.pop(0)
            self._start_task(pick, place)
            return

        if task['action'] == 'task':
            self.queue = []
            self._start_task(task['pick'], task['place'])
            return

        self.get_logger().info(
            "Prompt not recognized. Example: 'pick up the green block and place it "
            "on top of the yellow block'.")

    # ------------------------------------------------------------- JEPA worker
    def image_callback(self, msg):
        try:
            self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._frame_event.set()
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")

    def _jepa_worker(self):
        # P4-lag fix: CEM planning is expensive (~10s of MPS). Planning around
        # the clock -- even while idle -- starved RViz2's render loop on the
        # shared GPU (arm lag/glitch). Now: plan ONLY during active tasks,
        # max once per 20s, and sleep-poll otherwise.
        last_run = 0.0
        while True:
            if self.state == 'DONE' or time.monotonic() - last_run < 20.0:
                time.sleep(0.5)
                continue
            frame = getattr(self, '_latest_frame', None)
            if frame is None:
                time.sleep(0.25)
                continue
            try:
                img_tensor = self.transform(frame).unsqueeze(0).to(self.device)
                # P4: real proprio [ee_x, ee_y, ee_z, gripper_open] — encode()
                # normalizes against metaworld dataset stats internally.
                proprio = [self.current_ee[0], self.current_ee[1],
                           self.current_ee[2], 1.0 if self.gripper_open else 0.0]
                t0 = time.perf_counter()
                with torch.no_grad():
                    action = self.model.get_action(img_tensor, self.goal_tensor,
                                                   proprio=proprio)
                # Real inference latency for dashboard/CLI telemetry
                tele = Float32()
                tele.data = (time.perf_counter() - t0) * 1000.0
                self.jepa_telemetry_pub.publish(tele)
                self.latest_action = action
                last_run = time.monotonic()
            except Exception as e:
                self.get_logger().warn(f"JEPA inference error: {e}")
                time.sleep(2.0)
            finally:
                self._latest_frame = None

    # ------------------------------------------------------------- manual jog
    def ee_target_callback(self, msg):
        """Drag updates from the RViz interactive marker.

        P3.3 guardrail: ignored entirely while a task is running — JEPA/state
        machine owns the arm until it finishes.
        """
        if self.state not in ('DONE', 'MANUAL'):
            return
        self.manual_target = [msg.point.x, msg.point.y, msg.point.z]
        self.last_ee_msg_time = time.monotonic()
        if self.state != 'MANUAL':
            self.queue = []
            self.state = 'MANUAL'
            self.state_ticks = 0
            self.get_logger().info('MANUAL jog engaged (marker drag).')

    def manual_gripper_callback(self, msg):
        if self.state == 'MANUAL':
            self._set_gripper(bool(msg.data))

    def _stack_target_pos(self):
        """Dynamic stack point: fixed xy, z = current top-block center.

        P4.5 fix: arranging previously descended to a STATIC height every
        time, releasing blocks 2..n at the same altitude so they overlapped
        at the base level instead of building upward.
        """
        stx, sty = self.stack_target[0], self.stack_target[1]
        top = 0.02  # table-level first-layer center
        for i in range(4):
            bx, by, bz = self._block_pos(i)
            if abs(bx - stx) < 0.03 and abs(by - sty) < 0.03:
                top = max(top, bz)
        return [stx, sty, top]

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

        elif self.state == 'MANUAL':
            # Track the interactive-marker target live (it moves under the drag).
            # P3.6.2: MANUAL times out 3s after the last drag message so a stale
            # jog target can't make the arm wander through the scene later.
            if time.monotonic() - self.last_ee_msg_time > 3.0:
                self.get_logger().info('MANUAL timed out (no drag input).')
                self.state = 'RETREAT'
                self.state_ticks = 0
            else:
                t = getattr(self, 'manual_target', None)
                if t is not None:
                    self._goto(t[0], t[1], t[2])

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
                t = self._stack_target_pos()
            else:
                t = self._block_pos(self.place_block_id)
            if self._goto(t[0], t[1], t[2] + hover_z):
                self.state = 'PLACE'
                self.state_ticks = 0

        elif self.state == 'PLACE':
            if self.place_block_id is None:
                t = self._stack_target_pos()
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

        # Publish FSM state on transition (for dashboard/CLI telemetry)
        if self.state != self._last_fsm_published:
            self._last_fsm_published = self.state
            fsm_msg = String()
            fsm_msg.data = self.state
            self.fsm_pub.publish(fsm_msg)

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

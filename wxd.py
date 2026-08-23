"""wxd - WorldXD control center TUI.

Terminal dashboard for the robot sim: live FSM pipeline, joint readouts,
block table, prompt console, camera snapshot capture.

Run:  pixi run wxd          (or: pixi run python3 wxd.py)

Keys:
  a  arrange all blocks        r  reset scene
  g  pick green -> yellow      s  save camera frame to captures/
  p  focus prompt box          q  quit
"""
import os
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge

from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Bool, String, Float32
from visualization_msgs.msg import MarkerArray, Marker

from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import Header, Footer, Static, Input, RichLog, DataTable
from textual.reactive import reactive

BANNER = (
    "[bold cyan]"
    "\u2588\u2588\u2557    \u2588\u2588\u2557 \u2588\u2588\u2557  \u2588\u2588\u2557 \u250c\u2500\u2500\u2500\u2500\u2500\u2500\u2557 \u2588\u2588\u2557     \u2588\u2588\u2588\u2588\u2588\u2588\u2557 \u2588\u2588\u2557  \u2588\u2588\u2557\n"
    "\u2588\u2588\u2551    \u2588\u2588\u2551\u2588\u2588\u2554\u2550\u2550\u2550\u255d \u2588\u2588\u2551  \u2588\u2588\u2551\u2588\u2588\u2551    \u2588\u2588\u2554\u2550\u2550\u2550\u255d \u255a\u2588\u2588\u2557\u2588\u2588\u2554\u255d\n"
    "\u2588\u2588\u2551 \u2588\u2557 \u2588\u2588\u2551\u2588\u2588\u2551      \u2588\u2588\u2551  \u2588\u2588\u2551\u2588\u2588\u2551    \u2588\u2588\u2551       \u2588\u2588\u2551\u2588\u2588\u2551\n"
    "\u2588\u2588\u2551\u2588\u2588\u2588\u2557\u2588\u2588\u2551\u2588\u2588\u2551      \u2588\u2588\u2551  \u2588\u2588\u2551\u2588\u2588\u2551    \u2588\u2588\u2551       \u2588\u2588\u2551\u2588\u2588\u2551\n"
    "\u255a\u2588\u2588\u2554\u2550\u2588\u2588\u2554\u255d \u255a\u2588\u2588\u2588\u2588\u2588\u2557 \u2588\u2588\u2551  \u2588\u2588\u2551\u2588\u2588\u2551    \u255a\u2588\u2588\u2588\u2588\u2588\u2557 \u2588\u2588\u2551\u2588\u2588\u2551\n"
    " \u255a\u2550\u255d \u255a\u2550\u255d   \u255a\u2550\u2550\u2550\u2550\u255d \u255a\u2550\u255d  \u255a\u2550\u255d\u255a\u2550\u255d     \u255a\u2550\u2550\u2550\u2550\u255d \u255a\u2550\u255d \u255a\u2550\u255d[/] "
    "[dim]\u00b7 world-model robot arm control[/]"
)

FSM_STEPS = ['MOVE_ABOVE_BLOCK', 'DESCEND', 'CLOSE_GRIPPER', 'LIFT',
             'MOVE_ABOVE_STACK', 'PLACE', 'OPEN_GRIPPER', 'RETREAT']
COLOR_NAMES = {0: 'red', 1: 'green', 2: 'blue', 3: 'yellow'}


class RosNode(Node):
    """Headless ROS node feeding the TUI; runs on its own executor thread."""

    def __init__(self):
        super().__init__('wxd_tui')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.state = {
            'fsm': 'DONE', 'fsm_at': time.time(),
            'joints': [0.0] * 4, 'gripper': False,
            'inference_ms': None, 'blocks': [], 'last_frame': None,
        }
        self.prompt_pub = self.create_publisher(String, 'user_prompt', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Bool, '/gripper_closed', self.on_gripper, 10)
        self.create_subscription(String, 'fsm_state', self.on_fsm, 10)
        self.create_subscription(Float32, 'jepa_telemetry', self.on_jepa, 10)
        self.create_subscription(MarkerArray, 'workspace_blocks', self.on_blocks, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.on_camera, qos)

    def on_joints(self, msg):
        if len(msg.position) >= 4:
            with self.lock:
                self.state['joints'] = list(msg.position[:4])

    def on_gripper(self, msg):
        with self.lock:
            self.state['gripper'] = bool(msg.data)

    def on_fsm(self, msg):
        with self.lock:
            self.state['fsm'] = msg.data
            self.state['fsm_at'] = time.time()

    def on_jepa(self, msg):
        with self.lock:
            self.state['inference_ms'] = round(float(msg.data), 1)

    def on_blocks(self, msg):
        blocks = []
        for m in msg.markers:
            if m.type == Marker.CUBE:
                blocks.append((m.id, COLOR_NAMES.get(m.id, '?'),
                               m.pose.position.x, m.pose.position.y, m.pose.position.z))
        blocks.sort()
        with self.lock:
            self.state['blocks'] = blocks

    def on_camera(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.state['last_frame'] = frame
        except Exception:
            pass

    def send_prompt(self, text):
        msg = String()
        msg.data = text
        self.prompt_pub.publish(msg)


class StatusPanel(Static):
    """Live robot status: FSM, joints, gripper, JEPA latency."""

    def on_mount(self):
        self.set_interval(0.25, self.refresh_status)

    def refresh_status(self):
        ros = self.app.ros
        if ros is None:
            return
        with ros.lock:
            s = dict(ros.state)
        fsm = s['fsm']
        busy = fsm in FSM_STEPS
        step_idx = FSM_STEPS.index(fsm) if busy else -1

        # ASCII pipeline
        glyphs = []
        for i, name in enumerate(FSM_STEPS):
            short = {
                'MOVE_ABOVE_BLOCK': 'apprch', 'DESCEND': 'descnd',
                'CLOSE_GRIPPER': 'grip', 'LIFT': 'lift',
                'MOVE_ABOVE_STACK': 'transpt', 'PLACE': 'place',
                'OPEN_GRIPPER': 'releas', 'RETREAT': 'retreat',
            }[name]
            if busy and i == step_idx:
                glyphs.append(f'[bold cyan][{short}][/]')
            elif not busy or i < step_idx:
                glyphs.append(f'[green]{short}[/]')
            else:
                glyphs.append(f'[dim]{short}[/]')
        pipeline = ' \u2192 '.join(glyphs)

        j = s['joints']
        jd = [x * 180.0 / 3.14159 for x in j]
        inf = f"{s['inference_ms']:.0f} ms" if s['inference_ms'] else '-'
        grip = '[red]CLOSED[/]' if s['gripper'] else '[green]open[/]'
        state_line = (
            f"[bold]FSM:[/] {'[yellow]' + fsm + '[/]' if busy else '[dim]DONE (idle)[/]'}   "
            f"[bold]Gripper:[/] {grip}   [bold]CEM:[/] {inf}\n"
            f"{pipeline}\n"
            f"[bold]J1[/] {jd[0]:7.1f}\u00b0  [bold]J2[/] {jd[1]:7.1f}\u00b0  "
            f"[bold]J3[/] {jd[2]:7.1f}\u00b0  [bold]J4[/] {jd[3]:7.1f}\u00b0"
        )
        self.update(state_line)


class BlocksTable(DataTable):
    def on_mount(self):
        self.add_columns('ID', 'Color', 'x (m)', 'y (m)', 'z (m)')
        self.cursor_type = 'none'
        self.set_interval(0.5, self.refresh_blocks)

    def refresh_blocks(self):
        ros = self.app.ros
        if ros is None:
            return
        with ros.lock:
            blocks = list(ros.state['blocks'])
        self.clear()
        for bid, color, x, y, z in blocks:
            dot = {'red': '\u25a0 red', 'green': '\u25a0 green',
                   'blue': '\u25a0 blue', 'yellow': '\u25a0 yellow'}.get(color, color)
            self.add_row(str(bid), dot, f'{x:.3f}', f'{y:.3f}', f'{z:.3f}')


class WxdApp(App):
    CSS = """
    Screen { background: #0a0c12; }
    #banner { padding: 0 1; }
    .left-col { width: 1fr; margin: 0 0 0 1; }
    .right-col { width: 1fr; margin: 0 1; }
    Static.panel-box { border: round #2a3040; background: #0d1017;
                       padding: 1 2; margin-bottom: 1; }
    RichLog { height: 1fr; border: round #2a3040; background: #0d1017;
              margin-bottom: 1; }
    DataTable { height: auto; border: round #2a3040; background: #0d1017; }
    Input { dock: bottom; margin: 0 1 1 1; }
    """
    BINDINGS = [
        ('q', 'quit', 'Quit'),
        ('a', 'prompt_send', 'Arrange all'),
        ('r', 'reset_scene', 'Reset'),
        ('g', 'green_yellow', 'Green->Yel'),
        ('s', 'save_frame', 'Save cam'),
        ('p', 'focus_prompt', 'Prompt'),
    ]

    def __init__(self):
        super().__init__()
        self.ros = None
        self._last_fsm_logged = None

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal():
            with Vertical(classes='left-col'):
                yield Static(BANNER, id='banner')
                yield StatusPanel('', classes='panel-box', id='status')
                yield RichLog(highlight=False, markup=False, id='log')
            with Vertical(classes='right-col'):
                yield Static('[bold]Blocks[/]', classes='panel-box')
                yield BlocksTable()
                yield Static('[bold]Keys[/]: a arrange \u00b7 r reset \u00b7 g green\u2192yel '
                             '\u00b7 s save frame \u00b7 p prompt \u00b7 q quit',
                             classes='panel-box')
        yield Input(placeholder='Type a prompt for the robot and press Enter...', id='prompt')
        yield Footer()

    def on_mount(self):
        self.title = 'wxd \u2014 WorldXD'
        rclpy.init(args=None)
        self.ros = RosNode()
        threading.Thread(
            target=lambda: rclpy.spin(self.ros), daemon=True).start()
        self.set_interval(0.5, self.poll_log)

    def poll_log(self):
        """Surface FSM transitions into the log pane."""
        ros = self.ros
        if ros is None:
            return
        with ros.lock:
            fsm = ros.state['fsm']
        if fsm != self._last_fsm_logged:
            stamp = time.strftime('%H:%M:%S')
            log = self.query_one('#log', RichLog)
            if self._last_fsm_logged is None:
                log.write(f'[{stamp}] [bold cyan]wxd connected to ROS graph[/]')
            else:
                log.write(f'[{stamp}] FSM \u2192 [bold]{fsm}[/]')
            self._last_fsm_logged = fsm

    def dispatch_prompt(self, text):
        if self.ros is None:
            return
        self.ros.send_prompt(text)
        log = self.query_one('#log', RichLog)
        log.write(f'[{time.strftime("%H:%M:%S")}] prompt: "{text}"')

    def action_prompt_send(self):
        self.dispatch_prompt('arrange all blocks')

    def action_reset_scene(self):
        self.dispatch_prompt('reset')

    def action_green_yellow(self):
        self.dispatch_prompt('pick up the green block and place it on top of the yellow block')

    def action_focus_prompt(self):
        self.query_one('#prompt', Input).focus()

    def on_input_submitted(self, event: Input.Submitted):
        text = event.value.strip()
        if text:
            self.dispatch_prompt(text)
        event.input.value = ''

    def action_save_frame(self):
        if self.ros is None:
            return
        with self.ros.lock:
            frame = self.ros.state['last_frame']
        if frame is None:
            return
        os.makedirs('captures', exist_ok=True)
        path = os.path.join('captures', time.strftime('cam_%Y%m%d_%H%M%S.png'))
        cv2.imwrite(path, frame)
        log = self.query_one('#log', RichLog)
        log.write(f'[{time.strftime("%H:%M:%S")}] saved {path}')

    def action_quit(self):
        self.exit()


def main():
    WxdApp().run()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

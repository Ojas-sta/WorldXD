import rclpy
from rclpy.node import Node
import math
import numpy as np
import cv2
from cv_bridge import CvBridge

import torch
import torchvision.transforms as transforms
from jepa_model import JEPAWorldModel

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

class StackingController(Node):
    def __init__(self):
        super().__init__('stacking_controller')
        
        # Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper_closed', 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.prompt_sub = self.create_subscription(String, 'user_prompt', self.prompt_callback, 10)
        
        self.bridge = CvBridge()
        
        # Load JEPA Model (dummy initialized)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = JEPAWorldModel().to(self.device)
        self.model.eval()
        
        # Standard transform
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        
        # Dummy goal image (empty black image for now, ideally rendered stacked tower)
        dummy_goal = np.zeros((224, 224, 3), dtype=np.uint8)
        self.goal_tensor = self.transform(dummy_goal).unsqueeze(0).to(self.device)
        
        # Kinematics variables
        self.l1 = 0.134
        self.l2 = 0.120
        self.base_z = 0.078
        self.current_angles = [0.0, 0.0, 0.0, 0.0]
        self.max_velocity = 41.8879 # 400 rpm limit
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # State Machine variables
        self.state = 'DONE' # Start idle
        self.target_block = 0
        self.stacked_count = 0
        self.stack_all = False
        self.stack_target = [0.25, 0.0, 0.02]
        self.gripper_open = True
        self.state_timer = 0
        
        # TF to find blocks
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0/30.0, self.control_loop)
        self.get_logger().info("Stacking Controller Initialized.")
        
    def prompt_callback(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f"Received prompt: {text}")
        
        if 'reset' in text or 'down' in text or 'separate' in text:
            self.stacked_count = 0
            self.stack_all = False
            self.state = 'DONE'
            self.get_logger().info("Resetting stack counter.")
            return
            
        if 'arrange' in text or 'all' in text:
            self.stack_all = True
            self.target_block = 0
            self.state = 'IDENTIFY'
            self.state_timer = 0
            self.get_logger().info("Executing prompt: Stacking all blocks sequentially")
            return
            
        target = -1
        if 'red' in text: target = 0
        elif 'green' in text: target = 1
        elif 'blue' in text: target = 2
        elif 'yellow' in text: target = 3
        
        if target != -1:
            self.stack_all = False
            self.target_block = target
            self.state = 'IDENTIFY'
            self.state_timer = 0
            self.get_logger().info(f"Executing prompt: Moving block {target}")
        else:
            self.get_logger().info("Prompt not recognized. Example: 'stack the red box'.")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_tensor = self.transform(cv_img).unsqueeze(0).to(self.device)
            
            with torch.no_grad():
                # Inference using JEPA (actions bounded [-1, 1])
                jepa_action = self.model.get_action(img_tensor, self.goal_tensor)
                self.latest_action = jepa_action
                
        except Exception as e:
            self.get_logger().warn(f"Image processing error: {e}")

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
        
        theta2_math = beta - gamma
        theta2 = -theta2_math
        theta3 = -theta3_math
        theta4 = -(theta2 + theta3)
        return [theta1, theta2, theta3, theta4]

    def control_loop(self):
        if not hasattr(self, 'current_ee'):
            self.current_ee = [0.15, 0.0, 0.15]
            self.latest_action = [0.0, 0.0, 0.0, 0.0]

        if self.state != 'DONE' and self.latest_action is not None:
            # Action is [dx, dy, dz, dgripper]
            # Use small gains to map [-1, 1] network outputs to meter displacements
            gain = 0.05
            self.current_ee[0] += self.latest_action[0] * gain
            self.current_ee[1] += self.latest_action[1] * gain
            self.current_ee[2] += self.latest_action[2] * gain
            
            # Constrain to reasonable workspace
            self.current_ee[0] = max(min(self.current_ee[0], 0.3), 0.05)
            self.current_ee[1] = max(min(self.current_ee[1], 0.3), -0.3)
            self.current_ee[2] = max(min(self.current_ee[2], 0.3), 0.02)
            
            if self.latest_action[3] > 0.0:
                self.gripper_open = True
            else:
                self.gripper_open = False
        else:
            self.current_ee = [0.15, 0.0, 0.15]

        target_angles = self.solve_ik(self.current_ee[0], self.current_ee[1], self.current_ee[2])
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

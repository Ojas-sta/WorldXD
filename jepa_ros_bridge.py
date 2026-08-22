import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import torch
import numpy as np
import time

from jepa_world_model import JEPAWorldModel

class JepaRosBridge(Node):
    def __init__(self):
        super().__init__('jepa_ros_bridge')
        
        # Initialize publisher for joint states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # Initialize JEPA model
        self.model = JEPAWorldModel()
        self.model.eval() # Inference mode
        
        # The 4 joints defined in our URDF
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Timer for 30Hz control loop
        timer_period = 1.0 / 30.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("JEPA ROS 2 Bridge Initialized. Publishing to /joint_states.")

    def timer_callback(self):
        # 1. Generate a dummy 2D image (Batch=1, Channels=3, H=64, W=64)
        dummy_image = torch.randn(1, 3, 64, 64)
        
        # 2. Get action from JEPA policy
        with torch.no_grad():
            action = self.model.get_action(dummy_image)
            
        # Action is in range [-1, 1]. Map it to joint angles (e.g., [-pi/2, pi/2])
        joint_angles = action.squeeze(0).numpy() * (np.pi / 2.0)
        
        # 3. Create JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_angles.tolist()
        
        # 4. Publish
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    jepa_node = JepaRosBridge()
    
    try:
        rclpy.spin(jepa_node)
    except KeyboardInterrupt:
        pass
        
    jepa_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

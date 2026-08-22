import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

class IKInteractiveTracker(Node):
    def __init__(self):
        super().__init__('ik_interactive_tracker')
        
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.server = InteractiveMarkerServer(self, 'ik_target')
        
        # Arm lengths matching URDF
        self.l1 = 0.134 # lower arm length
        self.l2 = 0.120 # upper arm length
        self.base_z = 0.078 # Z offset of shoulder joint from base_link
        
        # Initial target position (default stretched out)
        self.target_x = 0.150
        self.target_y = 0.0
        self.target_z = 0.078
        
        self.current_angles = [0.0, 0.0, 0.0, 0.0]
        self.max_velocity = 41.8879 # rad/s (400 RPM)
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        self.create_interactive_marker()
        
        # Control loop at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("IK Interactive Tracker Initialized.")

    def create_interactive_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "target_marker"
        int_marker.description = "IK Target"
        int_marker.scale = 0.1
        
        int_marker.pose.position.x = self.target_x
        int_marker.pose.position.y = self.target_y
        int_marker.pose.position.z = self.target_z
        
        # Create a sphere visual for the marker
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.03
        sphere_marker.scale.y = 0.03
        sphere_marker.scale.z = 0.03
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.2
        sphere_marker.color.b = 0.2
        sphere_marker.color.a = 0.9
        
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere_control.name = "move_3d"
        sphere_control.markers.append(sphere_marker)
        int_marker.controls.append(sphere_control)
        
        # Translation axes
        for axis in ['x', 'y', 'z']:
            control = InteractiveMarkerControl()
            control.name = f"move_{axis}"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if axis == 'x':
                control.orientation.w = 1.0; control.orientation.x = 1.0; control.orientation.y = 0.0; control.orientation.z = 0.0
            if axis == 'y':
                control.orientation.w = 1.0; control.orientation.x = 0.0; control.orientation.y = 1.0; control.orientation.z = 0.0
            if axis == 'z':
                control.orientation.w = 1.0; control.orientation.x = 0.0; control.orientation.y = 0.0; control.orientation.z = 1.0
            int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        self.target_x = feedback.pose.position.x
        self.target_y = feedback.pose.position.y
        self.target_z = feedback.pose.position.z

    def solve_ik(self, x, y, z):
        # Base Yaw
        theta1 = math.atan2(y, x)
        
        # Convert to 2D planar problem
        r = math.sqrt(x**2 + y**2)
        z_rel = z - self.base_z
        
        # Distance squared from shoulder to target
        d_sq = r**2 + z_rel**2
        d = math.sqrt(d_sq)
        
        # Bounds check to prevent math domain errors if dragged out of reach
        if d > (self.l1 + self.l2):
            scale = (self.l1 + self.l2) / d * 0.99
            r *= scale
            z_rel *= scale
            d_sq = r**2 + z_rel**2
            d = math.sqrt(d_sq)
        
        # Cosine rule for theta3
        cos_theta3 = (d_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta3 = max(min(cos_theta3, 1.0), -1.0)
        
        # Elbow up configuration uses positive theta3 in our math
        theta3_math = math.atan2(-math.sqrt(1 - cos_theta3**2), cos_theta3)
        
        beta = math.atan2(z_rel, r)
        gamma = math.atan2(self.l2 * math.sin(theta3_math), self.l1 + self.l2 * math.cos(theta3_math))
        
        theta2_math = beta - gamma
        
        # URDF has rotation around Y pitching the arm DOWN when positive
        theta2 = -theta2_math
        theta3 = -theta3_math
        
        # Keep wrist horizontal (gripper parallel to ground)
        theta4 = -(theta2 + theta3)
        
        return [theta1, theta2, theta3, theta4]

    def timer_callback(self):
        target_angles = self.solve_ik(self.target_x, self.target_y, self.target_z)
        
        # Enforce max velocity (400 RPM)
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
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKInteractiveTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

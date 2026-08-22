import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import tkinter as tk

class PromptNode(Node):
    def __init__(self):
        super().__init__('prompt_node')
        self.publisher_ = self.create_publisher(String, 'user_prompt', 10)
        
    def send_prompt(self, text):
        if text.strip():
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Sent prompt: "{msg.data}"')

def run_gui(node):
    root = tk.Tk()
    root.title("JEPA Stack Prompt")
    
    # Make it float on top
    root.attributes('-topmost', True)
    root.geometry("400x100")
    
    label = tk.Label(root, text="Enter prompt for the robot:")
    label.pack(pady=5)
    
    entry = tk.Entry(root, width=40)
    entry.pack(pady=5)
    
    def on_submit(event=None):
        text = entry.get()
        node.send_prompt(text)
        entry.delete(0, tk.END)
        
    entry.bind('<Return>', on_submit)
    
    btn = tk.Button(root, text="Send to JEPA", command=on_submit)
    btn.pack()
    
    # Check for ROS shutdown and spin node
    def check_ros():
        if not rclpy.ok():
            root.destroy()
        else:
            rclpy.spin_once(node, timeout_sec=0.01)
            root.after(50, check_ros)
            
    root.after(50, check_ros)
    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = PromptNode()
    
    try:
        run_gui(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import subprocess
import time
import os
import signal
import sys

def main():
    print("Starting JEPA Robotic Arm Simulation...")

    urdf_path = os.path.abspath("robot_description/urdf/eezybotarm.urdf")
    rviz_config_path = os.path.abspath("robot_description/rviz/config.rviz")
    
    # Generate default rviz config if it doesn't exist
    os.makedirs(os.path.dirname(rviz_config_path), exist_ok=True)
    if not os.path.exists(rviz_config_path):
        with open(rviz_config_path, "w") as f:
            f.write("""Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
      Cell Size: 0.1
      Plane Cell Count: 20
    - Class: rviz_default_plugins/RobotModel
      Description Source: Topic
      Description Topic:
        Value: /robot_description
      Name: RobotModel
      Value: true
    - Class: rviz_default_plugins/InteractiveMarkers
      Name: InteractiveMarkers
      Update Topic:
        Value: /ik_target/update
      Value: true
  Global Options:
    Fixed Frame: base_link
    Frame Rate: 30
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 0.6
      Focal Point:
        X: 0
        Y: 0
        Z: 0.1
      Name: Current View
      Pitch: 0.4
      Target Frame: base_link
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.8
Window Geometry:
  Height: 800
  Width: 1200
""")

    processes = []
    
    try:
        # 1. Start robot_state_publisher
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
            
        print("Launching robot_state_publisher...")
        rsp_proc = subprocess.Popen(
            ["ros2", "run", "robot_state_publisher", "robot_state_publisher", urdf_path]
        )
        processes.append(rsp_proc)
        
        # 2. Start Workspace Environment
        print("Launching Workspace Environment (Blocks, TF, Camera)...")
        workspace_proc = subprocess.Popen(
            ["python3", "workspace_env.py"]
        )
        processes.append(workspace_proc)
        
        # 3. Start Stacking Controller (JEPA + State Machine)
        print("Launching Stacking Controller...")
        controller_proc = subprocess.Popen(
            ["python3", "stacking_controller.py"]
        )
        processes.append(controller_proc)

        # 3b. Start Manual Jog Marker (RViz interactive marker)
        print("Launching Manual Marker...")
        marker_proc = subprocess.Popen(
            ["python3", "manual_marker.py"]
        )
        processes.append(marker_proc)
        
        # 4. Start RViz2
        print("Launching RViz2...")
        rviz_proc = subprocess.Popen(
            ["rviz2", "-d", rviz_config_path]
        )
        processes.append(rviz_proc)
        
        # 5. Launch Prompt GUI
        print("Launching Prompt GUI...")
        prompt_proc = subprocess.Popen([
            "pixi", "run", "python3", "terminal_prompt.py"
        ])
        processes.append(prompt_proc)

        # Wait for RViz to close
        rviz_proc.wait()

    except KeyboardInterrupt:
        print("\nShutting down simulation...")
    finally:
        for p in processes:
            if p.poll() is None:
                p.terminate()
        for p in processes:
            p.wait()
        print("Simulation stopped.")

if __name__ == "__main__":
    main()

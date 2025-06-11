# LeRobot Hackathon 2025 at Robotec.ai

### Directory structure:

- `sim` - o3de project
- `ros2_ws` - ros2 workspace

# Usage

1. Build the O3DE project:
```bash
cd sim
cmake --preset linux-default
cmake --build --preset linux-editor
```

2. Build and source ROS2 workspace:
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.sh
```

3. Open the O3DE editor:
```
./sim/build/linux/bin/profile/Editor
```

Load the only level and press play.

4. Run the joint state publisher package to manually set the arm's joints' values.
```bash
ros2 launch lerobot_o3de joint_state_publisher.launch.py
```

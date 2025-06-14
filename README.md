# LeRobot Hackathon 2025 at Robotec.ai

### Directory structure:

- `sim` - o3de project
- `ros2_ws` - ros2 workspace

# Usage

1. Clone repository
```bash
git clone --recurse-submodules git@github.com:RobotecAI/lerobot-2025-hackaton.git
cd lerobot-2025-hackaton
```

2. Build the O3DE project:
```bash
mkdir o3de-modules
git clone --branch stabilization/25050 https://github.com/o3de/o3de.git o3de-modules/o3de
git clone --branch stabilization/25050 https://github.com/o3de/o3de-extras.git o3de-modules/o3de-extras

export BASE=$(pwd)

cd $BASE/o3de-modules/o3de
git lfs install
git lfs pull
python/get_python.sh
scripts/o3de.sh register --this-engine

$BASE/o3de-modules/o3de/scripts/o3de.sh register -agp $BASE/o3de-modules/o3de-extras/Gems


cd $BASE/sim
cmake -B build/linux -G "Ninja Multi-Config" -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile --target Editor 
```

3. Build and source ROS2 workspace:
```bash
cd $BASE
cd ros2_ws
colcon build --symlink-install
source install/setup.sh
```

4. Open the O3DE editor:
```
../sim/build/linux/bin/profile/Editor
```

Load the only level and press play.

5. Run the joint state publisher package to manually set the arm's joints' values.
```bash
ros2 launch lerobot_o3de joint_state_publisher.launch.py
```

6. Setup Python modules
```bash
cd $BASE
# install uv package manager
curl -LsSf https://astral.sh/uv/install.sh | sh
uv sync -p 3.10 # for Ubuntu 22.04
uv sync -p 3.12 # for Ubuntu 24.04

# install development tools
sudo apt install shellcheck
pre-commit install
```

7. Test python env

```bash
source setup_shell.sh
python scripts/test_setup.py
```

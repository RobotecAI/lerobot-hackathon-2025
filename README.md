# LeRobot Hackathon 2025 at Robotec.ai

![](docs/image.png)

This project showcases the integration of [RAI](https://github.com/RobotecAI/rai) with the [LeRobot SO-101 arm](https://github.com/TheRobotStudio/SO-ARM100), mounted on a [ROSbot XL](https://husarion.com/manuals/rosbot-xl/overview) inside the [O3DE](https://www.o3de.org/) simulation environment. It was developed during the LeRobot Hackathon 2025 by the team LeRobotec at Robotec.ai, during which we successfully recorded a dataset of 20 demonstrations of the SO-101 arm picking up a red ball using teleoperation with the follower robot inside the simulation! Here's how one of the demonstrations looks like:

![](docs/episode_000009.gif)

The robot can navigate around the environment using [nav2](https://docs.nav2.org/) and pick up red balls using the finetuned policy. All the tasks are executed by the RAI agent, which can be interacted with using the human-friendly [Streamlit](https://streamlit.io/) interface.

# Usage

> Note: This project was only tested on Ubuntu 22.04 with ROS2 Humble. Other Ubuntu versions or ROS2 distributions may require additional adjustments.

> Note: In the instructions below, we assume that you have already installed ROS2 Humble and that you have sourced the ROS2 setup script in each terminal.

0. Install nav2 dependencies

```bash
sudo apt install ros-humble-nav2-bringup
```

1. Clone repository
```bash
git clone --recurse-submodules git@github.com:RobotecAI/lerobot-hackathon-2025.git
cd lerobot-hackathon-2025

export BASE=$(pwd)
```

2. Build and source ROS2 workspace:
```bash
cd $BASE
cd ros2_ws
colcon build --symlink-install
source install/setup.sh
```

3. Setup Python modules
```bash
cd $BASE

# install uv package manager
curl -LsSf https://astral.sh/uv/install.sh | sh
uv sync -p 3.10
```

4. Test python env

```bash
cd $BASE
source setup_shell.sh
python scripts/test_setup.py
```

5. Download the finetuned policy weights and the simulation binary

```bash
cd $BASE

wget -q --show-progress https://robotec-ml-rai-public.s3.eu-north-1.amazonaws.com/LeRobotec2025Demo_jammyhumble.zip
unzip LeRobotec2025Demo_jammyhumble.zip

wget -q --show-progress https://robotec-ml-rai-public.s3.eu-north-1.amazonaws.com/pretrained_model.zip
unzip pretrained_model.zip
```

6. Run the simulation package.

```bash
cd $BASE
ros2 launch lerobot_o3de demo.launch.py game_launcher:=./LeRobotec2025Demo_jammyhumble/sim.GameLauncher
```

7. In second terminal, run RAI in streamlit

```bash
cd $BASE
source setup_shell.sh
streamlit run scripts/rosbot-xl-demo.py
```
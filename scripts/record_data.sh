#!/usr/bin/env bash

HF_USER=$(huggingface-cli whoami | head -n 1)
FOLLOWER_PORT=/dev/ttyACM1
LEADER_PORT=/dev/ttyACM0
TIME=$(date +%Y%m%d_%H%M%S)

uv run python -m lerobot.record \
    --robot.type=so101_follower \
    --robot.port=$FOLLOWER_PORT \
    --robot.id=follower \
    --robot.cameras="{ up: {type: opencv, index_or_path: /dev/video4, width: 640, height: 480, fps: 15} }" \
    --teleop.type=so101_leader \
    --teleop.port=$LEADER_PORT \
    --teleop.id=leader \
    --display_data=true \
    --dataset.repo_id=${HF_USER}/record-test-sim-v1.1 \
    --dataset.num_episodes=0 \
    --dataset.single_task="Pick up the red ball" \
    --dataset.push_to_hub=true \
    --dataset.episode_time_s=15 \
    --dataset.reset_time_s=10 \
    --resume=true


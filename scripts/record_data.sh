#!/usr/bin/env bash

HF_USER=$(huggingface-cli whoami | head -n 1)
FOLLOWER_PORT=/dev/ttyACM1
LEADER_PORT=/dev/ttyACM0
TIME=$(date +%Y%m%d_%H%M%S)

uv run python -m lerobot.record \
    --robot.type=so101_follower \
    --robot.port=$FOLLOWER_PORT \
    --robot.id=follower \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 720, height: 544, fps: 30}}" \
    --teleop.type=so101_leader \
    --dataset.repo_id=lerobotec/so101_test \
    --teleop.port=$LEADER_PORT \
    --teleop.id=leader \
    --display_data=true \
    --dataset.repo_id=${HF_USER}/record-test-20250614_141155 \
    --dataset.num_episodes=20 \
    --dataset.single_task="Grab the red ball" \
    --dataset.push_to_hub=False \
    --dataset.episode_time_s=20 \
    --dataset.reset_time_s=10


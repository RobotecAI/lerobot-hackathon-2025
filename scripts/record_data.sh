#!/usr/bin/env bash

PORT=/dev/ttyACM0 

uv run python -m lerobot.record \
    --robot.type=so101_follower \
    --robot.port=$PORT \
    --robot.id=follower \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --teleop.type=so101_leader \
    --dataset.repo_id=lerobotec/so101_test \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=leader \
    --display_data=true \


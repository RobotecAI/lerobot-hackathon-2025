source setup_shell.sh
rm -rf ./lerobot_eval_cache
HF_USER=$(huggingface-cli whoami | head -n 1)
HF_LEROBOT_HOME=./lerobot_eval_cache python -m lerobot.record  \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.cameras="{ up: {type: opencv, index_or_path: /dev/video4, width: 640, height: 480, fps: 15} }" \
  --robot.id=follower \
  --display_data=true \
  --dataset.repo_id=${HF_USER}/eval_super_policy \
  --dataset.single_task="Pick up the red ball" \
  --policy.path=./pretrained_model

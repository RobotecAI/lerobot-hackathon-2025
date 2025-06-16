#!/usr/bin/env bash

HF_USER=$(huggingface-cli whoami | head -n 1)
TIME=20250614_141155

uv run python modules/lerobot/lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/record-test-${TIME} \
  --policy.type=act \
  --output_dir=outputs/train/act_so101_test \
  --job_name=act_so101_test \
  --policy.device=cuda \
  --wandb.enable=true 

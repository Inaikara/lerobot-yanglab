python -m lerobot.scripts.lerobot_train \
    --policy.type=diffusion \
    --policy.push_to_hub=false \
    --dataset.repo_id=/home/yanglab/lerobot/datasets/dp_vel \
    --log_freq 10 \
    --batch_size 256 \
    --steps 2000 \
    --save_freq 200 \
    --save_checkpoint=true
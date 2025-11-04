sudo chmod 666 /dev/ttyUSB0
rm -rf ./outputs/eval_cn_obs
lerobot-record \
    --robot.type=elite \
    --robot.action_type=joint \
    --dataset.repo_id=yanglab/eval_dp_obs \
    --dataset.single_task="pick and place" \
    --dataset.root=outputs/eval_cn_obs \
    --dataset.push_to_hub=false \
    --dataset.fps=30 \
    --dataset.episode_time_s=90 \
    --dataset.reset_time_s=30 \
    --dataset.num_episodes=1 \
    --display_data=true \
    --policy.path=outputs/train/2025-11-04/15-22-58_diffusion/checkpoints/last/pretrained_model \
    --policy.device=cuda \
    --policy.use_amp=false

sudo chmod 666 /dev/ttyUSB0

lerobot-record \
    --robot.type=elite \
    --teleop.type=touchx \
    --dataset.repo_id=yanglab/cn_vel \
    --dataset.single_task="pick and place" \
    --dataset.root=datasets/cn_vel \
    --dataset.push_to_hub=false \
    --dataset.fps=30 \
    --dataset.num_episodes=10 \
    --display_data=true \
    --robot.delta_position_step=0.2

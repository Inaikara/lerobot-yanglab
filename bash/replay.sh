sudo chmod 666 /dev/ttyUSB0

lerobot-replay \
    --robot.type=elite \
    --robot.action_type=joint \
    --dataset.repo_id=/home/yanglab/lerobot/datasets/test \
    --dataset.episode=0
#!/bin/bash

run_id=7

python3 trainer.py --gym_env Velmwheel50-v1 --min_goal_dist 1.0  --timesteps 50000
python3 trainer.py --gym_env Velmwheel50-v1 --min_goal_dist 0.5  --timesteps 50000 --model ./models/velmwheel_v1/dqn/$run_id/dqn_50000_steps.zip  --replay_buffer ./models/velmwheel_v1/dqn/$run_id/dqn_replay_buffer_50000_steps.pkl
python3 trainer.py --gym_env Velmwheel50-v1 --min_goal_dist 0.25 --timesteps 50000 --model ./models/velmwheel_v1/dqn/$run_id/dqn_100000_steps.zip --replay_buffer ./models/velmwheel_v1/dqn/$run_id/dqn_replay_buffer_100000_steps.pkl
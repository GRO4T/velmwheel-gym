import argparse
import configparser
import math
import pickle
import time

import gymnasium as gym

from velmwheel_gym import *  # pylint: disable=wildcard-import, unused-wildcard-import
from velmwheel_gym.constants import MAX_REPLANNING_ATTEMPTS, NAVIGATION_DIFFICULTIES
from velmwheel_gym.logger import init_logging
from velmwheel_gym.types import Point
from velmwheel_rl.common import ParameterReader, bootstrap_argument_parser, load_model

# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #

parser = bootstrap_argument_parser()
parser.add_argument("--goal_x", type=float, help="Goal x coordinate")
parser.add_argument("--goal_y", type=float, help="Goal y coordinate")
parser.add_argument("--start_x", type=float, help="Start x coordinate")
parser.add_argument("--start_y", type=float, help="Start y coordinate")
parser.add_argument(
    "--render", type=bool, help="Render the environment", required=False
)
parser.add_argument(
    "--random",
    action=argparse.BooleanOptionalAction,
    help="Generate random goals",
    required=False,
)
parser.add_argument(
    "--save_footprint",
    type=str,
    help="Save the robot footprint",
    required=False,
    default="false",
)
parser.add_argument(
    "--time_limit",
    type=int,
    help="Time limit for the episode",
    required=False,
    default=3600,
)
parser.add_argument(
    "--benchmark",
    action=argparse.BooleanOptionalAction,
    help="Benchmark the model",
    required=False,
)
parser.add_argument(
    "--continue_benchmark",
    action=argparse.BooleanOptionalAction,
    help="Continue the run",
)

args = parser.parse_args()
config = configparser.ConfigParser()
config.read([args.config])
param_reader = ParameterReader("tester", args, config)

# pylint: disable=duplicate-code
log_level = param_reader.read("log_level")
gym_env = param_reader.read("gym_env")
algorithm = param_reader.read("algorithm")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")
navigation_difficulty_level = int(param_reader.read("navigation_difficulty_level"))
real_time_factor = float(param_reader.read("real_time_factor"))
goal_x = float(param_reader.read("goal_x"))
goal_y = float(param_reader.read("goal_y"))
start_x = float(param_reader.read("start_x"))
start_y = float(param_reader.read("start_y"))
render = param_reader.read("render")
save_footprint = True if param_reader.read("save_footprint") == "true" else False
time_limit = int(param_reader.read("time_limit"))

init_logging(log_level)

# ---------------------------------------------------------------------------- #
#                               Testing the model                              #
# ---------------------------------------------------------------------------- #

difficulty = NAVIGATION_DIFFICULTIES[navigation_difficulty_level]

env = gym.make(
    gym_env,
    difficulty=difficulty,
    real_time_factor=real_time_factor,
    render_mode="human" if render == "true" else None,
    training_mode=False,
    global_path_segment_length=float(
        param_reader.read("global_path_segment_length", "VelmwheelGym")
    ),
    variant=param_reader.read("variant", "VelmwheelGym"),
)

model, _ = load_model(
    algorithm, env, param_reader, model_path, replay_buffer_path, test_mode=True
)


def run_once(start_pos, goal) -> dict:
    run = {
        "actions": [],
        "velocities": [],
        "min_goal_dist": math.inf,
        "start": start_pos,
        "end": goal,
        "duration": None,
        "footprints": {
            "positions": [],
            "orientations": [],
            "time": [],
            "obstacles": [],
        },
        "success": False,
        "collided_at": None,
        "timed_out_at": None,
    }

    options = (
        None if start_pos is None else {"starting_position": start_pos, "goal": goal}
    )
    obs, _ = env.reset(options=options)
    start = time.time()
    total = 0.0
    while True:
        action, _ = model.predict(obs, deterministic=True)
        run["actions"].append(action)
        run["velocities"].append(env.env.robot_velocity)

        obs, reward, terminated, truncated, info = env.step(action)

        run["footprints"]["positions"].append(env.env.robot_position)
        run["footprints"]["orientations"].append(env.env.robot_orientation)
        run["footprints"]["time"].append(start)
        run["footprints"]["obstacles"].append(env.env.obstacles)

        # footprints["obstacles"].append(
        #     [(x, y) for x, y in zip(env.env.robot.env.dynamic_obstacles_x, env.env.robot.env.dynamic_obstacles_y)]
        # )

        dist_to_goal = math.dist(env.env.goal, env.env.robot_position)

        print("----------------------------------------------")
        print(f"{action=}")
        print(f"{reward=}")
        print(f"{terminated=}")
        print(f"{truncated=}")
        print(f"{dist_to_goal=}")
        print(f"min_goal_dist{run['min_goal_dist']}")
        print(f"{env.env.starting_position=}")
        print(f"{env.env.goal=}")
        print(f"{env.env.robot_position=}")
        print(f"{info=}")
        print(f"{obs=}")
        print("----------------------------------------------")

        if info.get("status", None) == "segment_reached":
            env.reset()

        if terminated and reward < 0:
            print("Collision detected!")
            run["collided_at"] = env.env.robot_position
            env.step([0.0, 0.0, 0.0])
            break

        if truncated:
            print("Max steps reached!")
            if info["replanning_count"] >= MAX_REPLANNING_ATTEMPTS:
                run["timed_out_at"] = env.env.robot_position
                env.step([0.0, 0.0, 0.0])
                break
            else:
                env.reset()

        if dist_to_goal < run["min_goal_dist"]:
            run["min_goal_dist"] = dist_to_goal

        if dist_to_goal < difficulty.goal_reached_threshold:
            print("Goal reached!")
            env.step([0.0, 0.0, 0.0])
            run["success"] = True
            break

        if render == "true":
            env.render()

        end = time.time()
        elapsed = end - start
        start = end
        total += elapsed
        # if elapsed < 0.050:
        #     time.sleep(0.050 - elapsed)
        print(f"fps: {1 / elapsed}")

        # if total > time_limit:
        #     print("Time limit reached!")
        #     env.step([0.0, 0.0, 0.0])
        #     break

    run["duration"] = total
    return run


runs = []
if args.continue_benchmark:
    with open("run.pkl", "rb") as f:
        runs = pickle.load(f)

if args.benchmark:
    points = [
        Point(3.0, 3.0),
        Point(3.0, -3.0),
        Point(-3.0, 3.0),
        Point(-3.0, -3.0),
        Point(7.0, 7.0),
        Point(-7.0, 7.0),
        Point(7.0, -7.0),
        Point(-7.0, -7.0),
    ]
    success_rate = 0.0
    attempts = 0
    combos = [(x, y) for x in points for y in points if x != y]
    for i in range(1):
        for start, goal in combos:
            time.sleep(1)
            attempts += 1
            run = run_once(start, goal)
            success = "success" if run["success"] else "failure"
            success_rate = (
                success_rate * (attempts - 1) + (1 if run["success"] else 0)
            ) / attempts
            print(
                f"[benchmark] success_rate={round(success_rate, 2)} min_goal_dist={round(run['min_goal_dist'], 2)} i: {i} {start} -> {goal}: {success}"
            )
            runs.append(run)
            with open("run.pkl", "wb") as f:
                pickle.dump(runs, f)

elif args.random:
    while True:
        run_once(start_pos=None, goal=None)
else:
    runs.append(run_once(Point(start_x, start_y), Point(goal_x, goal_y)))

with open("run.pkl", "wb") as f:
    pickle.dump(runs, f)

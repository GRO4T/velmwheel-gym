import pickle
import sys

run_file = str(sys.argv[1])

with open(run_file, "rb") as f:
    runs = pickle.load(f)


def calculate_avg_linear_speed(runs):
    linear_speeds = []
    for run in runs:
        for vx, vy in run["velocities"]:
            linear_speeds.append((vx**2 + vy**2) ** 0.5)
    return sum(linear_speeds) / len(linear_speeds)


def calculate_avg_rotational_speed(runs):
    rotational_speeds = []
    for run in runs:
        for _, _, w in run["actions"]:
            rotational_speeds.append(abs(w))
    return sum(rotational_speeds) / len(rotational_speeds)


def calculate_avg_path_length(runs):
    path_lengths = []
    for run in runs:
        path_length = 0.0
        prev_pos = run["footprints"]["positions"][0]
        for pos in run["footprints"]["positions"][1:]:
            import math

            path_length += math.dist(prev_pos, pos)
            prev_pos = pos
        path_lengths.append(path_length)
    return sum(path_lengths) / len(path_lengths)


success_rate = 0.0
collision_rate = 0.0
timeout_rate = 0.0
avg_linear_speed = 0.0
avg_rotational_speed = 0.0
avg_path_length = 0.0
avg_time = 0.0
avg_min_goal_dist = 0.0

success_rate = sum([run["success"] for run in runs]) / len(runs)
collision_rate = sum([1 for run in runs if run["collided_at"] is not None]) / len(runs)
timeout_rate = sum([1 for run in runs if run["timed_out_at"] is not None]) / len(runs)
avg_linear_speed = calculate_avg_linear_speed(runs)
avg_rotational_speed = calculate_avg_rotational_speed(runs)
avg_path_length = calculate_avg_path_length(runs)
avg_min_goal_dist = sum([run["min_goal_dist"] for run in runs]) / len(runs)

success_rate = round(success_rate, 2)
collision_rate = round(collision_rate, 2)
timeout_rate = round(timeout_rate, 2)
avg_linear_speed = round(avg_linear_speed, 2)
avg_rotational_speed = round(avg_rotational_speed, 2)
avg_path_length = round(avg_path_length, 2)
avg_min_goal_dist = round(avg_min_goal_dist, 2)

print(f"Success rate: {success_rate}")
print(f"Collision rate: {collision_rate}")
print(f"Timeout rate: {timeout_rate}")
print(f"Avg. linear speed: {avg_linear_speed}")
print(f"Avg. rotational speed: {avg_rotational_speed}")
print(f"Avg. path length: {avg_path_length}")
print(f"Avg. min goal dist: {avg_min_goal_dist}")

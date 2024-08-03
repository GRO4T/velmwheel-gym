import numpy as np


def angle_between_robot_and_goal(robot_pos, goal_pos, theta) -> float:
    x_r, y_r = robot_pos
    x_g, y_g = goal_pos
    # Robot's heading vector
    R = np.array([np.cos(theta), np.sin(theta)])
    # Goal direction vector
    G = np.array([x_g - x_r, y_g - y_r])
    G = G / np.linalg.norm(G)
    dot_product = np.dot(R, G)
    return np.arccos(dot_product)


def interpolate_coordinates(x1, y1, x2, y2, t):
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)
    return x, y

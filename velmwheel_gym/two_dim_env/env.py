""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar-env/blob/main/gym_robot2d/envs/robot2d_env.py """

import math

import gymnasium as gym
import numpy as np

from velmwheel_gym.constants import (
    COORDINATES_NORMALIZATION_FACTOR,
    GLOBAL_GUIDANCE_OBSERVATION_POINTS,
    LIDAR_DATA_SIZE,
)
from velmwheel_gym.types import Point

from .robot2d import Robot2D

ACTION_NORMALIZATION_FACTOR = 5.0


class Robot2dEnv(gym.Env):
    def __init__(
        self,
        dT=0.05,
        is_goal=True,
        eps_err=0.4,
        max_action_magnitude=5,
        **kwargs,
    ):
        super().__init__()

        # Initialize variables
        self.state = None
        self.viewer = None
        self.is_goal = is_goal
        self.dT = dT

        self.robot = Robot2D(dT=self.dT, is_render=True, is_goal=self.is_goal)
        self.eps_err = eps_err
        self.steps = 0

        self.final_state = 0
        self.max_action_magnitude = max_action_magnitude

        self.robot_goal = np.array([0.0, 0.0])

        self.xr0 = 0
        self.yr0 = 0
        self.thr0 = 0

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float64,
        )

        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS + LIDAR_DATA_SIZE,),
            dtype=np.float64,
        )

    @property
    def max_episode_steps(self) -> int:
        return self._time_limit_max_episode_steps

    @property
    def goal(self) -> Point:
        return Point(*self.robot_goal)

    @property
    def robot_position(self) -> Point:
        return Point(self.robot.xr, self.robot.yr)

    @property
    def starting_position(self) -> Point:
        return Point(self.xr0, self.yr0)

    def _observe(self, robot_pos):
        self.robot.scanning()

        xls = self.robot.xls
        yls = self.robot.yls

        if xls.size == 0:
            xls = np.array([self.robot.max_range])
            yls = np.array([0.0])

        # convert LIDAR touches to ranges
        ranges = []
        for xl, yl in zip(xls, yls):
            ranges.append(math.dist(self.robot_position, (xl, yl)))
        # clamp and normalize LIDAR ranges
        ranges = [min(r, self.robot.max_range) / self.robot.max_range for r in ranges]

        position_normalized = robot_pos / COORDINATES_NORMALIZATION_FACTOR
        goal_normalized = self.robot_goal / COORDINATES_NORMALIZATION_FACTOR
        empty_global_guidace = np.zeros(
            2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS
        )  # NOTE: At the moment global planner is not implemented

        return np.concatenate(
            (position_normalized, goal_normalized, empty_global_guidace, ranges)
        )

    def step(self, action):
        # State Update
        vx = ACTION_NORMALIZATION_FACTOR * action[0]
        vy = ACTION_NORMALIZATION_FACTOR * action[1]
        w = 0.0

        # Clip actions
        vx = np.clip(vx, -self.max_action_magnitude, self.max_action_magnitude)
        vy = np.clip(vy, -self.max_action_magnitude, self.max_action_magnitude)
        w = np.clip(w, -self.max_action_magnitude, self.max_action_magnitude)

        self.robot.step(vx, vy, w)

        # Transform robot pose in t wrt robot initial frame
        I_T_r = pose2mat_2d(self.robot.xr, self.robot.yr, self.robot.thr)
        I_T_r0 = pose2mat_2d(self.xr0, self.yr0, self.thr0)
        r0_T_I = inverse_mat_2d(I_T_r0)
        r0_T_r = r0_T_I.dot(I_T_r)
        xr_r0, yr_r0, thr_r0 = mat2pose_2d(r0_T_r)

        robot_pos = np.array([xr_r0, yr_r0])

        r_T_I = inverse_mat_2d(I_T_r)

        # Check if robot is crashed
        is_crashed = self.robot.is_crashed()

        ## Transform goal position to robot inertia frame
        I_T_g = pose2mat_2d(self.robot.xg, self.robot.yg, self.robot.thg)
        r_T_g = r_T_I.dot(I_T_g)
        xg_r, yg_r, _ = mat2pose_2d(r_T_g)

        ## Notice we do not use from Transformation because goal does not have orientation
        thg_r = np.arctan2(yg_r, xg_r)

        self.robot_goal = np.array([xg_r, yg_r])

        fov = np.pi / 2
        lt = 0.6  # 2.

        th_p = fov / 2
        th_m = -fov / 2

        is_success = False

        if (thg_r > th_m) and (thg_r < th_p) and (np.linalg.norm([yg_r, xg_r]) < lt):
            is_success = True
            # self.steps = 0
            # self.robot.set_random_goal()

        # Check if robot does not move
        # print('thg_r:{} , xg_r: {/}, yg_r:{} '.format(thg_r, xg_r,yg_r))

        # Rewards
        r_collide = -1.0
        r_success = 0.9
        goal_closeness_factor = (
            1
            - math.dist(self.robot_position, self.robot_goal)
            / COORDINATES_NORMALIZATION_FACTOR
        )
        r_alive = goal_closeness_factor * 0.1 / self.max_episode_steps
        r_nav = r_alive

        # Done condition
        terminated = bool(is_crashed or is_success)
        if not terminated:
            self.steps += 1
            # Sparse reward exploration
            reward = r_nav
        else:
            if is_crashed:
                self.final_state = 1
                reward = r_collide
            elif is_success:
                reward = r_success
                self.final_state = 2
                is_success = False
            else:
                reward = r_nav
                self.final_state = 3

        obs = self._observe(robot_pos)

        return obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):  # Return to initial state
        self.robot.reset()

        self.steps = 0
        self.final_state = 0

        # Save initial pose

        self.xr0 = self.robot.xr
        self.yr0 = self.robot.yr
        self.thr0 = self.robot.thr

        # Transform robot pose in t wrt robot initial frame
        I_T_r = pose2mat_2d(self.robot.xr, self.robot.yr, self.robot.thr)
        I_T_r0 = pose2mat_2d(self.xr0, self.yr0, self.thr0)
        r0_T_I = inverse_mat_2d(I_T_r0)
        r0_T_r = r0_T_I.dot(I_T_r)
        xr_r0, yr_r0, thr_r0 = mat2pose_2d(r0_T_r)

        robot_pos = np.array([xr_r0, yr_r0])

        r_T_I = inverse_mat_2d(I_T_r)

        # Transform goal position to robot inertia frame
        I_T_g = pose2mat_2d(self.robot.xg, self.robot.yg, self.robot.thg)
        r_T_g = r_T_I.dot(I_T_g)
        xg_r, yg_r, thg_r = mat2pose_2d(r_T_g)

        self.robot_goal = np.array([xg_r, yg_r])

        return self._observe(robot_pos), {}

    def render(self, mode="human"):
        self.robot.render()

    def close(self):
        self.robot.close()


def pose2mat_2d(x, y, th):
    T = np.array([[np.cos(th), -np.sin(th), x], [np.sin(th), np.cos(th), y], [0, 0, 1]])
    return T


def mat2pose_2d(T):
    th = np.arctan2(T[1, 0], T[0, 0])
    x = T[0, 2]
    y = T[1, 2]
    return x, y, th


def inverse_mat_2d(T):
    """
    Calculates the inverse of a 2D Transformation Matrix (3x3).
    """
    R_in = T[:2, :2]
    t_in = T[:2, [-1]]
    R_out = R_in.T
    t_out = -np.matmul(R_out, t_in)
    return np.vstack((np.hstack((R_out, t_out)), np.array([0, 0, 1])))

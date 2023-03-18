import gym

from envs.velmwheel import VelmwheelEnv

import time

env = gym.make("Velmwheel-v0")

env.reset()

while True:
    print("Moving backward")
    env.step(VelmwheelEnv.Actions.BACKWARD)
# print("Moving forward")
# for i in range(20):
#     env.step(VelmwheelEnv.Actions.FORWARD)
#     time.sleep(1)
# print("Moving left")
# env.step(VelmwheelEnv.Actions.LEFT)
# time.sleep(10)
# print("Moving right")
# env.step(VelmwheelEnv.Actions.RIGHT)
# time.sleep(3)

env.close()

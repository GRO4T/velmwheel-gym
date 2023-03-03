import gym

from envs.velmwheel import VelmwheelEnv

import time

env = gym.make("Velmwheel-v0")

env.reset()

print("Moving forward")
env.step(VelmwheelEnv.Actions.FORWARD)
time.sleep(3)
print("Moving backward")
env.step(VelmwheelEnv.Actions.BACKWARD)
time.sleep(3)
print("Moving left")
env.step(VelmwheelEnv.Actions.LEFT)
time.sleep(3)
print("Moving right")
env.step(VelmwheelEnv.Actions.RIGHT)
time.sleep(3)

env.close()
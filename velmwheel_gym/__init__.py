from gym.envs.registration import register

register(
    id="Velmwheel10-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=10,
)

register(
    id="Velmwheel25-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=25,
)

register(
    id="Velmwheel50-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=50,
)

register(
    id="Velmwheel100-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=100,
)

register(
    id="Velmwheel250-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=250,
)

register(
    id="Velmwheel500-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=500,
)

register(
    id="Velmwheel1000-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=1000,
)

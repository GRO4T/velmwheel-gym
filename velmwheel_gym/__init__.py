from gym.envs.registration import register

register(
    id="Velmwheel25-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=50,
)

register(
    id="Velmwheel50-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=50,
)

register(
    id="Velmwheel500-v1",
    entry_point="velmwheel_gym.env:VelmwheelEnv",
    max_episode_steps=500,
)

from gym.envs.registration import register

register(
    id="Velmwheel-v0",
    entry_point="velmwheel_gym.envs.velmwheel:VelmwheelEnv",
    max_episode_steps=50,
)

register(
    id="Velmwheel-v2",
    entry_point="velmwheel_gym.envs.velmwheel_v2:VelmwheelEnvV2",
    max_episode_steps=50,
)

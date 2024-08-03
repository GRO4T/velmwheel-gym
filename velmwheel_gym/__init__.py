from gymnasium.envs.registration import register

from velmwheel_gym.max_episode_steps_injection_wrapper import (
    MaxEpisodeStepsInjectionWrapper,
)

register(
    id="Velmwheel50-v0",
    entry_point="velmwheel_gym.gazebo_env.env:VelmwheelEnv",
    max_episode_steps=50,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel100-v0",
    entry_point="velmwheel_gym.gazebo_env.env:VelmwheelEnv",
    max_episode_steps=100,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel250-v0",
    entry_point="velmwheel_gym.gazebo_env.env:VelmwheelEnv",
    max_episode_steps=250,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel500-v0",
    entry_point="velmwheel_gym.gazebo_env.env:VelmwheelEnv",
    max_episode_steps=500,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel1000-v0",
    entry_point="velmwheel_gym.gazebo_env.env:VelmwheelEnv",
    max_episode_steps=1000,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel2000-v0",
    entry_point="velmwheel_gym.gazebo_env.env:VelmwheelEnv",
    max_episode_steps=2000,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel2D100-v0",
    entry_point="velmwheel_gym.two_dim_env.env:Robot2dEnv",
    max_episode_steps=100,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel2D250-v0",
    entry_point="velmwheel_gym.two_dim_env.env:Robot2dEnv",
    max_episode_steps=250,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel2D500-v0",
    entry_point="velmwheel_gym.two_dim_env.env:Robot2dEnv",
    max_episode_steps=500,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

register(
    id="Velmwheel2D1000-v0",
    entry_point="velmwheel_gym.two_dim_env.env:Robot2dEnv",
    max_episode_steps=1000,
    additional_wrappers=(MaxEpisodeStepsInjectionWrapper.wrapper_spec(),),
)

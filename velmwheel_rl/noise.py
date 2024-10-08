import copy
import logging
from typing import Optional

import numpy as np
from numpy.typing import DTypeLike
from stable_baselines3.common.noise import ActionNoise

logger = logging.getLogger(__name__)


class OrnsteinUhlenbeckActionNoiseWithDecay(ActionNoise):
    """
    An Ornstein Uhlenbeck action noise, this is designed to approximate Brownian motion with friction.

    Based on http://math.stackexchange.com/questions/1287634/implementing-ornstein-uhlenbeck-in-matlab

    :param mean: Mean of the noise
    :param sigma: Scale of the noise
    :param theta: Rate of mean reversion
    :param dt: Timestep for the noise
    :param initial_noise: Initial value for the noise output, (if None: 0)
    :param dtype: Type of the output noise
    """

    # pylint: disable=too-many-arguments
    def __init__(
        self,
        mean: np.ndarray,
        sigma: np.ndarray,
        target_sigma: np.ndarray,
        theta: float = 0.15,
        dt: float = 1e-2,
        initial_noise: Optional[np.ndarray] = None,
        dtype: DTypeLike = np.float32,
        decay_steps: int = 5e5,
    ) -> None:
        self._theta = theta
        self._mu = mean
        self._sigma = sigma
        self._initial_sigma = copy.deepcopy(sigma)
        self._target_sigma = copy.deepcopy(target_sigma)
        self._dt = dt
        self._dtype = dtype
        self._decay_steps = decay_steps
        self._calls = 0
        self.initial_noise = initial_noise
        self.noise_prev = np.zeros_like(self._mu)
        self.reset()
        super().__init__()

    def __call__(self) -> np.ndarray:
        noise = (
            self.noise_prev
            + self._theta * (self._mu - self.noise_prev) * self._dt
            + self._sigma * np.sqrt(self._dt) * np.random.normal(size=self._mu.shape)
        )

        self._calls += 1
        t = min(1.0, self._calls / self._decay_steps)
        self._sigma = self._initial_sigma * (1 - t) + self._target_sigma * t

        self.noise_prev = noise
        return noise.astype(self._dtype)

    def reset(self) -> None:
        """
        reset the Ornstein Uhlenbeck noise, to the initial position
        """
        self.noise_prev = (
            self.initial_noise
            if self.initial_noise is not None
            else np.zeros_like(self._mu)
        )

    def __repr__(self) -> str:
        return f"OrnsteinUhlenbeckActionNoise(mu={self._mu}, sigma={self._sigma})"

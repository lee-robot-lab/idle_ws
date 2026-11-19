from __future__ import annotations

from types import SimpleNamespace

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
    from gymnasium.envs.registration import register

    GYMNASIUM_AVAILABLE = True
except ModuleNotFoundError:
    GYMNASIUM_AVAILABLE = False

    class Env:
        metadata: dict = {}

        def __init__(self) -> None:
            self.np_random = np.random.default_rng()

        def reset(self, *, seed: int | None = None, options: dict | None = None):
            del options
            if seed is not None:
                self.np_random = np.random.default_rng(seed)

    class Box:
        def __init__(self, low, high, shape=None, dtype=np.float32):
            self.dtype = np.dtype(dtype)
            if shape is None:
                shape = np.asarray(low, dtype=self.dtype).shape
            self.shape = tuple(shape)
            self.low = np.broadcast_to(np.asarray(low, dtype=self.dtype), self.shape).copy()
            self.high = np.broadcast_to(np.asarray(high, dtype=self.dtype), self.shape).copy()

        def sample(self):
            low_finite = np.isfinite(self.low)
            high_finite = np.isfinite(self.high)
            sample = np.zeros(self.shape, dtype=np.float64)
            bounded = low_finite & high_finite
            sample[bounded] = np.random.uniform(self.low[bounded], self.high[bounded])
            lower_only = low_finite & ~high_finite
            sample[lower_only] = self.low[lower_only] + np.random.exponential(size=np.count_nonzero(lower_only))
            upper_only = ~low_finite & high_finite
            sample[upper_only] = self.high[upper_only] - np.random.exponential(size=np.count_nonzero(upper_only))
            unbounded = ~low_finite & ~high_finite
            sample[unbounded] = np.random.normal(size=np.count_nonzero(unbounded))
            return sample.astype(self.dtype)

        def contains(self, x) -> bool:
            arr = np.asarray(x, dtype=self.dtype)
            if arr.shape != self.shape:
                return False
            return bool(np.all(arr >= self.low) and np.all(arr <= self.high))

    class Dict:
        def __init__(self, spaces_dict: dict):
            self.spaces = dict(spaces_dict)

        def sample(self):
            return {key: space.sample() for key, space in self.spaces.items()}

        def contains(self, x) -> bool:
            if not isinstance(x, dict):
                return False
            if set(x.keys()) != set(self.spaces.keys()):
                return False
            return all(space.contains(x[key]) for key, space in self.spaces.items())

    spaces = SimpleNamespace(Box=Box, Dict=Dict)

    def register(*args, **kwargs):
        del args, kwargs
        return None

    gym = SimpleNamespace(Env=Env, spaces=spaces, register=register)

"""
Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
License: BSD 3-Clause. See LICENSE.md file in root directory.
"""

import numpy as np


class Sampler(object):
    def __init__(self, extents):
        """Construct a sampler for the half-open interval defined by extents.

        Args:
            extents: np.array of lower and upper bounds with shape D x 2
        """
        self.extents = extents
        self.dim = extents.shape[0]

    def sample(self, num_samples):
        """Return samples from the sampler.

        Args:
            num_samples: number of samples to return

        Returns:
            samples: np.array of N x D sample configurations
        """
        raise NotImplementedError


class RandomSampler(Sampler):
    def sample(self, num_samples):
        """Return samples from the random sampler.

        Args:
            num_samples: number of samples to return

        Returns:
            samples: np.array of N x D sample configurations
        """
        return np.random.uniform(
            self.extents[:, 0],
            self.extents[:, 1],
            size=(num_samples, self.dim),
        )


sampler_selection = {
    "random": RandomSampler,
}

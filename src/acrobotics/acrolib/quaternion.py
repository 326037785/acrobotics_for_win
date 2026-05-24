"""Quaternion extension used by the path sampling code."""

import numpy as np
import pyquaternion


class Quaternion(pyquaternion.Quaternion):
    def random_near(self, distance):
        if distance > 0.25 * np.pi:
            return Quaternion.random()

        amount = np.random.uniform()
        axis = np.random.normal(size=3)
        delta = Quaternion(axis=axis, angle=2 * amount ** (1 / 3) * distance)
        return self * delta

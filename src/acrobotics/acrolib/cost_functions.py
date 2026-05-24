"""Pairwise configuration costs used by sampling-based planning."""

import numpy as np


def _differences(first, second):
    return np.asarray(first)[:, None, :] - np.asarray(second)[None, :, :]


def norm_l1(first, second):
    return np.abs(_differences(first, second)).sum(axis=2)


def norm_l2(first, second):
    return np.linalg.norm(_differences(first, second), axis=2)


def sum_squared(first, second):
    differences = _differences(first, second)
    return (differences**2).sum(axis=2)


def weighted_sum_squared(first, second, weights):
    differences = _differences(first, second)
    return ((differences**2) * np.asarray(weights)).sum(axis=2)


def norm_infinity(first, second):
    return np.abs(_differences(first, second)).max(axis=2)

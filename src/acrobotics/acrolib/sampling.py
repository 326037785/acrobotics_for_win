"""Sampling helpers for path tolerances."""

from enum import Enum

import numpy as np


class SampleMethod(Enum):
    random_uniform = 0
    deterministic_uniform = 1


class Sampler:
    def __init__(self):
        self.halton_sampler = None

    def sample(self, num_samples, sample_dim, method):
        if method == SampleMethod.random_uniform:
            return np.random.rand(num_samples, sample_dim)
        if method == SampleMethod.deterministic_uniform:
            if self.halton_sampler is None:
                self.halton_sampler = HaltonSampler(sample_dim)
            assert self.halton_sampler.dim == sample_dim
            return self.halton_sampler.get_samples(num_samples)
        raise NotImplementedError(f"Unkown sampling method: {method}")


def vdc(n, base=2):
    value, denominator = 0, 1
    while n:
        denominator *= base
        n, remainder = divmod(n, base)
        value += remainder / denominator
    return value


def next_prime():
    def is_prime(value):
        return all(value % divisor for divisor in range(2, int(value**0.5) + 1))

    value = 3
    while True:
        if is_prime(value):
            yield value
        value += 2


class HaltonSampler:
    def __init__(self, dim):
        self.dim = dim
        primes = next_prime()
        self.primes = [next(primes) for _ in range(dim)]
        self.count = 1

    def get_samples(self, number):
        samples = [
            [vdc(i, base) for i in range(self.count, self.count + number)]
            for base in self.primes
        ]
        self.count += number
        return np.array(samples).T

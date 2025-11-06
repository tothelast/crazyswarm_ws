#!/usr/bin/env python
"""Mathematical utility functions for affine transformations."""

import numpy as np


def compute_beta(t, T):
    """Smooth step function (quintic polynomial) from 0 to 1."""
    if T == 0:
        return 1.0 if t >= 0 else 0.0
    r = np.clip(t / T, 0.0, 1.0)
    return 6 * r**5 - 15 * r**4 + 10 * r**3


def rotation_matrix_3d_z(theta):
    """Creates a 3D rotation matrix for rotation around the Z-axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])


def numerical_diff(times, data):
    """Calculates the derivative of data with respect to times."""
    if len(times) < 2:
        return np.zeros_like(data)
    diff = np.gradient(data, times, axis=0, edge_order=1)
    if data.ndim == 1 and diff.ndim > 1 and diff.shape[1] == 1:
        diff = diff.squeeze()
    return diff


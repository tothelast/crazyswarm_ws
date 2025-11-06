#!/usr/bin/env python
"""Continuum deformation weight calculation utilities."""

import numpy as np


def calculate_weights(follower_initial_pos, neighbor_initial_positions):
    """
    Calculates the continuum deformation weights based on initial 2D positions.
    
    Args:
        follower_initial_pos: 3D position [x, y, z] of the follower drone
        neighbor_initial_positions: List of 3 neighbor positions [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3]]
    
    Returns:
        weights: Array of 3 weights for the weighted sum
    """
    xi, yi, _ = follower_initial_pos
    n1_pos, n2_pos, n3_pos = neighbor_initial_positions
    
    M = np.array([
        [n1_pos[0], n2_pos[0], n3_pos[0]],
        [n1_pos[1], n2_pos[1], n3_pos[1]],
        [1,         1,         1        ]
    ])
    
    b = np.array([xi, yi, 1])
    
    try:
        weights = np.linalg.solve(M, b)
        return weights
    except np.linalg.LinAlgError:
        return np.array([1/3, 1/3, 1/3])


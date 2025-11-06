#!/usr/bin/env python
"""Trajectory generation for leader drones using affine transformations."""

import numpy as np
from collections import defaultdict
from .math_utils import compute_beta, rotation_matrix_3d_z, numerical_diff


def generate_leader_trajectories(leader_names, initial_positions, mode_params, dt):
    """
    Generates 3D trajectories for leader drones using affine transformations.
    
    Args:
        leader_names: List of leader drone names
        initial_positions: Dictionary mapping drone names to initial positions
        mode_params: Dictionary containing transformation parameters (L1, L2, TH, PS, D1, D2, D3, T)
        dt: Time step for trajectory generation
    
    Returns:
        Dictionary mapping leader names to trajectory data (time, pos, vel, acc, yaw, omega)
    """
    num_modes = len(mode_params['T'])
    expected_len = num_modes + 1
    
    # Verify parameter list lengths
    for key, value in mode_params.items():
        if key == 'T':
            if len(value) != num_modes:
                raise ValueError(f"Length of T ({len(value)}) should be {num_modes}")
        elif len(value) != expected_len:
            raise ValueError(f"Length of {key} ({len(value)}) should be {expected_len}")

    leader_init_pos_list = [np.array(initial_positions[name]) for name in leader_names]
    initial_centroid = np.mean(leader_init_pos_list, axis=0)
    
    # Define canonical vectors based on initial relative positions
    a_vectors = {name: pos - initial_centroid for name, pos in zip(leader_names, leader_init_pos_list)}
    
    temp_trajectories = defaultdict(lambda: {'pos': [], 'yaw': []})
    trajectory_times = []
    
    current_time = 0.0
    last_centroid_offset = np.zeros(3)
    
    # Add initial state at t=0
    trajectory_times.append(current_time)
    for name in leader_names:
        temp_trajectories[name]['pos'].append(list(leader_init_pos_list[leader_names.index(name)]))
        temp_trajectories[name]['yaw'].append(mode_params['TH'][0])

    for mode_idx in range(num_modes):
        t_start_mode = current_time
        T_mode = mode_params['T'][mode_idx]
        if T_mode <= 0:
            last_centroid_offset = np.array([mode_params['D1'][mode_idx+1], mode_params['D2'][mode_idx+1], mode_params['D3'][mode_idx+1]])
            continue

        num_steps = max(1, int(np.ceil(T_mode / dt)))
        
        # Get parameters for the start and end of this mode
        l10, l1f = mode_params['L1'][mode_idx], mode_params['L1'][mode_idx+1]
        l20, l2f = mode_params['L2'][mode_idx], mode_params['L2'][mode_idx+1]
        th0, thf = mode_params['TH'][mode_idx], mode_params['TH'][mode_idx+1]
        sig0, sigf = mode_params['PS'][mode_idx], mode_params['PS'][mode_idx+1]
        
        dx0_off, dxf_off = mode_params['D1'][mode_idx], mode_params['D1'][mode_idx+1]
        dy0_off, dyf_off = mode_params['D2'][mode_idx], mode_params['D2'][mode_idx+1]
        dz0_off, dzf_off = mode_params['D3'][mode_idx], mode_params['D3'][mode_idx+1]

        start_offset = last_centroid_offset
        target_offset = np.array([dxf_off, dyf_off, dzf_off])

        for i in range(1, num_steps + 1):
            t_mode = min(i * dt, T_mode)
            current_time = t_start_mode + t_mode
            beta = compute_beta(t_mode, T_mode)

            # Interpolate parameters
            l1 = (1 - beta) * l10 + beta * l1f
            l2 = (1 - beta) * l20 + beta * l2f
            th = (1 - beta) * th0 + beta * thf
            sig = (1 - beta) * sig0 + beta * sigf
            
            # Interpolate centroid offset
            current_offset = (1 - beta) * start_offset + beta * target_offset
            
            # Calculate transformation components
            R1 = rotation_matrix_3d_z(th)
            R2 = rotation_matrix_3d_z(sig)
            L = np.diag([l1, l2, 1.0])

            Q = R1 @ R2 @ L @ R2.T
            d = initial_centroid + current_offset

            if not trajectory_times or not np.isclose(current_time, trajectory_times[-1]):
                trajectory_times.append(current_time)
                for name in leader_names:
                    a_vec = a_vectors[name]
                    pos = Q @ a_vec + d
                    temp_trajectories[name]['pos'].append(pos.tolist())
                    temp_trajectories[name]['yaw'].append(th)
            else:
                list_index = -1
                for name in leader_names:
                    a_vec = a_vectors[name]
                    pos = Q @ a_vec + d
                    if len(temp_trajectories[name]['pos']) > 0:
                        temp_trajectories[name]['pos'][list_index] = pos.tolist()
                    else:
                        temp_trajectories[name]['pos'].append(pos.tolist())
                    if len(temp_trajectories[name]['yaw']) > 0:
                        temp_trajectories[name]['yaw'][list_index] = th
                    else:
                        temp_trajectories[name]['yaw'].append(th)

            if np.isclose(t_mode, T_mode) and i < num_steps:
                break

        last_centroid_offset = target_offset

    # Post-process: Calculate Velocities, Accelerations, Omega
    times = np.array(trajectory_times)
    final_trajectories = {}

    unique_indices = np.unique(times, return_index=True)[1]
    unique_indices = np.sort(unique_indices)
    times = times[unique_indices]

    for name in leader_names:
        positions = np.array(temp_trajectories[name]['pos'])[unique_indices]
        yaws = np.array(temp_trajectories[name]['yaw'])[unique_indices]
        
        if len(times) < 2:
            velocities = np.zeros_like(positions)
            accelerations = np.zeros_like(positions)
            omegas = np.zeros_like(positions)
        else:
            velocities = numerical_diff(times, positions)
            accelerations = numerical_diff(times, velocities)
            yaws_unwrapped = np.unwrap(yaws)
            omega_z = numerical_diff(times, yaws_unwrapped)
            omegas = np.zeros((len(times), 3))
            omegas[:, 2] = omega_z

        final_trajectories[name] = {
            'time': times,
            'pos': positions,
            'vel': velocities,
            'acc': accelerations,
            'yaw': yaws,
            'omega': omegas
        }

    return final_trajectories


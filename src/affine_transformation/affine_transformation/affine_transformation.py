
#!/usr/bin/env python
"""Affine transformation + continuum deformation flight control for Crazyflie drones."""

import numpy as np
import csv
from datetime import datetime

from crazyflie_py import Crazyswarm
from .utils import (
    generate_leader_trajectories,
    calculate_weights,
    execute_takeoff,
    move_to_hover_positions,
    stabilize_positions,
    execute_landing
)

def main():
    """Main control loop for affine transformation flight."""
    swarm = Crazyswarm()
    logger = swarm.allcfs.get_logger()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    logger.info("=" * 80)
    logger.info("AFFINE TRANSFORMATION + CONTINUUM DEFORMATION FLIGHT")
    logger.info("=" * 80)

    # Define the neighbor topology for each follower
    follower_neighbors_map = {
        'cf3': ['cf1', 'cf2', 'cf5'],
        'cf4': ['cf2', 'cf5', 'cf6'],
        'cf5': ['cf1', 'cf2', 'cf6']
    }

    # Main leaders defining the overall transformation
    main_leader_names = ['cf1', 'cf2', 'cf6']
    main_leaders = {name: allcfs.crazyfliesByName[name] for name in main_leader_names}

    # Followers acquiring the transformation
    follower_names = list(follower_neighbors_map.keys())
    followers = {name: allcfs.crazyfliesByName[name] for name in follower_names}

    logger.info(f"Configuration: {len(main_leader_names)} leaders, {len(follower_names)} followers")

    # Get Initial Positions
    initial_positions = {name: np.array(cf.initialPosition) for name, cf in allcfs.crazyfliesByName.items()}

    # Calculate Continuum Deformation Weights
    follower_weights = {}
    for follower_name in follower_names:
        neighbor_names = follower_neighbors_map[follower_name]
        if len(neighbor_names) != 3:
            raise ValueError(f"Follower {follower_name} must have exactly 3 neighbors.")

        follower_init_pos = initial_positions[follower_name]
        neighbor_init_pos = [initial_positions[n_name] for n_name in neighbor_names]

        weights = calculate_weights(follower_init_pos, neighbor_init_pos)
        follower_weights[follower_name] = weights

    # Define Affine Transformation Modes for Leaders
    rate_hz = 50
    dt = 1.0 / rate_hz
    TAKEOFF_HEIGHT = 0.75

    # Parameters for the transformation
    mode_params = {
        'L1': [1.0, 0.5, 0.5, 0.6],
        'L2': [1.0, 0.5, 0.5, 0.9],
        'TH': [0.0, 0.0, 0.0, 0.5],
        'PS': [0.0, 0.0, 0.0, 0.25],
        'D1': [0.0, 0.0, 2.0, 2.0],
        'D2': [0.0, 0.0, 0.0, 0.0],
        'D3': [0.0, 0.0, 0.0, 0.0],
        'T':  [5.0, 20.0, 5.0]
    }

    logger.info("Generating leader trajectories...")
    leader_trajectories = generate_leader_trajectories(
        main_leader_names,
        initial_positions,
        mode_params,
        dt
    )
    total_traj_duration = sum(mode_params['T'])
    logger.info(f"Trajectory duration: {total_traj_duration}s")

    # Get time array (same for all leaders)
    leader_time_array = leader_trajectories[main_leader_names[0]]['time']

    # Take off all drones
    execute_takeoff(allcfs, timeHelper, logger, TAKEOFF_HEIGHT)

    # Move all drones to initial hover positions using streaming commands
    initial_hover_positions = move_to_hover_positions(
        allcfs, timeHelper, logger, initial_positions, TAKEOFF_HEIGHT, duration=2.0, rate_hz=rate_hz
    )

    # Control gains for feedback (tuned for aggressive tracking)
    # Leaders: Track pre-computed trajectory with moderate gains
    K_p_leader = np.array([2.5, 2.5, 4.0])  # [x, y, z] - higher z for gravity
    K_d_leader = np.array([1.5, 1.5, 2.0])  # [x, y, z] - higher damping in z

    # Followers: Track moving targets (leaders) - need higher gains
    K_p_follower = np.array([3.5, 3.5, 5.0])  # [x, y, z] - aggressive tracking
    K_d_follower = np.array([2.0, 2.0, 2.5])  # [x, y, z] - prevent oscillations

    # Tracking error statistics
    max_errors = {'leaders': [], 'followers': []}
    avg_errors = {'leaders': [], 'followers': []}

    # CSV data logging for validation
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"logs/experiment_data_{timestamp}.csv"
    csv_file = open(csv_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)

    # CSV header
    header = ['time', 'iteration']
    for name in main_leaders.keys():
        header.extend([f'{name}_x_des', f'{name}_y_des', f'{name}_z_des',
                      f'{name}_x_act', f'{name}_y_act', f'{name}_z_act'])
    for name in followers.keys():
        header.extend([f'{name}_x_des', f'{name}_y_des', f'{name}_z_des',
                      f'{name}_x_act', f'{name}_y_act', f'{name}_z_act'])
    csv_writer.writerow(header)

    logger.info(f"Logging data to: {csv_filename}")

    # Main Control Loop
    logger.info("=" * 80)
    logger.info("STARTING DECENTRALIZED FEEDBACK CONTROL LOOP")
    logger.info("=" * 80)
    logger.info(f"Duration: {total_traj_duration}s, Rate: {rate_hz} Hz")
    logger.info(f"Leader gains: K_p={K_p_leader}, K_d={K_d_leader}")
    logger.info(f"Follower gains: K_p={K_p_follower}, K_d={K_d_follower}")

    start_time = timeHelper.time()
    dt = 1.0 / rate_hz

    # Initialize previous actual positions for velocity estimation
    prev_actual_positions = {}
    for name, cf in allcfs.crazyfliesByName.items():
        prev_actual_positions[name] = np.array(cf.get_position())

    loop_iteration = 0
    last_log_time = start_time

    while timeHelper.time() - start_time < total_traj_duration:
        if timeHelper.isShutdown():
            logger.warn("Shutdown requested!")
            break

        current_wall_time = timeHelper.time()
        elapsed_time = current_wall_time - start_time
        t = np.clip(elapsed_time, 0.0, leader_time_array[-1])

        # Log progress every 5 seconds
        if current_wall_time - last_log_time >= 5.0:
            progress_pct = (elapsed_time / total_traj_duration) * 100
            logger.info(f"CONTROL LOOP Progress: {progress_pct:.1f}% ({elapsed_time:.1f}/{total_traj_duration:.1f}s) - Iteration {loop_iteration}")
            last_log_time = current_wall_time

        # Read actual positions from Vicon for all drones
        actual_positions = {}
        actual_velocities = {}
        for name, cf in allcfs.crazyfliesByName.items():
            pos_actual = np.array(cf.get_position())
            actual_positions[name] = pos_actual
            # Estimate velocity via finite difference
            vel_actual = (pos_actual - prev_actual_positions[name]) / dt
            actual_velocities[name] = vel_actual

        # LEADERS: Decentralized feedback control around affine transformation trajectory
        leader_errors = []
        for name, leader_drone in main_leaders.items():
            traj_data = leader_trajectories[name]
            t_interp = np.clip(t, traj_data['time'][0], traj_data['time'][-1])

            # Desired state from pre-computed trajectory
            pos_desired = np.array([np.interp(t_interp, traj_data['time'], traj_data['pos'][:, i]) for i in range(3)])
            vel_desired = np.array([np.interp(t_interp, traj_data['time'], traj_data['vel'][:, i]) for i in range(3)])
            acc_feedforward = np.array([np.interp(t_interp, traj_data['time'], traj_data['acc'][:, i]) for i in range(3)])
            yaw = np.interp(t_interp, traj_data['time'], traj_data['yaw'])
            omega = np.array([np.interp(t_interp, traj_data['time'], traj_data['omega'][:, i]) for i in range(3)])

            pos_desired[2] += TAKEOFF_HEIGHT

            # Actual state from Vicon
            pos_actual = actual_positions[name]
            vel_actual = actual_velocities[name]

            # Feedback control law: u = -K_p*(x - x_d) - K_d*(v - v_d) + a_ff
            # Use element-wise multiplication for axis-specific gains
            pos_error = pos_actual - pos_desired
            vel_error = vel_actual - vel_desired
            acc_control = -K_p_leader * pos_error - K_d_leader * vel_error + acc_feedforward

            # Track error for logging
            leader_errors.append(np.linalg.norm(pos_error))

            # Send command with feedback control
            leader_drone.cmdFullState(pos_desired, vel_desired, acc_control, yaw, omega)

        # FOLLOWERS: Decentralized feedback control using actual neighbor positions
        # Per Equation (8) in the paper: r_i,d(t) = Σ w_i,j * r_j(t)
        # Followers use ONLY position (not velocity) of neighbors
        follower_errors = []
        for follower_name, follower_drone in followers.items():
            neighbor_names = follower_neighbors_map[follower_name]
            weights = follower_weights[follower_name]

            # Compute desired position as weighted sum of ACTUAL neighbor positions
            # This matches Equation (8): r_i,d = Σ w_i,j * r_j(t)
            pos_desired_follower = np.zeros(3)

            for i, neighbor_name in enumerate(neighbor_names):
                # Use actual positions from Vicon (decentralized!)
                neighbor_pos_actual = actual_positions[neighbor_name]
                pos_desired_follower += weights[i] * neighbor_pos_actual

            # Actual state from Vicon
            pos_actual_follower = actual_positions[follower_name]
            vel_actual_follower = actual_velocities[follower_name]

            # Feedback control law: u = -K_p*(x - x_d) - K_d*v
            # Note: vel_desired = 0 because we only have position constraint
            # Use element-wise multiplication for axis-specific gains
            pos_error_f = pos_actual_follower - pos_desired_follower
            acc_control_f = -K_p_follower * pos_error_f - K_d_follower * vel_actual_follower

            # Track error for logging
            follower_errors.append(np.linalg.norm(pos_error_f))

            # Send command with feedback control
            # vel_desired = 0 (no velocity reference, only position constraint)
            follower_drone.cmdFullState(
                pos_desired_follower,
                np.zeros(3),  # Zero velocity reference (position-only constraint)
                acc_control_f,
                0.0,
                np.zeros(3)
            )

        # Store tracking errors for statistics
        if leader_errors:
            max_errors['leaders'].append(max(leader_errors))
            avg_errors['leaders'].append(np.mean(leader_errors))
        if follower_errors:
            max_errors['followers'].append(max(follower_errors))
            avg_errors['followers'].append(np.mean(follower_errors))

        # Log data to CSV for validation
        csv_row = [t, loop_iteration]

        # Leader data
        for name in main_leaders.keys():
            traj_data = leader_trajectories[name]
            t_interp = np.clip(t, traj_data['time'][0], traj_data['time'][-1])
            pos_desired = np.array([np.interp(t_interp, traj_data['time'], traj_data['pos'][:, i]) for i in range(3)])
            pos_desired[2] += TAKEOFF_HEIGHT
            pos_actual = actual_positions[name]
            csv_row.extend([pos_desired[0], pos_desired[1], pos_desired[2],
                           pos_actual[0], pos_actual[1], pos_actual[2]])

        # Follower data
        for follower_name in followers.keys():
            neighbor_names = follower_neighbors_map[follower_name]
            weights = follower_weights[follower_name]
            pos_desired_follower = np.zeros(3)
            for i, neighbor_name in enumerate(neighbor_names):
                pos_desired_follower += weights[i] * actual_positions[neighbor_name]
            pos_actual_follower = actual_positions[follower_name]
            csv_row.extend([pos_desired_follower[0], pos_desired_follower[1], pos_desired_follower[2],
                           pos_actual_follower[0], pos_actual_follower[1], pos_actual_follower[2]])

        csv_writer.writerow(csv_row)

        # Update previous positions for next iteration
        prev_actual_positions = actual_positions.copy()
        loop_iteration += 1

        timeHelper.sleepForRate(rate_hz)

    logger.info(f"Control loop complete: {loop_iteration} iterations")

    # Close CSV file
    csv_file.close()
    logger.info(f"Data saved to: {csv_filename}")

    # Log tracking error statistics
    logger.info("=" * 80)
    logger.info("TRACKING ERROR STATISTICS")
    logger.info("=" * 80)
    if max_errors['leaders']:
        logger.info(f"Leaders - Max error: {max(max_errors['leaders'])*100:.2f} cm, Avg error: {np.mean(avg_errors['leaders'])*100:.2f} cm")
    if max_errors['followers']:
        logger.info(f"Followers - Max error: {max(max_errors['followers'])*100:.2f} cm, Avg error: {np.mean(avg_errors['followers'])*100:.2f} cm")

    # Build final positions from ACTUAL positions (not desired)
    final_positions = {}
    for name in allcfs.crazyfliesByName.keys():
        final_positions[name] = np.array(allcfs.crazyfliesByName[name].get_position())

    # Validate positions
    for name, pos in final_positions.items():
        if np.any(np.isnan(pos)) or np.any(np.isinf(pos)):
            logger.error(f"[{name}] Invalid position! Using initial hover")
            final_positions[name] = initial_hover_positions[name].copy()

    # Stabilize before landing - continue streaming cmdFullState
    stabilize_positions(allcfs, timeHelper, logger, final_positions, rate_hz)

    # Execute landing - continue streaming cmdFullState
    execute_landing(allcfs, timeHelper, logger, final_positions, rate_hz)

    logger.info("=" * 80)
    logger.info("FLIGHT COMPLETE")
    logger.info("=" * 80)

if __name__ == '__main__':
    main() 

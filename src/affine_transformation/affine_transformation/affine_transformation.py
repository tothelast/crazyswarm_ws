
#!/usr/bin/env python
"""Affine transformation + continuum deformation flight control for Crazyflie drones."""

import numpy as np

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
    rate_hz = 100
    dt = 1.0 / rate_hz
    TAKEOFF_HEIGHT = 1.0

    # Parameters for the transformation
    mode_params = {
        'L1': [1.0, 0.5, 0.5, 0.6],
        'L2': [1.0, 0.5, 0.5, 0.9],
        'TH': [0.0, 0.0, 0.0, 0.5],
        'PS': [0.0, 0.0, 0.0, 0.25],
        'D1': [0.0, 0.0, 2.0, 2.0],
        'D2': [0.0, 0.0, 0.0, 0.0],
        'D3': [0.0, 0.0, 0.0, 0.0],
        'T':  [10.0, 10.0, 10.0]
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

    # Move all drones to initial hover positions
    initial_hover_positions = move_to_hover_positions(
        allcfs, timeHelper, logger, initial_positions, TAKEOFF_HEIGHT
    )

    # Main Control Loop
    logger.info("=" * 80)
    logger.info("STARTING CONTROL LOOP")
    logger.info("=" * 80)
    logger.info(f"Duration: {total_traj_duration}s, Rate: {rate_hz} Hz")

    start_time = timeHelper.time()
    prev_follower_desired_pos = {name: initial_hover_positions[name] for name in allcfs.crazyfliesByName.keys()}
    last_loop_time = start_time
    loop_iteration = 0

    while timeHelper.time() - start_time < total_traj_duration:
        if timeHelper.isShutdown():
            logger.warn("Shutdown requested!")
            break

        current_wall_time = timeHelper.time()
        elapsed_time = current_wall_time - start_time
        t = np.clip(elapsed_time, 0.0, leader_time_array[-1])

        current_positions = {}

        # Command Leaders based on generated Affine Trajectory
        for name, leader_drone in main_leaders.items():
            traj_data = leader_trajectories[name]
            t_interp = np.clip(t, traj_data['time'][0], traj_data['time'][-1])

            pos = np.array([np.interp(t_interp, traj_data['time'], traj_data['pos'][:, i]) for i in range(3)])
            vel = np.array([np.interp(t_interp, traj_data['time'], traj_data['vel'][:, i]) for i in range(3)])
            acc = np.array([np.interp(t_interp, traj_data['time'], traj_data['acc'][:, i]) for i in range(3)])
            yaw = np.interp(t_interp, traj_data['time'], traj_data['yaw'])
            omega = np.array([np.interp(t_interp, traj_data['time'], traj_data['omega'][:, i]) for i in range(3)])

            pos[2] += TAKEOFF_HEIGHT
            leader_drone.cmdFullState(pos, vel, acc, yaw, omega)
            current_positions[name] = pos

        # Determine follower positions based on weighted sum of neighbors
        for follower_name, follower_drone in followers.items():
            neighbor_names = follower_neighbors_map[follower_name]
            weights = follower_weights[follower_name]

            local_desired_pos = np.zeros(3)

            for i, neighbor_name in enumerate(neighbor_names):
                if neighbor_name in current_positions:
                    neighbor_pos = current_positions[neighbor_name]
                else:
                    neighbor_pos = prev_follower_desired_pos.get(neighbor_name, initial_hover_positions[neighbor_name])
                local_desired_pos += weights[i] * neighbor_pos

            # Calculate velocity
            loop_dt = current_wall_time - last_loop_time
            if loop_dt <= 1e-6:
                loop_dt = 1.0 / rate_hz

            prev_pos = prev_follower_desired_pos.get(follower_name, initial_hover_positions[follower_name])
            velocity = (local_desired_pos - prev_pos) / loop_dt

            follower_drone.cmdFullState(
                local_desired_pos,
                velocity,
                np.zeros(3),
                0.0,
                np.zeros(3)
            )

            current_positions[follower_name] = local_desired_pos

        prev_follower_desired_pos = current_positions.copy()
        last_loop_time = current_wall_time
        loop_iteration += 1

        timeHelper.sleepForRate(rate_hz)

    logger.info(f"Control loop complete: {loop_iteration} iterations")

    # Build final positions for all drones
    final_positions = {}
    for name in allcfs.crazyfliesByName.keys():
        if name in prev_follower_desired_pos:
            final_positions[name] = prev_follower_desired_pos[name].copy()
        else:
            final_positions[name] = initial_hover_positions[name].copy()
            logger.warn(f"[{name}] Using initial hover position for landing")

    # Validate positions
    for name, pos in final_positions.items():
        if np.any(np.isnan(pos)) or np.any(np.isinf(pos)):
            logger.error(f"[{name}] Invalid position! Using initial hover")
            final_positions[name] = initial_hover_positions[name].copy()

    # Stabilize before landing
    stabilize_positions(allcfs, timeHelper, logger, final_positions, rate_hz)

    # Execute landing
    execute_landing(allcfs, timeHelper, logger, final_positions, rate_hz)

    logger.info("=" * 80)
    logger.info("FLIGHT COMPLETE")
    logger.info("=" * 80)

if __name__ == '__main__':
    main() 

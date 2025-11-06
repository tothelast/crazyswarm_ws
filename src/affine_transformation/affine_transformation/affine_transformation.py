
#!/usr/bin/env python

import numpy as np
import time
from collections import defaultdict

from crazyflie_py import Crazyswarm

def compute_beta(t, T):
    """Smooth step function (quintic polynomial) from 0 to 1"""
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

def generate_leader_trajectories(leader_names, initial_positions, mode_params, dt):
    """Generates 3D trajectories for leader drones using affine transformations."""
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
        num_unique_points = len(unique_indices)
        if len(temp_trajectories[name]['pos']) == num_unique_points and len(temp_trajectories[name]['yaw']) == num_unique_points:
            positions = np.array(temp_trajectories[name]['pos'])[unique_indices]
            yaws = np.array(temp_trajectories[name]['yaw'])[unique_indices]
        else:
            min_len = min(len(temp_trajectories[name]['pos']), len(temp_trajectories[name]['yaw']), num_unique_points)
            positions = np.array(temp_trajectories[name]['pos'])[:min_len]
            yaws = np.array(temp_trajectories[name]['yaw'])[:min_len]
            current_times = times[:min_len]
        
        if 'current_times' not in locals():
            current_times = times
        
        if len(current_times) < 2:
            velocities = np.zeros_like(positions)
            accelerations = np.zeros_like(positions)
            omegas = np.zeros_like(positions)
        else:
            velocities = numerical_diff(current_times, positions)
            accelerations = numerical_diff(current_times, velocities)
            yaws_unwrapped = np.unwrap(yaws)
            omega_z = numerical_diff(current_times, yaws_unwrapped)
            omegas = np.zeros((len(current_times), 3))
            omegas[:, 2] = omega_z

        final_trajectories[name] = {
            'time': current_times,
            'pos': positions,
            'vel': velocities,
            'acc': accelerations,
            'yaw': yaws,
            'omega': omegas
        }
        
        if 'current_times' in locals():
            del current_times

    return final_trajectories

def calculate_weights(follower_initial_pos, neighbor_initial_positions):
    """
    Calculates the continuum deformation weights based on initial 2D positions.
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

def main():
    swarm = Crazyswarm()
    logger = swarm.allcfs.get_logger()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    logger.info("=" * 80)
    logger.info("AFFINE TRANSFORMATION + CONTINUUM DEFORMATION FLIGHT SCRIPT")
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

    logger.info("Drone Configuration:")
    logger.info(f"  Leaders: {main_leader_names}")
    logger.info(f"  Followers: {follower_names}")
    logger.info(f"  Total drones: {len(allcfs.crazyfliesByName)}")

    # Get Initial Positions
    initial_positions = {name: np.array(cf.initialPosition) for name, cf in allcfs.crazyfliesByName.items()}
    logger.info("Initial Positions:")
    for name, pos in initial_positions.items():
        logger.info(f"  [{name}] {pos}")

    # Calculate Continuum Deformation Weights
    follower_weights = {}
    logger.info("Calculating Continuum Deformation weights...")
    
    for follower_name in follower_names:
        neighbor_names = follower_neighbors_map[follower_name]
        if len(neighbor_names) != 3:
            raise ValueError(f"Follower {follower_name} must have exactly 3 neighbors.")
            
        follower_init_pos = initial_positions[follower_name]
        neighbor_init_pos = [initial_positions[n_name] for n_name in neighbor_names]
        
        weights = calculate_weights(follower_init_pos, neighbor_init_pos)
        follower_weights[follower_name] = weights
        logger.info(f"Weights for {follower_name}: {weights}")

    # Define Affine Transformation Modes for Leaders
    rate_hz = 100
    dt = 1.0 / rate_hz
    TAKEOFF_HEIGHT = 1.0

    # Parameters for the transformation
    lmin = 0.5
    lf1 = 0.6
    lf2 = 0.9
    thf = 0.5
    psf = 0.25
    df1 = 2.0  # Reduced from 4.0 to 2.0 meters for smaller lab space
    df2 = 0.0

    mode_params = {
        'L1': [1.0, lmin, lmin, lf1],
        'L2': [1.0, lmin, lmin, lf2],
        'TH': [0.0, 0.0, 0.0, thf],
        'PS': [0.0, 0.0, 0.0, psf],
        'D1': [0.0, 0.0, df1, df1],
        'D2': [0.0, 0.0, df2, 0.0],
        'D3': [0.0, 0.0, 0.0, 0.0],
        'T':  [10.0, 10.0, 10.0]
    }

    logger.info("=" * 80)
    logger.info("GENERATING LEADER TRAJECTORIES")
    logger.info("=" * 80)
    leader_trajectories = generate_leader_trajectories(
        main_leader_names,
        initial_positions,
        mode_params,
        dt
    )
    total_traj_duration = sum(mode_params['T']) if mode_params['T'] else 0
    logger.info(f"Trajectory generation complete. Total duration: {total_traj_duration}s")

    # Get time array (same for all leaders)
    leader_time_array = leader_trajectories[main_leader_names[0]]['time']

    # Take off all drones
    logger.info("=" * 80)
    logger.info("TAKEOFF PHASE")
    logger.info("=" * 80)
    logger.info(f"Commanding all drones to takeoff to {TAKEOFF_HEIGHT}m (duration: 3.5s)...")
    allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=3.5)
    logger.info("Waiting 4.0s for takeoff to complete...")
    timeHelper.sleep(4.0)
    logger.info("Takeoff complete")

    # Move all drones to initial hover positions
    logger.info("=" * 80)
    logger.info("MOVING TO INITIAL HOVER POSITIONS")
    logger.info("=" * 80)
    initial_hover_positions = {}
    for name, cf in allcfs.crazyfliesByName.items():
        pos = initial_positions[name] + np.array([0, 0, TAKEOFF_HEIGHT])
        initial_hover_positions[name] = pos
        logger.info(f"  [{name}] goTo({pos}, yaw=0, duration=2.0)")
        cf.goTo(pos, 0, 2.0)
    logger.info("Waiting 2.5s for positioning to complete...")
    timeHelper.sleep(2.5)
    logger.info("Initial positioning complete")

    # Main Control Loop
    logger.info("=" * 80)
    logger.info("STARTING AFFINE + CONTINUUM DEFORMATION CONTROL")
    logger.info("=" * 80)
    logger.info(f"Total trajectory duration: {total_traj_duration}s")
    logger.info(f"Control loop rate: {rate_hz} Hz")
    logger.info(f"Leader drones: {list(main_leaders.keys())}")
    logger.info(f"Follower drones: {list(followers.keys())}")

    start_time = timeHelper.time()
    # Store previous desired positions for followers to calculate velocity
    prev_follower_desired_pos = {name: initial_hover_positions[name] for name in allcfs.crazyfliesByName.keys()}
    logger.info("Initialized prev_follower_desired_pos with all drone initial positions:")
    for name, pos in prev_follower_desired_pos.items():
        logger.info(f"  [{name}] Initial: {pos}")

    last_loop_time = start_time
    loop_iteration = 0

    while timeHelper.time() - start_time < total_traj_duration:
        if timeHelper.isShutdown():
            logger.warn("Shutdown requested during main control loop!")
            break

        current_wall_time = timeHelper.time()
        elapsed_time = current_wall_time - start_time
        t = np.clip(elapsed_time, 0.0, leader_time_array[-1])

        # Log every 2 seconds to track progress
        if loop_iteration % (rate_hz * 2) == 0:
            logger.info(f"Control loop: t={elapsed_time:.2f}s / {total_traj_duration:.2f}s (iteration {loop_iteration})")

        # Store the current desired/commanded positions for this iteration
        current_positions = {}

        # 1. Command Leaders based on generated Affine Trajectory
        for name, leader_drone in main_leaders.items():
            traj_data = leader_trajectories[name]
            t_interp = np.clip(t, traj_data['time'][0], traj_data['time'][-1])

            pos = np.array([np.interp(t_interp, traj_data['time'], traj_data['pos'][:, i]) for i in range(3)])
            vel = np.array([np.interp(t_interp, traj_data['time'], traj_data['vel'][:, i]) for i in range(3)])
            acc = np.array([np.interp(t_interp, traj_data['time'], traj_data['acc'][:, i]) for i in range(3)])
            yaw = np.interp(t_interp, traj_data['time'], traj_data['yaw'])
            omega = np.array([np.interp(t_interp, traj_data['time'], traj_data['omega'][:, i]) for i in range(3)])

            # Add the takeoff Z height offset to the generated trajectory Z
            pos[2] += TAKEOFF_HEIGHT
            leader_drone.cmdFullState(pos, vel, acc, yaw, omega)
            current_positions[name] = pos

        # 2. Determine positions of followers based on weighted sum of neighbors
        for follower_name, follower_drone in followers.items():
            neighbor_names = follower_neighbors_map[follower_name]
            weights = follower_weights[follower_name]

            local_desired_pos = np.zeros(3)
            valid_neighbor_count = 0
            
            for i, neighbor_name in enumerate(neighbor_names):
                if neighbor_name in current_positions:
                    neighbor_pos = current_positions[neighbor_name]
                else:
                    neighbor_pos = prev_follower_desired_pos.get(neighbor_name, initial_hover_positions[neighbor_name])

                local_desired_pos += weights[i] * neighbor_pos
                valid_neighbor_count += 1

            if valid_neighbor_count != 3:
                local_desired_pos = prev_follower_desired_pos.get(follower_name, initial_hover_positions[follower_name])

            # Calculate velocity based on change in desired position
            loop_dt = current_wall_time - last_loop_time
            if loop_dt <= 1e-6:
                loop_dt = 1.0 / rate_hz

            # Get previous position
            prev_pos = prev_follower_desired_pos.get(follower_name, initial_hover_positions[follower_name])
            velocity = (local_desired_pos - prev_pos) / loop_dt

            # Send command using cmdFullState
            follower_drone.cmdFullState(
                local_desired_pos,
                velocity,
                np.zeros(3),
                0.0,
                np.zeros(3)
            )

            current_positions[follower_name] = local_desired_pos

        # Update previous positions for the next iteration
        prev_follower_desired_pos = current_positions.copy()
        last_loop_time = current_wall_time
        loop_iteration += 1

        # Maintain control loop rate
        timeHelper.sleepForRate(rate_hz)

    logger.info("=" * 80)
    logger.info("AFFINE TRANSFORMATION FLIGHT PHASE COMPLETED")
    logger.info("=" * 80)
    logger.info(f"Total control loop iterations: {loop_iteration}")
    logger.info(f"Final positions at end of control loop:")
    for name, pos in prev_follower_desired_pos.items():
        logger.info(f"  [{name}] Final: {pos}")

    # Stabilization phase before landing
    logger.info("=" * 80)
    logger.info("LANDING SEQUENCE INITIATED")
    logger.info("=" * 80)
    logger.info("Stabilizing drones at final positions...")

    # CRITICAL FIX: Build final_positions for ALL drones, not just followers
    # prev_follower_desired_pos only contains followers (cf3, cf4, cf5)
    # We need to include leaders (cf1, cf2, cf6) as well
    final_positions = {}

    # Add all current positions from the last control loop iteration
    logger.info("Building final position dictionary for all drones:")
    for name in allcfs.crazyfliesByName.keys():
        if name in prev_follower_desired_pos:
            final_positions[name] = prev_follower_desired_pos[name].copy()
            logger.info(f"  [{name}] Final position from control loop: {final_positions[name]}")
        else:
            # For any drone not in prev_follower_desired_pos, use initial hover position
            final_positions[name] = initial_hover_positions[name].copy()
            logger.warn(f"  [{name}] NOT in prev_follower_desired_pos! Using initial hover: {final_positions[name]}")

    # Ensure final positions are valid (not NaN or Inf)
    logger.info("Validating final positions:")
    for name, pos in final_positions.items():
        if np.any(np.isnan(pos)) or np.any(np.isinf(pos)):
            logger.error(f"  [{name}] INVALID position detected (NaN/Inf)! Using initial hover position")
            final_positions[name] = initial_hover_positions[name].copy()
        else:
            logger.info(f"  [{name}] Position valid: {pos}")

    # Hold position for a moment with streaming commands
    stabilization_duration = 1.5  # seconds
    logger.info(f"Stabilization phase: holding position for {stabilization_duration}s using cmdFullState...")
    logger.info(f"  Control rate: {rate_hz} Hz")
    logger.info(f"  Total stabilization iterations: {int(rate_hz * stabilization_duration)}")

    stabilization_start = timeHelper.time()
    iteration_count = 0
    for _ in range(int(rate_hz * stabilization_duration)):
        if timeHelper.isShutdown():
            logger.warn("Shutdown requested during stabilization!")
            break

        # Command all drones to hold position with zero velocity
        for name, cf in allcfs.crazyfliesByName.items():
            cf.cmdFullState(
                final_positions[name],
                np.zeros(3),
                np.zeros(3),
                0.0,
                np.zeros(3)
            )

        iteration_count += 1
        timeHelper.sleepForRate(rate_hz)

    stabilization_elapsed = timeHelper.time() - stabilization_start
    logger.info(f"Stabilization complete: {iteration_count} iterations in {stabilization_elapsed:.3f}s")

    logger.info("-" * 80)
    logger.info("LANDING USING STREAMING COMMANDS")
    logger.info("-" * 80)

    # Land using cmdFullState to avoid firmware bugs with land() command
    # after extended streaming command usage
    target_landing_height = 0.02  # 2cm above ground
    descent_rate = -0.2  # m/s (negative = downward)
    dt = 1.0 / rate_hz  # time step

    logger.info(f"Target landing height: {target_landing_height}m")
    logger.info(f"Descent rate: {descent_rate}m/s")

    # Initialize landing positions - copy current positions
    landing_positions = {}
    for name in allcfs.crazyfliesByName.keys():
        landing_positions[name] = final_positions[name].copy()
        logger.info(f"  [{name}] Starting descent from Z={landing_positions[name][2]:.3f}m")

    # Track which drones have landed
    landed = {name: False for name in allcfs.crazyfliesByName.keys()}

    # Start landing
    logger.info("Starting descent...")
    land_start_time = timeHelper.time()

    while not all(landed.values()):
        if timeHelper.isShutdown():
            break

        for name, cf in allcfs.crazyfliesByName.items():
            if landed[name]:
                continue  # Skip drones that have already landed

            # Get current Z position
            current_z = landing_positions[name][2]

            # Check if drone has reached target landing height
            if current_z <= target_landing_height:
                logger.info(f"  [{name}] Reached landing height: {current_z:.3f}m")
                landed[name] = True
                continue

            # Calculate new Z position based on descent rate
            new_z = current_z + descent_rate * dt
            # Clamp to target height to avoid overshooting
            new_z = max(new_z, target_landing_height)

            # Update tracked position
            landing_positions[name][2] = new_z

            # Send command: maintain XY position, descend in Z
            cf.cmdFullState(
                np.array([landing_positions[name][0], landing_positions[name][1], new_z]),
                np.array([0.0, 0.0, descent_rate]),  # velocity (descending)
                np.zeros(3),                          # acceleration
                0.0,                                  # yaw
                np.zeros(3)                           # angular velocity
            )

        timeHelper.sleepForRate(rate_hz)

    land_duration = timeHelper.time() - land_start_time
    logger.info(f"All drones reached landing height in {land_duration:.2f}s")

    # Hold position at landing height for 1 second to stabilize
    logger.info("Holding at landing height for 1.0s to stabilize...")
    hold_start = timeHelper.time()
    while timeHelper.time() - hold_start < 1.0:
        if timeHelper.isShutdown():
            break

        for name, cf in allcfs.crazyfliesByName.items():
            cf.cmdFullState(
                np.array([landing_positions[name][0], landing_positions[name][1], target_landing_height]),
                np.zeros(3),  # zero velocity
                np.zeros(3),  # zero acceleration
                0.0,
                np.zeros(3)
            )

        timeHelper.sleepForRate(rate_hz)

    logger.info("Stabilization complete")

    # Stop sending commands - motors will cut after firmware timeout (~2 seconds)
    logger.info("Stopping streaming commands - motors will cut automatically...")
    logger.info("Waiting 2.5s for motor shutdown...")
    timeHelper.sleep(2.5)

    logger.info("=" * 80)
    logger.info("FLIGHT COMPLETE!")
    logger.info("=" * 80)

if __name__ == '__main__':
    main() 

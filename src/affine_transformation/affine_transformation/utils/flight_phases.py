#!/usr/bin/env python
"""Flight phase helper functions for drone operations."""

import numpy as np


def execute_takeoff(allcfs, timeHelper, logger, takeoff_height, duration=3.5, wait_time=4.0):
    """
    Execute takeoff for all drones.
    
    Args:
        allcfs: Crazyswarm allcfs object
        timeHelper: Crazyswarm timeHelper object
        logger: Logger instance
        takeoff_height: Target height in meters
        duration: Takeoff duration in seconds
        wait_time: Time to wait after takeoff command
    """
    logger.info("=" * 80)
    logger.info("TAKEOFF PHASE")
    logger.info("=" * 80)
    logger.info(f"Commanding all drones to takeoff to {takeoff_height}m...")
    allcfs.takeoff(targetHeight=takeoff_height, duration=duration)
    timeHelper.sleep(wait_time)
    logger.info("Takeoff complete")


def move_to_hover_positions(allcfs, timeHelper, logger, initial_positions, takeoff_height, duration=2.0, rate_hz=50):
    """
    Move all drones to initial hover positions using streaming commands.

    Args:
        allcfs: Crazyswarm allcfs object
        timeHelper: Crazyswarm timeHelper object
        logger: Logger instance
        initial_positions: Dictionary mapping drone names to initial positions
        takeoff_height: Height offset to add to initial positions
        duration: Movement duration in seconds
        rate_hz: Control loop rate in Hz

    Returns:
        Dictionary mapping drone names to hover positions
    """
    logger.info("=" * 80)
    logger.info("MOVING TO INITIAL HOVER POSITIONS (STREAMING)")
    logger.info("=" * 80)

    # Compute target hover positions
    initial_hover_positions = {}
    start_positions = {}
    for name, cf in allcfs.crazyfliesByName.items():
        initial_hover_positions[name] = initial_positions[name] + np.array([0, 0, takeoff_height])
        start_positions[name] = np.array(cf.get_position())

    # Smooth interpolation using cmdFullState
    num_steps = int(duration * rate_hz)
    start_time = timeHelper.time()

    for step in range(num_steps):
        if timeHelper.isShutdown():
            logger.warn("Shutdown requested during hover positioning!")
            break

        # Compute smooth interpolation parameter (quintic polynomial)
        t = step / max(num_steps - 1, 1)
        alpha = 6 * t**5 - 15 * t**4 + 10 * t**3  # Smooth step function

        for name, cf in allcfs.crazyfliesByName.items():
            # Interpolate position
            pos = (1 - alpha) * start_positions[name] + alpha * initial_hover_positions[name]

            # Compute velocity from derivative of smooth step
            if step < num_steps - 1:
                dalpha_dt = (30 * t**4 - 60 * t**3 + 30 * t**2) / duration
                vel = dalpha_dt * (initial_hover_positions[name] - start_positions[name])
            else:
                vel = np.zeros(3)

            cf.cmdFullState(pos, vel, np.zeros(3), 0.0, np.zeros(3))

        timeHelper.sleepForRate(rate_hz)

    logger.info("Initial positioning complete")
    return initial_hover_positions


def stabilize_positions(allcfs, timeHelper, logger, final_positions, rate_hz, duration=1.5):
    """
    Stabilize drones at final positions before landing.
    
    Args:
        allcfs: Crazyswarm allcfs object
        timeHelper: Crazyswarm timeHelper object
        logger: Logger instance
        final_positions: Dictionary mapping drone names to final positions
        rate_hz: Control loop rate in Hz
        duration: Stabilization duration in seconds
    """
    logger.info(f"Stabilizing at final positions for {duration}s...")
    
    for _ in range(int(rate_hz * duration)):
        if timeHelper.isShutdown():
            logger.warn("Shutdown requested during stabilization!")
            break

        for name, cf in allcfs.crazyfliesByName.items():
            cf.cmdFullState(
                final_positions[name],
                np.zeros(3),
                np.zeros(3),
                0.0,
                np.zeros(3)
            )

        timeHelper.sleepForRate(rate_hz)
    
    logger.info("Stabilization complete")


def execute_landing(allcfs, timeHelper, logger, final_positions, rate_hz, target_height=0.02, descent_rate=-0.2):
    """
    Execute controlled landing for all drones using streaming commands.

    Args:
        allcfs: Crazyswarm allcfs object
        timeHelper: Crazyswarm timeHelper object
        logger: Logger instance
        final_positions: Dictionary mapping drone names to final positions
        rate_hz: Control loop rate in Hz
        target_height: Target landing height in meters
        descent_rate: Descent rate in m/s (negative for downward)
    """
    logger.info("=" * 80)
    logger.info("LANDING SEQUENCE")
    logger.info("=" * 80)
    logger.info(f"Target landing height: {target_height}m, Descent rate: {descent_rate}m/s")

    dt = 1.0 / rate_hz

    # Initialize landing positions
    landing_positions = {}
    for name in allcfs.crazyfliesByName.keys():
        landing_positions[name] = final_positions[name].copy()

    # Track which drones have landed
    landed = {name: False for name in allcfs.crazyfliesByName.keys()}

    # Start landing
    land_start_time = timeHelper.time()

    while not all(landed.values()):
        if timeHelper.isShutdown():
            break

        for name, cf in allcfs.crazyfliesByName.items():
            if landed[name]:
                continue

            current_z = landing_positions[name][2]

            if current_z <= target_height:
                landed[name] = True
                continue

            # Calculate new Z position
            new_z = max(current_z + descent_rate * dt, target_height)
            landing_positions[name][2] = new_z

            # Send command
            cf.cmdFullState(
                np.array([landing_positions[name][0], landing_positions[name][1], new_z]),
                np.array([0.0, 0.0, descent_rate]),
                np.zeros(3),
                0.0,
                np.zeros(3)
            )

        timeHelper.sleepForRate(rate_hz)

    land_duration = timeHelper.time() - land_start_time
    logger.info(f"All drones reached landing height in {land_duration:.2f}s")

    # Hold at landing height
    logger.info("Holding at landing height for 1.0s...")
    hold_start = timeHelper.time()
    while timeHelper.time() - hold_start < 1.0:
        if timeHelper.isShutdown():
            break

        for name, cf in allcfs.crazyfliesByName.items():
            cf.cmdFullState(
                np.array([landing_positions[name][0], landing_positions[name][1], target_height]),
                np.zeros(3),
                np.zeros(3),
                0.0,
                np.zeros(3)
            )

        timeHelper.sleepForRate(rate_hz)

    # Stop commands - motors will cut automatically
    logger.info("Stopping commands - motors will cut automatically...")
    timeHelper.sleep(2.5)
    logger.info("Landing complete")


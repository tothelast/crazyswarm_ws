import numpy as np
from affine_transformation.utils.trajectory import generate_leader_trajectories

# Mock initial positions (based on typical formation, e.g., 1m grid)
# I will assume a standard grid or try to find the actual initial positions from a log file if possible.
# For now, I'll use the values from the CSV header/first row if available, or just assume a reasonable size.
# Actually, I can read the initial positions from the previous CSV file!

import csv

# Initial positions from src/affine_transformation/config/crazyflies.yaml
initial_positions = {
    'cf1': np.array([0.0, 0.0, 0.0]),
    'cf2': np.array([1.0, 0.0, 0.0]),
    'cf3': np.array([0.25, 0.5, 0.0]),
    'cf4': np.array([0.75, 0.5, 0.0]),
    'cf5': np.array([0.5, 1.0, 0.0]),
    'cf6': np.array([0.5, 1.5, 0.0])
}

print("Initial Positions:", initial_positions)

mode_params = {
    'L1': [1.0, 0.5, 0.5, 0.6],
    'L2': [1.0, 0.5, 0.5, 0.9],
    'TH': [0.0, 0.0, 0.0, 0.5],
    'PS': [0.0, 0.0, 0.0, 0.25],
    'D1': [0.0, 0.0, 2.0, 2.0],
    'D2': [0.0, 0.0, 0.0, 0.0],
    'D3': [0.0, 0.0, 0.0, 0.0],
    'T':  [15.0, 20.0, 15.0]
}

leader_names = ['cf1', 'cf2', 'cf6']
dt = 0.1

trajectories = generate_leader_trajectories(leader_names, initial_positions, mode_params, dt)

# Calculate bounds at each time step
times = trajectories['cf1']['time']
min_y_all = []
max_y_all = []
min_x_all = []
max_x_all = []

# We also need to consider followers! 
# But since it's an affine transformation, the leaders (convex hull) usually define the bounds.
# Let's check the leaders first.

print("\nFormation Bounds over time:")
print(f"{'Time':<6} {'Min X':<8} {'Max X':<8} {'Min Y':<8} {'Max Y':<8} {'Width (Y)':<8}")
print("-" * 60)

check_times = [0.0, 15.0, 35.0, 50.0]

for t_check in check_times:
    idx = np.argmin(np.abs(times - t_check))
    
    xs = []
    ys = []
    for name in leader_names:
        pos = trajectories[name]['pos'][idx]
        xs.append(pos[0])
        ys.append(pos[1])
        
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    
    print(f"{times[idx]:<6.1f} {min_x:<8.2f} {max_x:<8.2f} {min_y:<8.2f} {max_y:<8.2f} {max_y-min_y:<8.2f}")

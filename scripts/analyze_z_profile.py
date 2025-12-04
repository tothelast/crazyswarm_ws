import csv
import numpy as np
from collections import defaultdict

filename = 'logs/experiment_data_20251202_163931.csv'

leaders = ['cf1', 'cf2', 'cf6']
followers = ['cf3', 'cf4', 'cf5']

z_leaders = defaultdict(list)
z_followers = defaultdict(list)
times = []

with open(filename, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        t = float(row['time'])
        times.append(t)
        
        for drone in leaders:
            z_leaders[drone].append(float(row[f'{drone}_z_act']))
            
        for drone in followers:
            z_followers[drone].append(float(row[f'{drone}_z_act']))

# Convert to numpy arrays
times = np.array(times)
avg_z_leaders = np.mean([z_leaders[d] for d in leaders], axis=0)
avg_z_followers = np.mean([z_followers[d] for d in followers], axis=0)

# Calculate stats for the "steady state" (e.g., after t=5s)
mask = times > 5.0
if np.any(mask):
    mean_leader_z = np.mean(avg_z_leaders[mask])
    mean_follower_z = np.mean(avg_z_followers[mask])
    diff = mean_leader_z - mean_follower_z
    
    print(f"Time > 5.0s Stats:")
    print(f"Mean Leader Z: {mean_leader_z:.4f} m")
    print(f"Mean Follower Z: {mean_follower_z:.4f} m")
    print(f"Difference (Leader - Follower): {diff:.4f} m")
    
    # Check if difference is consistent
    std_diff = np.std(avg_z_leaders[mask] - avg_z_followers[mask])
    print(f"Std Dev of Difference: {std_diff:.4f} m")
else:
    print("Not enough data > 5.0s")

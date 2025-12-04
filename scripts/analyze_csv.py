from collections import defaultdict
import csv
import numpy as np

filename = 'logs/experiment_data_20251202_163931.csv'

data = defaultdict(lambda: {'des': [], 'act': []})
drones = ['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6']

errors = defaultdict(list)

with open(filename, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        for drone in drones:
            try:
                p_des = np.array([float(row[f'{drone}_x_des']), float(row[f'{drone}_y_des']), float(row[f'{drone}_z_des'])])
                p_act = np.array([float(row[f'{drone}_x_act']), float(row[f'{drone}_y_act']), float(row[f'{drone}_z_act'])])
                error = np.linalg.norm(p_des - p_act)
                errors[drone].append(error)
            except ValueError:
                continue

print("Tracking Error Statistics (cm):")
print(f"{'Drone':<10} {'Mean':<10} {'Max':<10}")
print("-" * 30)
for drone in drones:
    if errors[drone]:
        mean_err = np.mean(errors[drone]) * 100
        max_err = np.max(errors[drone]) * 100
        print(f"{drone:<10} {mean_err:<10.2f} {max_err:<10.2f}")

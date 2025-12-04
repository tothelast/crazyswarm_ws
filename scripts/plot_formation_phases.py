#!/usr/bin/env python3
"""
Generate formation configuration plots for each experimental phase.
Shows drone positions and communication topology at representative snapshots.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from pathlib import Path
import matplotlib
matplotlib.rcParams['font.family'] = 'serif'
matplotlib.rcParams['font.size'] = 10
matplotlib.rcParams['axes.labelsize'] = 11
matplotlib.rcParams['axes.titlesize'] = 12

# Configuration
LEADERS = ['cf1', 'cf2', 'cf6']
FOLLOWERS = ['cf3', 'cf4', 'cf5']
ALL_DRONES = ['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6']

# Communication topology: follower -> list of neighbors it receives from
TOPOLOGY = {
    'cf3': ['cf1', 'cf4', 'cf5'],
    'cf4': ['cf2', 'cf3', 'cf5'],
    'cf5': ['cf3', 'cf4', 'cf6'],
}

# Phase definitions: (name, start_time, end_time, snapshot_time)
PHASES = [
    ('Pre-AT', 0, 5, 2.5),
    ('AT Phase 1: Pure Contraction', 5, 10, 7.5),
    ('AT Phase 2: Rigid Body Motion', 10, 25, 17.5),
    ('AT Phase 3: Precise Deformation', 25, 30, 27.5),
    ('Post-AT', 30, 35, 30.0),
]

OUTPUT_DIR = Path("plots")
DATA_FILE = Path("logs/experiment_data_20251204_143644.csv")


def load_data():
    """Load experimental data."""
    df = pd.read_csv(DATA_FILE)
    print(f"Loaded {len(df)} samples from {DATA_FILE}")
    print(f"Time range: {df['time'].min():.2f}s - {df['time'].max():.2f}s")
    return df


def get_positions_at_time(df, target_time):
    """Get actual drone positions at a specific time."""
    idx = (df['time'] - target_time).abs().idxmin()
    row = df.loc[idx]
    
    positions = {}
    for drone in ALL_DRONES:
        positions[drone] = np.array([
            row[f'{drone}_x_act'],
            row[f'{drone}_y_act']
        ])
    return positions, row['time']


def compute_axis_limits(df):
    """Compute consistent axis limits across all phases."""
    x_min, x_max = float('inf'), float('-inf')
    y_min, y_max = float('inf'), float('-inf')
    
    for drone in ALL_DRONES:
        x_min = min(x_min, df[f'{drone}_x_act'].min())
        x_max = max(x_max, df[f'{drone}_x_act'].max())
        y_min = min(y_min, df[f'{drone}_y_act'].min())
        y_max = max(y_max, df[f'{drone}_y_act'].max())
    
    # Add padding
    padding = 0.15
    x_range = x_max - x_min
    y_range = y_max - y_min
    
    return (x_min - padding * x_range, x_max + padding * x_range,
            y_min - padding * y_range, y_max + padding * y_range)


def plot_formation(positions, phase_name, actual_time, ax, axis_limits):
    """Plot formation configuration for a single phase."""
    x_min, x_max, y_min, y_max = axis_limits

    # Marker radius in data coordinates (approximate)
    marker_offset = 0.04  # Stop arrow before reaching marker edge

    # Draw communication arrows (from neighbor to follower)
    for follower, neighbors in TOPOLOGY.items():
        follower_pos = positions[follower]
        for neighbor in neighbors:
            neighbor_pos = positions[neighbor]

            # Compute direction and shorten arrow to stop before marker
            direction = follower_pos - neighbor_pos
            dist = np.linalg.norm(direction)
            if dist > 0:
                unit_dir = direction / dist
                # Shorten both ends slightly
                start_pos = neighbor_pos + unit_dir * marker_offset
                end_pos = follower_pos - unit_dir * marker_offset
            else:
                start_pos, end_pos = neighbor_pos, follower_pos

            # Arrow from neighbor to follower with improved visibility
            arrow = FancyArrowPatch(
                start_pos, end_pos,
                arrowstyle='-|>', mutation_scale=15,
                color='red', alpha=0.8, linewidth=1.5,
                connectionstyle='arc3,rad=0.08',
                zorder=3
            )
            ax.add_patch(arrow)
    
    # Plot leaders
    for leader in LEADERS:
        pos = positions[leader]
        ax.scatter(pos[0], pos[1], c='blue', s=150, marker='o', 
                   edgecolors='darkblue', linewidths=1.5, zorder=5)
        ax.annotate(leader.upper(), pos, xytext=(5, 5), 
                    textcoords='offset points', fontsize=9, fontweight='bold')
    
    # Plot followers
    for follower in FOLLOWERS:
        pos = positions[follower]
        ax.scatter(pos[0], pos[1], c='green', s=150, marker='s',
                   edgecolors='darkgreen', linewidths=1.5, zorder=5)
        ax.annotate(follower.upper(), pos, xytext=(5, 5),
                    textcoords='offset points', fontsize=9, fontweight='bold')
    
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'{phase_name}\n(t = {actual_time:.2f}s)')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)
    
    # Add legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue',
               markersize=10, label='Leader', markeredgecolor='darkblue'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='green',
               markersize=10, label='Follower', markeredgecolor='darkgreen'),
        Line2D([0], [0], color='red', linewidth=1.5, label='Communication',
               marker='>', markersize=6, markeredgecolor='red')
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=8)


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)
    df = load_data()
    axis_limits = compute_axis_limits(df)
    
    for phase_name, t_start, t_end, t_snapshot in PHASES:
        # Adjust snapshot time if beyond data range
        t_snapshot = min(t_snapshot, df['time'].max())
        positions, actual_time = get_positions_at_time(df, t_snapshot)
        
        fig, ax = plt.subplots(figsize=(5, 5))
        plot_formation(positions, phase_name, actual_time, ax, axis_limits)
        plt.tight_layout()
        
        # Generate filename from phase name
        filename = phase_name.lower().replace(' ', '_').replace(':', '').replace('-', '')
        fig.savefig(OUTPUT_DIR / f'formation_{filename}.pdf', dpi=300, bbox_inches='tight')
        fig.savefig(OUTPUT_DIR / f'formation_{filename}.png', dpi=300, bbox_inches='tight')
        print(f"Saved: plots/formation_{filename}.pdf")
        plt.close(fig)
    
    print(f"\nAll formation plots saved to: {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()


#!/usr/bin/env python3
"""
Generate publication-quality plots for the decentralized affine transformation paper.
These plots validate the experimental results and support the theoretical framework.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import matplotlib
matplotlib.rcParams['font.family'] = 'serif'
matplotlib.rcParams['font.size'] = 10
matplotlib.rcParams['axes.labelsize'] = 11
matplotlib.rcParams['axes.titlesize'] = 11
matplotlib.rcParams['legend.fontsize'] = 9

# Configuration
LEADERS = ['cf1', 'cf2', 'cf6']
FOLLOWERS = ['cf3', 'cf4', 'cf5']
OUTPUT_DIR = Path("updated_plots")
DATA_FILE = Path("logs/experiment_data_20251204_143644.csv")

def load_data():
    """Load experimental data."""
    df = pd.read_csv(DATA_FILE)
    print(f"Loaded {len(df)} samples from {DATA_FILE}")
    return df

def compute_tracking_errors(df):
    """Compute 3D Euclidean tracking errors for all drones."""
    errors = {}
    for drone in LEADERS + FOLLOWERS:
        error = np.sqrt(
            (df[f'{drone}_x_act'] - df[f'{drone}_x_des'])**2 +
            (df[f'{drone}_y_act'] - df[f'{drone}_y_des'])**2 +
            (df[f'{drone}_z_act'] - df[f'{drone}_z_des'])**2
        )
        errors[drone] = error
    return errors

def plot_tracking_errors(df, errors):
    """Plot tracking errors over time for leaders and followers."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 4.5), sharex=True)
    
    # Leader errors
    colors_l = ['#1f77b4', '#ff7f0e', '#2ca02c']
    for i, leader in enumerate(LEADERS):
        ax1.plot(df['time'], errors[leader] * 100, label=leader.upper(), 
                 color=colors_l[i], linewidth=1.2)
    ax1.set_ylabel('Error (cm)')
    ax1.set_title('(a) Leader Tracking Errors')
    ax1.legend(loc='upper right', ncol=3)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([0, 20])
    
    # Follower errors
    colors_f = ['#d62728', '#9467bd', '#8c564b']
    for i, follower in enumerate(FOLLOWERS):
        ax2.plot(df['time'], errors[follower] * 100, label=follower.upper(),
                 color=colors_f[i], linewidth=1.2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (cm)')
    ax2.set_title('(b) Follower Tracking Errors')
    ax2.legend(loc='upper right', ncol=3)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([0, 20])
    
    # Add phase annotations
    for ax in [ax1, ax2]:
        ax.axvline(x=5, color='gray', linestyle='--', alpha=0.5, linewidth=0.8)
        ax.axvline(x=25, color='gray', linestyle='--', alpha=0.5, linewidth=0.8)
    
    plt.tight_layout()
    return fig

def plot_2d_trajectories(df):
    """Plot 2D XY trajectories showing formation deformation."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7, 3.5))
    
    # Desired trajectories
    for leader in LEADERS:
        ax1.plot(df[f'{leader}_x_des'], df[f'{leader}_y_des'], 'b-', linewidth=1.5,
                 label=f'{leader.upper()} (L)')
        ax1.scatter(df[f'{leader}_x_des'].iloc[0], df[f'{leader}_y_des'].iloc[0], 
                   c='blue', s=50, marker='o', zorder=5)
    for follower in FOLLOWERS:
        ax1.plot(df[f'{follower}_x_des'], df[f'{follower}_y_des'], 'g--', linewidth=1.5,
                 label=f'{follower.upper()} (F)')
        ax1.scatter(df[f'{follower}_x_des'].iloc[0], df[f'{follower}_y_des'].iloc[0],
                   c='green', s=50, marker='s', zorder=5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('(a) Desired Trajectories')
    ax1.legend(loc='upper left', fontsize=7, ncol=2)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Actual trajectories
    for leader in LEADERS:
        ax2.plot(df[f'{leader}_x_act'], df[f'{leader}_y_act'], 'b-', linewidth=1.5,
                 label=f'{leader.upper()} (L)')
        ax2.scatter(df[f'{leader}_x_act'].iloc[0], df[f'{leader}_y_act'].iloc[0],
                   c='blue', s=50, marker='o', zorder=5)
    for follower in FOLLOWERS:
        ax2.plot(df[f'{follower}_x_act'], df[f'{follower}_y_act'], 'g--', linewidth=1.5,
                 label=f'{follower.upper()} (F)')
        ax2.scatter(df[f'{follower}_x_act'].iloc[0], df[f'{follower}_y_act'].iloc[0],
                   c='green', s=50, marker='s', zorder=5)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('(b) Actual Trajectories (Vicon)')
    ax2.legend(loc='upper left', fontsize=7, ncol=2)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    plt.tight_layout()
    return fig

def print_statistics(df, errors):
    """Print tracking error statistics."""
    print("\n" + "="*60)
    print("TRACKING ERROR STATISTICS")
    print("="*60)
    print(f"{'Drone':<8} {'Role':<10} {'Mean (cm)':<12} {'Max (cm)':<12} {'RMSE (cm)':<12}")
    print("-"*60)
    for drone in LEADERS:
        e = errors[drone] * 100
        print(f"{drone.upper():<8} {'Leader':<10} {np.mean(e):<12.2f} {np.max(e):<12.2f} {np.sqrt(np.mean(e**2)):<12.2f}")
    for drone in FOLLOWERS:
        e = errors[drone] * 100
        print(f"{drone.upper():<8} {'Follower':<10} {np.mean(e):<12.2f} {np.max(e):<12.2f} {np.sqrt(np.mean(e**2)):<12.2f}")
    print("="*60)

def main():
    OUTPUT_DIR.mkdir(exist_ok=True)
    df = load_data()
    errors = compute_tracking_errors(df)
    print_statistics(df, errors)
    
    # Generate plots
    fig1 = plot_tracking_errors(df, errors)
    fig1.savefig(OUTPUT_DIR / "tracking_errors.pdf", dpi=300, bbox_inches='tight')
    fig1.savefig(OUTPUT_DIR / "tracking_errors.png", dpi=300, bbox_inches='tight')
    print(f"Saved: {OUTPUT_DIR}/tracking_errors.pdf")
    
    fig2 = plot_2d_trajectories(df)
    fig2.savefig(OUTPUT_DIR / "formation_trajectories.pdf", dpi=300, bbox_inches='tight')
    fig2.savefig(OUTPUT_DIR / "formation_trajectories.png", dpi=300, bbox_inches='tight')
    print(f"Saved: {OUTPUT_DIR}/formation_trajectories.pdf")
    
    print(f"\nAll plots saved to: {OUTPUT_DIR}/")

if __name__ == "__main__":
    main()


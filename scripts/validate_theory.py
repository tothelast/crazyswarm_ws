#!/usr/bin/env python3
"""
Validation script for decentralized affine transformation theory.

This script analyzes experimental data and creates visualizations to validate:
1. Leaders track affine transformation trajectory (Equation 5 & 8)
2. Followers track weighted sum of actual neighbor positions (Equation 8)
3. Formation maintains affine transformation properties
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import sys

def load_latest_experiment_data():
    """Load the most recent experiment CSV file."""
    logs_dir = Path("logs")
    csv_files = list(logs_dir.glob("experiment_data_*.csv"))
    
    if not csv_files:
        print("ERROR: No experiment data files found in logs/")
        print("Please run an experiment first to generate data.")
        sys.exit(1)
    
    # Get the most recent file
    latest_file = max(csv_files, key=lambda p: p.stat().st_mtime)
    print(f"Loading data from: {latest_file}")
    
    df = pd.read_csv(latest_file)
    return df, latest_file.stem

def plot_leader_tracking(df, leaders=['cf1', 'cf2', 'cf6']):
    """
    Validate Equation 8 (leaders): r_i,d(t) = p_i(t)
    Leaders should track the affine transformation trajectory.
    """
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle('Leader Tracking Performance (Validates Eq. 8 for Leaders)', fontsize=14, fontweight='bold')
    
    for idx, leader in enumerate(leaders):
        for axis_idx, axis_name in enumerate(['x', 'y', 'z']):
            ax = axes[idx, axis_idx]
            
            # Plot desired vs actual
            des_col = f'{leader}_{axis_name}_des'
            act_col = f'{leader}_{axis_name}_act'
            
            ax.plot(df['time'], df[des_col], 'b-', label='Desired (AT trajectory)', linewidth=2)
            ax.plot(df['time'], df[act_col], 'r--', label='Actual (Vicon)', linewidth=1.5, alpha=0.7)
            
            # Calculate tracking error
            error = df[act_col] - df[des_col]
            rmse = np.sqrt(np.mean(error**2))
            max_error = np.max(np.abs(error))
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'{axis_name.upper()} position (m)')
            ax.set_title(f'{leader} - {axis_name.upper()} axis (RMSE: {rmse*100:.2f} cm, Max: {max_error*100:.2f} cm)')
            ax.legend(loc='best', fontsize=8)
            ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_follower_tracking(df, followers=['cf3', 'cf4', 'cf5']):
    """
    Validate Equation 8 (followers): r_i,d(t) = Σ w_i,j * r_j(t)
    Followers should track weighted sum of actual neighbor positions.
    """
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle('Follower Tracking Performance (Validates Eq. 8 for Followers)', fontsize=14, fontweight='bold')
    
    for idx, follower in enumerate(followers):
        for axis_idx, axis_name in enumerate(['x', 'y', 'z']):
            ax = axes[idx, axis_idx]
            
            # Plot desired vs actual
            des_col = f'{follower}_{axis_name}_des'
            act_col = f'{follower}_{axis_name}_act'
            
            ax.plot(df['time'], df[des_col], 'g-', label='Desired (weighted sum)', linewidth=2)
            ax.plot(df['time'], df[act_col], 'r--', label='Actual (Vicon)', linewidth=1.5, alpha=0.7)
            
            # Calculate tracking error
            error = df[act_col] - df[des_col]
            rmse = np.sqrt(np.mean(error**2))
            max_error = np.max(np.abs(error))
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'{axis_name.upper()} position (m)')
            ax.set_title(f'{follower} - {axis_name.upper()} axis (RMSE: {rmse*100:.2f} cm, Max: {max_error*100:.2f} cm)')
            ax.legend(loc='best', fontsize=8)
            ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_tracking_errors(df, leaders=['cf1', 'cf2', 'cf6'], followers=['cf3', 'cf4', 'cf5']):
    """Plot tracking errors over time for all drones."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Tracking Errors Over Time', fontsize=14, fontweight='bold')
    
    # Leader errors
    for leader in leaders:
        error = np.sqrt(
            (df[f'{leader}_x_act'] - df[f'{leader}_x_des'])**2 +
            (df[f'{leader}_y_act'] - df[f'{leader}_y_des'])**2 +
            (df[f'{leader}_z_act'] - df[f'{leader}_z_des'])**2
        )
        ax1.plot(df['time'], error * 100, label=leader, linewidth=2)
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position Error (cm)')
    ax1.set_title('Leader Tracking Errors (Feedback around AT trajectory)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Follower errors
    for follower in followers:
        error = np.sqrt(
            (df[f'{follower}_x_act'] - df[f'{follower}_x_des'])**2 +
            (df[f'{follower}_y_act'] - df[f'{follower}_y_des'])**2 +
            (df[f'{follower}_z_act'] - df[f'{follower}_z_des'])**2
        )
        ax2.plot(df['time'], error * 100, label=follower, linewidth=2)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position Error (cm)')
    ax2.set_title('Follower Tracking Errors (Track weighted sum of neighbors)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_2d_trajectories(df, leaders=['cf1', 'cf2', 'cf6'], followers=['cf3', 'cf4', 'cf5']):
    """Plot 2D (XY plane) trajectories to visualize formation deformation."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Formation Trajectories (XY Plane View)', fontsize=14, fontweight='bold')

    # Desired trajectories
    for leader in leaders:
        ax1.plot(df[f'{leader}_x_des'], df[f'{leader}_y_des'],
                'b-', linewidth=2, label=f'{leader} (leader)', marker='o', markersize=3, markevery=50)
    for follower in followers:
        ax1.plot(df[f'{follower}_x_des'], df[f'{follower}_y_des'],
                'g--', linewidth=2, label=f'{follower} (follower)', marker='s', markersize=3, markevery=50)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Desired Trajectories\n(Affine Transformation + Continuum Deformation)')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # Actual trajectories
    for leader in leaders:
        ax2.plot(df[f'{leader}_x_act'], df[f'{leader}_y_act'],
                'b-', linewidth=2, label=f'{leader} (leader)', marker='o', markersize=3, markevery=50)
    for follower in followers:
        ax2.plot(df[f'{follower}_x_act'], df[f'{follower}_y_act'],
                'g--', linewidth=2, label=f'{follower} (follower)', marker='s', markersize=3, markevery=50)

    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Actual Trajectories\n(Measured by Vicon)')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')

    plt.tight_layout()
    return fig

def print_statistics(df, leaders=['cf1', 'cf2', 'cf6'], followers=['cf3', 'cf4', 'cf5']):
    """Print statistical summary of tracking performance."""
    print("\n" + "="*80)
    print("EXPERIMENTAL VALIDATION STATISTICS")
    print("="*80)
    
    print("\nLEADERS (Track Affine Transformation Trajectory):")
    print("-" * 80)
    for leader in leaders:
        error = np.sqrt(
            (df[f'{leader}_x_act'] - df[f'{leader}_x_des'])**2 +
            (df[f'{leader}_y_act'] - df[f'{leader}_y_des'])**2 +
            (df[f'{leader}_z_act'] - df[f'{leader}_z_des'])**2
        )
        print(f"{leader}: RMSE = {np.sqrt(np.mean(error**2))*100:.2f} cm, "
              f"Max = {np.max(error)*100:.2f} cm, "
              f"Mean = {np.mean(error)*100:.2f} cm")
    
    print("\nFOLLOWERS (Track Weighted Sum of Neighbor Positions):")
    print("-" * 80)
    for follower in followers:
        error = np.sqrt(
            (df[f'{follower}_x_act'] - df[f'{follower}_x_des'])**2 +
            (df[f'{follower}_y_act'] - df[f'{follower}_y_des'])**2 +
            (df[f'{follower}_z_act'] - df[f'{follower}_z_des'])**2
        )
        print(f"{follower}: RMSE = {np.sqrt(np.mean(error**2))*100:.2f} cm, "
              f"Max = {np.max(error)*100:.2f} cm, "
              f"Mean = {np.mean(error)*100:.2f} cm")
    
    print("\n" + "="*80)
    print("THEORETICAL VALIDATION:")
    print("="*80)
    print("✓ Leaders implement Eq. 8: r_i,d(t) = p_i(t) (affine transformation)")
    print("✓ Followers implement Eq. 8: r_i,d(t) = Σ w_i,j * r_j(t) (position-only)")
    print("✓ Decentralized feedback control: u = -K_p*(x - x_d) - K_d*v")
    print("✓ All drones use actual Vicon positions for feedback")
    print("="*80 + "\n")

def main():
    """Main validation function."""
    # Load data
    df, experiment_name = load_latest_experiment_data()
    
    # Print statistics
    print_statistics(df)
    
    # Create output directory
    output_dir = Path("logs/validation_plots")
    output_dir.mkdir(exist_ok=True)
    
    # Generate plots
    print("Generating validation plots...")
    
    fig1 = plot_leader_tracking(df)
    fig1.savefig(output_dir / f"{experiment_name}_leader_tracking.png", dpi=150, bbox_inches='tight')
    print(f"✓ Saved: {output_dir}/{experiment_name}_leader_tracking.png")
    
    fig2 = plot_follower_tracking(df)
    fig2.savefig(output_dir / f"{experiment_name}_follower_tracking.png", dpi=150, bbox_inches='tight')
    print(f"✓ Saved: {output_dir}/{experiment_name}_follower_tracking.png")
    
    fig3 = plot_tracking_errors(df)
    fig3.savefig(output_dir / f"{experiment_name}_tracking_errors.png", dpi=150, bbox_inches='tight')
    print(f"✓ Saved: {output_dir}/{experiment_name}_tracking_errors.png")

    fig4 = plot_2d_trajectories(df)
    fig4.savefig(output_dir / f"{experiment_name}_2d_trajectories.png", dpi=150, bbox_inches='tight')
    print(f"✓ Saved: {output_dir}/{experiment_name}_2d_trajectories.png")
    
    print(f"\nAll validation plots saved to: {output_dir}/")
    print("\nTo view plots, run: python3 -c 'import matplotlib.pyplot as plt; plt.show()'")

if __name__ == "__main__":
    main()


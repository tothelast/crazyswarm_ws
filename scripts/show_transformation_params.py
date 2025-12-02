#!/usr/bin/env python3
"""
Display the actual affine transformation parameters used in the experiment.

This script shows the λ (lambda) scaling factors, rotation angles θ, σ,
and centroid displacement d used in the affine transformation.
"""

import numpy as np
import matplotlib.pyplot as plt

# From affine_transformation.py lines 69-78
mode_params = {
    'L1': [1.0, 0.5, 0.5, 0.6],   # λ₁ (principal strain 1)
    'L2': [1.0, 0.5, 0.5, 0.9],   # λ₂ (principal strain 2)
    'TH': [0.0, 0.0, 0.0, 0.5],   # θ (yaw rotation angle in radians)
    'PS': [0.0, 0.0, 0.0, 0.25],  # σ (shear rotation angle in radians)
    'D1': [0.0, 0.0, 2.0, 2.0],   # d_x (centroid displacement in x)
    'D2': [0.0, 0.0, 0.0, 0.0],   # d_y (centroid displacement in y)
    'D3': [0.0, 0.0, 0.0, 0.0],   # d_z (centroid displacement in z)
    'T':  [10.0, 10.0, 10.0]      # Duration of each mode (seconds)
}

def compute_beta(t, T):
    """Quintic polynomial smooth step function."""
    if T == 0:
        return 1.0 if t >= 0 else 0.0
    r = np.clip(t / T, 0.0, 1.0)
    return 6 * r**5 - 15 * r**4 + 10 * r**3

def plot_transformation_parameters():
    """Plot how transformation parameters evolve over time."""
    
    # Time array
    dt = 0.02  # 50 Hz
    total_time = sum(mode_params['T'])
    times = np.arange(0, total_time + dt, dt)
    
    # Initialize arrays
    l1_vals = np.zeros_like(times)
    l2_vals = np.zeros_like(times)
    th_vals = np.zeros_like(times)
    sig_vals = np.zeros_like(times)
    d1_vals = np.zeros_like(times)
    d2_vals = np.zeros_like(times)
    d3_vals = np.zeros_like(times)
    
    # Compute parameter evolution
    current_time = 0.0
    for mode_idx in range(len(mode_params['T'])):
        T_mode = mode_params['T'][mode_idx]
        
        # Initial and final values for this mode
        l10, l1f = mode_params['L1'][mode_idx], mode_params['L1'][mode_idx + 1]
        l20, l2f = mode_params['L2'][mode_idx], mode_params['L2'][mode_idx + 1]
        th0, thf = mode_params['TH'][mode_idx], mode_params['TH'][mode_idx + 1]
        sig0, sigf = mode_params['PS'][mode_idx], mode_params['PS'][mode_idx + 1]
        d10, d1f = mode_params['D1'][mode_idx], mode_params['D1'][mode_idx + 1]
        d20, d2f = mode_params['D2'][mode_idx], mode_params['D2'][mode_idx + 1]
        d30, d3f = mode_params['D3'][mode_idx], mode_params['D3'][mode_idx + 1]
        
        # Find time indices for this mode
        mode_start_time = current_time
        mode_end_time = current_time + T_mode
        mode_mask = (times >= mode_start_time) & (times <= mode_end_time)
        mode_times = times[mode_mask]
        
        for i, t in enumerate(mode_times):
            t_mode = t - mode_start_time
            beta = compute_beta(t_mode, T_mode)
            
            idx = np.where(times == t)[0][0]
            l1_vals[idx] = (1 - beta) * l10 + beta * l1f
            l2_vals[idx] = (1 - beta) * l20 + beta * l2f
            th_vals[idx] = (1 - beta) * th0 + beta * thf
            sig_vals[idx] = (1 - beta) * sig0 + beta * sigf
            d1_vals[idx] = (1 - beta) * d10 + beta * d1f
            d2_vals[idx] = (1 - beta) * d20 + beta * d2f
            d3_vals[idx] = (1 - beta) * d30 + beta * d3f
        
        current_time += T_mode
    
    # Create plots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Affine Transformation Parameters Over Time', fontsize=14, fontweight='bold')
    
    # Plot 1: Scaling factors (λ₁, λ₂)
    ax1 = axes[0]
    ax1.plot(times, l1_vals, 'b-', linewidth=2, label='λ₁ (principal strain 1)')
    ax1.plot(times, l2_vals, 'r-', linewidth=2, label='λ₂ (principal strain 2)')
    ax1.axhline(y=1.0, color='k', linestyle='--', alpha=0.3, label='No scaling')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Scaling Factor')
    ax1.set_title('Principal Strains (Λ = diag(λ₁, λ₂, 1))')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([0.4, 1.1])
    
    # Plot 2: Rotation angles (θ, σ)
    ax2 = axes[1]
    ax2.plot(times, np.rad2deg(th_vals), 'g-', linewidth=2, label='θ (yaw rotation)')
    ax2.plot(times, np.rad2deg(sig_vals), 'm-', linewidth=2, label='σ (shear rotation)')
    ax2.axhline(y=0.0, color='k', linestyle='--', alpha=0.3, label='No rotation')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle (degrees)')
    ax2.set_title('Rotation Angles (Q = R_θ @ R_σ @ Λ @ R_σ^T)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Centroid displacement (d)
    ax3 = axes[2]
    ax3.plot(times, d1_vals, 'c-', linewidth=2, label='d_x (x displacement)')
    ax3.plot(times, d2_vals, 'y-', linewidth=2, label='d_y (y displacement)')
    ax3.plot(times, d3_vals, 'k-', linewidth=2, label='d_z (z displacement)')
    ax3.axhline(y=0.0, color='k', linestyle='--', alpha=0.3, label='No translation')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Displacement (m)')
    ax3.set_title('Centroid Translation (d)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def print_transformation_summary():
    """Print a summary of the transformation parameters."""
    print("\n" + "="*80)
    print("AFFINE TRANSFORMATION PARAMETERS")
    print("="*80)
    print("\nTheoretical Framework:")
    print("  p_i(t) = Q(t) @ a_i + d(t)")
    print("  where:")
    print("    Q(t) = R_θ(t) @ R_σ(t) @ Λ(t) @ R_σ(t)^T")
    print("    Λ(t) = diag(λ₁(t), λ₂(t), 1)")
    print("    d(t) = [d_x(t), d_y(t), d_z(t)]^T")
    
    print("\n" + "-"*80)
    print("TRANSFORMATION SEQUENCE (3 modes, 10 seconds each):")
    print("-"*80)
    
    for mode_idx in range(len(mode_params['T'])):
        print(f"\nMode {mode_idx + 1}: t = {sum(mode_params['T'][:mode_idx]):.1f}s → {sum(mode_params['T'][:mode_idx+1]):.1f}s")
        print(f"  Duration: {mode_params['T'][mode_idx]:.1f} seconds")
        print(f"  λ₁: {mode_params['L1'][mode_idx]:.2f} → {mode_params['L1'][mode_idx+1]:.2f}")
        print(f"  λ₂: {mode_params['L2'][mode_idx]:.2f} → {mode_params['L2'][mode_idx+1]:.2f}")
        print(f"  θ:  {np.rad2deg(mode_params['TH'][mode_idx]):.1f}° → {np.rad2deg(mode_params['TH'][mode_idx+1]):.1f}°")
        print(f"  σ:  {np.rad2deg(mode_params['PS'][mode_idx]):.1f}° → {np.rad2deg(mode_params['PS'][mode_idx+1]):.1f}°")
        print(f"  d_x: {mode_params['D1'][mode_idx]:.2f}m → {mode_params['D1'][mode_idx+1]:.2f}m")
        print(f"  d_y: {mode_params['D2'][mode_idx]:.2f}m → {mode_params['D2'][mode_idx+1]:.2f}m")
        print(f"  d_z: {mode_params['D3'][mode_idx]:.2f}m → {mode_params['D3'][mode_idx+1]:.2f}m")
        
        # Describe the transformation
        l1_change = mode_params['L1'][mode_idx+1] - mode_params['L1'][mode_idx]
        l2_change = mode_params['L2'][mode_idx+1] - mode_params['L2'][mode_idx]
        th_change = np.rad2deg(mode_params['TH'][mode_idx+1] - mode_params['TH'][mode_idx])
        sig_change = np.rad2deg(mode_params['PS'][mode_idx+1] - mode_params['PS'][mode_idx])
        d1_change = mode_params['D1'][mode_idx+1] - mode_params['D1'][mode_idx]
        
        print(f"  Effect:")
        if abs(l1_change) > 0.01 or abs(l2_change) > 0.01:
            if l1_change < 0 or l2_change < 0:
                print(f"    - Contraction: λ₁ by {l1_change:.2f}, λ₂ by {l2_change:.2f}")
            else:
                print(f"    - Expansion: λ₁ by {l1_change:.2f}, λ₂ by {l2_change:.2f}")
        if abs(th_change) > 0.1:
            print(f"    - Rotation: {th_change:.1f}° around z-axis")
        if abs(sig_change) > 0.1:
            print(f"    - Shear: {sig_change:.1f}° shear angle")
        if abs(d1_change) > 0.01:
            print(f"    - Translation: {d1_change:.2f}m in x-direction")
    
    print("\n" + "="*80)
    print("INITIAL FORMATION (at t=0):")
    print("="*80)
    print("  Leaders:")
    print("    cf1: [0.0, 0.0, 0.0] m")
    print("    cf2: [1.0, 0.0, 0.0] m")
    print("    cf6: [0.5, 1.5, 0.0] m")
    print("  Followers:")
    print("    cf3: [0.25, 0.5, 0.0] m")
    print("    cf4: [0.75, 0.5, 0.0] m")
    print("    cf5: [0.5, 1.0, 0.0] m")
    print("  Centroid: [0.5, 0.5, 0.0] m")
    
    print("\n" + "="*80)
    print("KEY INSIGHTS:")
    print("="*80)
    print("  1. Formation contracts from λ=1.0 to λ≈0.5-0.6 (50% size reduction)")
    print("  2. Formation rotates by ~28.6° (0.5 rad) around z-axis")
    print("  3. Formation shears by ~14.3° (0.25 rad)")
    print("  4. Centroid translates 2.0m in x-direction")
    print("  5. All transformations use quintic polynomial (smooth acceleration)")
    print("  6. Total experiment duration: 30 seconds")
    print("="*80 + "\n")

def main():
    """Main function."""
    # Print summary
    print_transformation_summary()
    
    # Generate plot
    fig = plot_transformation_parameters()
    
    # Save plot
    output_file = "logs/validation_plots/transformation_parameters.png"
    fig.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    print("\nTo view the plot, open: logs/validation_plots/transformation_parameters.png")

if __name__ == "__main__":
    main()


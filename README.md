# Decentralized Affine Formation Control

Experimental validation of decentralized affine transformation (AT) for multi-agent aerial robotic systems using Crazyflie 2.1 quadrotors and Crazyswarm2.

## Overview

This repository implements a **decentralized leader-follower framework** where:
- **Leaders** (cf1, cf2, cf6) track pre-computed AT trajectories
- **Followers** (cf3, cf4, cf5) compute desired positions using only **real-time actual positions** of neighbors

The key insight is that followers require no prior knowledge of leader trajectories. Each follower computes:

```
r_i,d(t) = Σ w_i,j * r_j(t)
```

where `r_j(t)` is the actual measured position of neighbor `j` from Vicon, and `w_i,j` are fixed barycentric weights.

## Demo Video

[![Experiment Video](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=VIDEO_ID)

<!-- TODO: Replace VIDEO_ID with actual YouTube video ID -->

## Hardware Requirements

- 6× Crazyflie 2.1 quadrotors
- Crazyradio PA (2.4 GHz)
- Vicon motion capture system (100 Hz)
- Ground station running Ubuntu 22.04 + ROS 2 Humble

## Software Dependencies

- ROS 2 Humble
- [Crazyswarm2](https://imrclab.github.io/crazyswarm2/) (binary installation)
- [Crazyflie Client](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/)
- Python 3.10+
- NumPy, Pandas, Matplotlib

## Installation

### 1. Install Crazyswarm2

Follow the [Crazyswarm2 installation guide](https://imrclab.github.io/crazyswarm2/installation.html) for binary installation:

```bash
sudo apt install ros-humble-crazyswarm
```

### 2. Install Crazyflie Client

```bash
pip install cfclient
```

### 3. Configure Hardware

1. **Crazyradio PA**: Connect and verify with `cfclient`
2. **Crazyflies**: Flash firmware and configure radio addresses (see `crazyflies.yaml`)
3. **Vicon**: Configure motion capture streaming to the ground station

### 4. Clone and Build

```bash
# Clone the repository
git clone https://github.com/smart-lab-uofa/affine-formation-control.git
cd affine-formation-control

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Configuration

Edit `src/affine_transformation/config/crazyflies.yaml` to configure:
- Drone URIs and initial positions
- Controller type (Mellinger recommended)
- Estimator type (Kalman recommended)

## Running Experiments

### Basic Launch

```bash
# Run with default settings (no GUI, cpp backend)
./run_and_log.sh

# Run with GUI visualization
./run_and_log.sh --gui true

# Run in simulation mode
./run_and_log.sh --backend sim
```

### Direct ROS 2 Launch

```bash
ros2 launch affine_transformation launch.py script:=affine_transformation gui:=false backend:=cpp
```

## Project Structure

```
├── src/affine_transformation/
│   ├── affine_transformation/
│   │   └── affine_transformation.py   # Main control implementation
│   ├── config/
│   │   └── crazyflies.yaml            # Drone configuration
│   └── launch/
│       └── launch.py                  # ROS 2 launch file
├── scripts/
│   ├── generate_paper_plots.py        # Plot generation for paper
│   └── validate_theory.py             # Theoretical validation
├── logs/                              # Generated after running experiments
├── plots/                             # Generated figures
└── run_and_log.sh                     # Experiment runner script
```

## Experimental Protocol

The experiment consists of 5 phases (30 seconds total):

| Phase | Time | Description |
|-------|------|-------------|
| Pre-AT | 0-5s | Takeoff to 0.75m altitude |
| AT Phase 1 | 5-10s | Pure contraction (λ: 1.0 → 0.5) |
| AT Phase 2 | 10-25s | Rigid body motion |
| AT Phase 3 | 25-30s | Precise deformation (λ₁→0.6, λ₂→0.9) |
| Post-AT | — | Synchronized landing |

## Generating Plots

```bash
python3 scripts/generate_paper_plots.py
```

Outputs are saved to `plots/`:
- `tracking_errors.pdf` - Leader and follower tracking performance
- `formation_trajectories.pdf` - 2D XY formation paths

## Control Parameters

| Parameter | Leaders | Followers |
|-----------|---------|-----------|
| K_p (x,y,z) | (2.5, 2.5, 4.0) | (3.5, 3.5, 5.0) |
| K_d (x,y,z) | (1.5, 1.5, 2.0) | (2.0, 2.0, 2.5) |
| Feedforward | Yes (from trajectory) | No (position-only) |

## Results

Tracking error statistics from experimental validation:

| Drone | Role | Mean Error | Max Error | RMSE |
|-------|------|------------|-----------|------|
| cf1 | Leader | 9.29 cm | 11.62 cm | 9.31 cm |
| cf2 | Leader | 9.08 cm | 11.40 cm | 9.09 cm |
| cf6 | Leader | 4.13 cm | 6.40 cm | 4.16 cm |
| cf3 | Follower | 10.79 cm | 16.57 cm | 10.93 cm |
| cf4 | Follower | 7.82 cm | 14.42 cm | 8.14 cm |
| cf5 | Follower | 11.09 cm | 16.15 cm | 11.24 cm |

## Citation

If you use this code, please cite our paper (BibTeX coming soon).

## Acknowledgments

This project uses [Crazyswarm2](https://imrclab.github.io/crazyswarm2/), a ROS 2 framework for Crazyflie quadrotors developed by the [IMRCLab](https://imrclab.github.io/) at TU Berlin.




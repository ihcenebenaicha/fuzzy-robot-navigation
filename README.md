# Fuzzy Robot Navigation

This repository contains the implementation of an improved fuzzy logic controller (FLC) for mobile robots navigating in unknown environments. The approach is based on:

- **Obstacle Avoidance:** A dedicated FLC ensures safe navigation around obstacles.
- **Goal Pursuit:** A separate FLC guides the robot toward a target.
- **Fusion and Prioritization:** A fusion block combines the outputs of the two controllers, with a direction prioritization mechanism.
- **Sub-goal Generation:** A sub-goal generator improves path planning in complex environments.

The code includes simulations with ROS and MATLAB and is structured for easy adaptation to real robot platforms.

## Features
- Fuzzy logic-based navigation
- Obstacle avoidance and goal-seeking behaviors
- Fusion mechanism with directional prioritization
- Sub-goal generation for complex paths
- Simulation-ready for ROS and MATLAB

## Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/ihcenebenaicha/fuzzy-robot-navigation.git
   
## Requirements
- **ROS1 (Noetic or Melodic recommended)**
- **Gazebo Simulator** (for virtual environment testing)
- **MATLAB** 
- OR a **real robot** with:
  - Laser sensor (LIDAR)
  - ROS1 interface

## Execution
To run the program, launch the main file named laser_FLC_IB_ros1.py. If the environment requires sub-goal generation, uncomment the section marked # sub goal generator in the main file

For further explanation, consult the article: An Improved Fuzzy Logic Controller for Mobile Robots Navigation in Unknown Environments. https://doi.org/10.1002/rob.70037

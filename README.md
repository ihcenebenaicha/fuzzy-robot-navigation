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

## Getting Started

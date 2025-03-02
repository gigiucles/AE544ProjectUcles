# Attitude Kinematics Simulation: Euler Angles vs. Quaternions

This repository contains a simulation to compare the performance of Euler angles and quaternions in attitude kinematics. The focus is on understanding how these two methods handle singularities and ambiguities in the context of numerical integration. The project explores the challenges of simulating attitude motion close to or at the singularity points, and investigates the impact of different numerical integrators on the singularity problem.

## Overview

Attitude kinematics describes the orientation of a rigid body in 3D space. Two popular methods to represent attitude are Euler angles and quaternions:

- **Euler Angles**: Represent rotations as a sequence of three angles corresponding to rotations about fixed or moving axes. Euler angles are subject to singularities at specific values, where the system loses degrees of freedom, such as \( \theta = \pm 90^\circ \) for the (3-2-1) set of Euler angles or \( \theta = 0^\circ \) and \( \theta = 180^\circ \) for the (3-1-3) set. This limitation can cause instability or ambiguity in large rotations.

- **Quaternions**: Provide a four-dimensional representation of rotations, avoiding the singularity problem by offering a continuous and smooth representation of rotations. However, quaternions have an inherent ambiguity: a quaternion and its inverse represent the same physical orientation.

This simulation explores these differences, especially near singularities, and compares the two representations by integrating their kinematic equations using different numerical integrators.

## Key Features

- **Euler Angles Singularities**: The simulation demonstrates the singularity behavior in Euler angles, showing how rotations close to \( \theta = \pm 90^\circ \) (for the (3-2-1) set) or \( \theta = 0^\circ / 180^\circ \) (for the (3-1-3) set) can cause numerical instability and ambiguity in attitude representation.

- **Quaternion Stability**: Unlike Euler angles, quaternions handle large rotations and motions near singularities without instability, providing a smoother, more robust representation.

- **Numerical Integrators**: The simulation compares three different numerical integrators:
  - `ode45` (variable step-size method)
  - `ode4` (fixed-step method)
  - `ode15s` (stiff integrator)

The performance of these integrators is evaluated to assess their impact on capturing the singularity and resolving ambiguities in both Euler angles and quaternions.

- **3D Animation**: A 3D animation is generated to visually analyze the attitude motion and its behavior near singularities for both Euler angles and quaternions. The animation shows smooth rotations for quaternions and potential abrupt changes for Euler angles when approaching singularities.

## Simulation Process

1. **Angular Motion Trace**: A representative angular motion trace is selected to create scenarios where the system approaches or passes through singularities. This motion is simulated using both Euler angles and quaternions.

2. **Integration**: The attitude kinematics equations for both Euler angles and quaternions are numerically integrated using the selected integrators (`ode45`, `ode4`, `ode15s`). The simulation captures the impact of these integrators on the behavior near singularities.

3. **Singularity and Ambiguity Analysis**: The results of the integration are compared, particularly looking for numerical instability or erratic behavior near singularities for Euler angles, and smooth, continuous motion for quaternions.

4. **3D Animation**: A 3D visualization is generated to illustrate how each representation handles rotations near critical points. The animation will provide insight into how Euler angles can experience gimbal lock or instability, while quaternions maintain a smooth rotation.

## Usage

1. **Run the simulation**: In MATLAB, open and run the script attitude_simulation.m. This will generate the necessary simulations and outputs for Euler angles and quaternions.
2. **Analyze the results**: The results will be displayed as numerical outputs and 3D animations. You can compare the stability of the Euler angle and quaternion representations, as well as the effects of different numerical integrators on the singularity problem.
3. **Adjust Parameters**: Modify the input angular motion trace or change the integrators to explore different scenarios or compare different conditions.

## Conclusion

This simulation highlights the limitations of Euler angles, especially their susceptibility to singularities, and the advantages of using quaternions for stable and smooth attitude kinematics. By comparing the two methods and analyzing their behavior near critical points, the project provides a deeper understanding of how numerical integrators can impact the representation of large rotations and singularity avoidance.

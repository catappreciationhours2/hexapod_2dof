# 12-Servo 2-DOF Hexapod Adaptation

# Table of Contents

* [Hexapod Project Description](#hexapod-project-description)
* [Hackberry Pi Adaptation](#hackberry-pi-adaptation)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
* [Authors](#authors)
* [Acknowledgements & Citations](#acknowledgements--citations)
* [Description of Each Script](#description-of-each-script)
  * [Rotation Matrices](#rotation-matrices)
  * [Body Functions](#body-functions)
  * [Leg Functions (12-Servo / 2-DOF)](#leg-functions)
  * [Movement Functions](#movement-functions)
  * [Movement Cycles](#movement-cycles)
  * [Model Visualization](#model-visualization)

# Hexapod Project Description

The goal of this project is to model and control a radially symmetrical hexapod (six-legged robot) capable of omnidirectional movement and independent body manipulation. This project includes a computational simulation used to verify inverse kinematics and a library of functions to drive a physical robot via serial commands.

# Hackberry Pi Adaptation
This repository contains a specialized adaptation developed for **Hackberry Pi (ACM@CMU)**. While the original design utilized an 18-servo configuration (3-DOF per leg), this version has been refactored to work with a **12-servo setup** (2-DOF per leg). 

In this architecture:
1. **Base Servos:** Handle left/right horizontal rotation (Yaw).
2. **Knee Servos:** Handle up/down vertical lifting (Pitch).
3. **Rigid Limbs:** The third joint (Tibia) is replaced with a fixed rigid segment, simplifying the weight and power requirements while maintaining highly effective walking and turning gaits.

## Getting Started

This repository contains the computational model for testing 2-DOF inverse kinematics. The core logic is contained within the Jupyter Notebook `Walking Model.ipynb`. This simulation verifies that the robot can maintain a level body while the feet follow the specific arc-trajectories required by fixed-length limbs.

### Prerequisites

[Python 3.8.12 or later](https://www.python.org/)
* Required libraries:
  * `Numpy`
  * `Matplotlib`
  * `Seaborn`
  * `Math`
  * `Time`

## Authors

* **Noella Horo** - *2-DOF Adaptation & Implementation*
* **Nabeel Chowdhury** - *Original 18-servo Computational Model*

## Acknowledgements & Citations

This project is an adaptation of the original Hexapod project by **Nabeel Chowdhury**. 
* **Original Source Code:** [Nabizzle/Hexapod](https://github.com/Nabizzle/Hexapod)
* **Design Inspiration:** Adapted from the [Capers II](https://www.instructables.com/Capers-II-a-Hexapod-Robot/) design by **Toglefritz**.
* **Kinematics Guidance:** Equations based on research by [Oscar Liang](https://oscarliang.com/inverse-kinematics-implementation-hexapod-robots/).

# Description of Each Script

## Rotation Matrices
These functions calculate the rotation of 3D points around primary axes to transform target coordinates into the local frame of each leg.
* **xRot / yRot / zRot**: Rotation about the X, Y, and Z axes respectively in degrees.

## Body Functions
* **bodyPos**: Generates the 3D hexagon model representing the chassis. It calculates the position of the six leg attachment points based on global pitch, roll, yaw, and translation.

## Leg Functions
This module has been refactored for the **12-servo (2-DOF)** kinematic chain.
* **legAngle (Inverse Kinematics)**: Calculates the necessary `Base` and `Knee` angles to reach a target coordinate. Unlike the 3-DOF version, this uses a 2D trigonometric approach to solve for the pitch of the rigid leg.
* **legPos (Forward Kinematics)**: Determines the 3D coordinates of the joints (Body, Knee, and Foot) based on current servo angles. 
* **startLegPos**: Initializes the hexapod in a neutral stance. The stance radius and height are optimized to ensure the 2-DOF limbs reach the ground plane effectively.
* **getFeetPos**: Returns the global contact points of each leg with the ground.
* **recalculateLegAngles**: Batch processes all six legs to find the required servo positions for a set of foot coordinates.
* **legModel**: Constructs the visual array of joint points required for 3D plotting.

## Movement Functions
* **stepForward**: Calculates the parabolic arc for a single linear step. One set of legs (tripod gait) lifts and moves to the target while the others remain grounded.
* **stepTurnFoot**: Calculates the circular arc required for a single foot to contribute to a body rotation.
* **stepTurn**: Coordinates all six feet to execute a step in a rotational turn cycle.

## Movement Cycles
* **walk**: Orchestrates the tripod gait. It breaks a target distance into manageable steps and generates the animation sequence.
* **turn**: Orchestrates the rotational gait, allowing the robot to change its heading.

## Model Visualization
* **showModel**: Renders the 3D Matplotlib plot. Updated to ensure the Z-axis limits correctly show the robot standing on the ground plane ($Z \approx -60$).
* **animate**: The frame-by-frame update function used for movement cycle animations.
* **animateBodyTranslate**: Demonstrates 6-DOF chassis movement while the feet remain locked in place.

---
*Built for Hackberry Pi 2026 - ACM@CMU*
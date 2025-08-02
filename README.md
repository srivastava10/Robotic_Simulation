# Robotic Arm Pick and Place Simulation in MATLAB & CoppeliaSim

This repository contains a MATLAB-based simulation of a UR5e robotic arm performing a complete pick-and-place task. The robot's motion is governed by a robust PID controller with a filtered derivative gain to ensure smooth and accurate trajectory tracking.

The entire project is built upon the principles and MATLAB software library from the book **"Modern Robotics: Mechanics, Planning, and Control" by Kevin M. Lynch and Frank C. Park**.

---

###  Project Showcase

*A demonstration of the UR5e robot executing the full pick-and-place trajectory, controlled via Feedforward+PID Controller.*
<img width="469" height="354" alt="Screenshot 2025-08-03 000550" src="https://github.com/user-attachments/assets/0b285dc2-7ce9-4004-ac6f-2209a33b66f4" />

##  Overview

The primary goal of this project is to simulate a UR5e manipulator executing a precise "pick and place" operation. The simulation demonstrates a complete cycle:
1.  The robot begins at a home configuration.
2.  It moves to a standoff position above a target block.
3.  It descends to grasp the block.
4.  It transports the block to the desired final location.
5.  It releases the block at the target destination.
6.  It opens the end-effector and moves it to a standoff configuration above the block. 

##  Control Strategy

The simulation is founded on the feedforward/feedback control concepts detailed in Chapter 11 and Chapter 13 of the "Modern Robotics" textbook.

### Trajectory Generation

The end-effector's path is generated using the `trajectoryGenerator.m` script. This function creates a smooth, continuous reference trajectory for the end-effector frame {e} by defining several key poses. The `ScrewTrajectoryMod` function modified from the *Modern Robotics* library is used to generate smooth, screw-motion-based segments between these keyframes.

##  Software and Dependencies

-   **MATLAB R2024b** or newer.
-   **CoppeliaSim Edu V4.1.0** or compatible version.
-   The MR code library .

---

##  Results
[Simulation of Four-Mecanum-Wheeled-Robot](https://www.youtube.com/watch?v=eXXvUIysoNU/)


[Simulation of Desired Trajectory that End-Effector Follows](https://www.youtube.com/watch?v=ueHek9R3Sz8/)



---

##  Acknowledgments

-   This project is a direct application of the knowledge and software from **Kevin M. Lynch and Frank C. Park**. Their book and open-source library are invaluable resources for the robotics community.
    -   [Modern Robotics](http://modernrobotics.org/)
-   Thanks to the developers of **CoppeliaSim** for creating a versatile and user-friendly platform for robotics simulation.

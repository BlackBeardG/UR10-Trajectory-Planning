# Trajectory Planning and Kinematic Control of a UR10 Robot for Door-Handle Manipulation

George Ioannidis, undergraduate student, ECE AUTH

## Overview

This project focuses on trajectory planning and kinematic control for a UR10 robotic manipulator performing a door-handle manipulation task.

The assignment is divided into two parts. Part A models the motion of a door and its handle, while Part B uses the generated end-effector trajectory to simulate the corresponding motion of a UR10 robot through inverse kinematics.

## Part A: Door and Handle Trajectory Planning

The first part generates a smooth trajectory for the door-handle system over a total duration of 5 seconds.

The motion is divided into three phases:

* the handle rotates from 0° to -45°
* the door opens from 0° to -30° while the handle remains fixed
* the handle returns from -45° to 0° while the door remains fixed

The trajectories are generated using 5th-degree polynomial interpolation, ensuring zero initial and final velocities and accelerations. Homogeneous transformation matrices are used to describe the pose of the door, handle and end-effector frames.

The orientation of the handle is also represented using unit quaternions in order to verify that the rotational motion remains smooth and continuous.

## Part B: UR10 Inverse Kinematics

The second part loads the end-effector trajectory produced in Part A and applies it to a UR10 robot model.

The end-effector velocity is computed from consecutive poses using spatial displacement. Then, the robot Jacobian is used to calculate the required joint velocities through the Jacobian pseudo-inverse method.

The joint velocities are numerically integrated over time to obtain the corresponding joint positions, allowing the UR10 robot to follow the desired end-effector trajectory.

## Results

The simulation shows that the generated trajectories are smooth and continuous. The door and handle motion follows the required sequence, while the UR10 joint positions and velocities remain continuous throughout the motion.

The final end-effector position and orientation errors are also computed to evaluate the accuracy of the trajectory tracking.

## Files

* `code/part_a_door_handle_trajectory.m`: generates the door, handle and end-effector trajectories
* `code/part_b_ur10_inverse_kinematics.m`: performs UR10 inverse kinematics and robot motion simulation
* `Project_Report.pdf`: project report

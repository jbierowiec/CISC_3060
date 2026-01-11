# CISC 3060 Introduction to Robotics Labs & Final Project

This repository contains all four labs and final project I have worked on with other students in relation to Introduction to Robotics.

## Labs

- Lab 1 — Waypoint Navigation & Control Parameter Analysis
  - Objective:
    - Implement and analyze a basic waypoint-following controller using ROS.
  - Summary:
    - In this lab, a ROS node was developed to navigate a robot through an arbitrary list of waypoints using proportional control. The system continuously adjusted linear and angular velocities until each waypoint was reached within a configurable termination threshold. The lab focused on analyzing how control parameters—such as termination threshold, angular gain, linear velocity, and sample rate—affect navigation accuracy, stability, and motion smoothness. Experimental results included accuracy plots, velocity profiles, and trajectory traces, providing insight into trade-offs between precision, speed, and oscillatory behavior in closed-loop robot motion.
- Lab 2 — Environment Scanning & Reactive Motion
  - Objective:
    - Use laser range data to sense the environment and make reactive navigation decisions.
  - Summary:
    - This lab introduced spatial perception using laser range measurements. The robot performed a full 360° rotation while sampling front-distance data, storing it in a histogram representing obstacle proximity as a function of orientation. Experiments were conducted at multiple locations within an enclosed environment to observe how surrounding geometry affected sensor readings. The histogram resolution was increased to improve directional precision, and a reactive behavior was implemented to identify the direction of maximum free space. The robot then rotated toward that direction and advanced forward, demonstrating a simple perception-driven navigation strategy.
- Lab 3 — Real-World TurtleBot Operation & Obstacle Avoidance
  - Objective:
    - Deploy and evaluate autonomous navigation behaviors on a physical TurtleBot3.
  - Summary:
    - This lab transitioned from simulation to real-world robotics using the TurtleBot3 platform. Students configured ROS communication, visualized sensor data in RViz, and evaluated laser scan readings in a physical enclosure. A wandering behavior previously developed in simulation was ported to the real robot and tuned for safe operation in confined spaces. Through systematic experimentation, parameters such as obstacle detection distance, angular velocity, and linear speed were adjusted to achieve smooth, collision-free motion. The lab emphasized sensor interpretation, real-world noise, and the differences between simulated and physical robot behavior.
- Lab 4 — Path Planning & Trajectory Optimization
  - Objective:
    - Plan and execute a collision-free path using an occupancy grid and A\* search.
  - Summary:
    - This lab explored autonomous path planning using an occupancy grid representation of the environment. An A\* search algorithm was used to compute an optimal path between a start and goal location while accounting for robot size via obstacle dilation. A follower node then guided the robot along the planned path by sequentially issuing intermediate goals. The lab investigated how the pursuit distance affected path-tracking accuracy and smoothness. An enhanced follower algorithm was implemented to detect straight-line segments and skip unnecessary intermediate waypoints, resulting in more efficient and natural robot motion.

## Final Project

- Final Project — Visual Navigation with Color Targets
  - Objective:
    - Design a complete autonomous system that combines vision-based navigation and obstacle avoidance.
  - Summary:
    - The final project required building a multi-node ROS system to autonomously locate and navigate to four colored targets (orange, white, yellow, and black) in a cluttered environment without prior knowledge of their locations. Computer vision techniques were used to detect targets via color thresholding and image moments, allowing the robot to estimate target position and distance. A dedicated obstacle-avoidance node modified velocity commands based on laser scan data to prevent collisions. Upon reaching each target, the robot recorded annotated images containing the goal number, robot position, and elapsed time. The project demonstrated integrated perception, planning, control, and system coordination in a competitive, real-time robotic task.

## Project Structure

```plaintext
├── lab_1
│   ├── CISC 3060 Lab Assignment 1.pdf
│   └── Introduction to Robotics Lab Report 1.pdf
├── lab_2
│   ├── CISC 3060 Lab Assignment 2.pdf
│   └── Introduction to Robotics Lab Report 2.pdf
├── lab_3
│   ├── CISC 3060 Lab Assignment 3.pdf
│   └── Introduction to Robotics Lab Report 3.pdf
├── lab_4
│   ├── CISC 3060 Lab Assignment 4.pdf
│   └── Introduction to Robotics Lab Report 4.pdf
└── final_project
    ├── CISC 3060 Class Project.pdf
    ├── Introduction to Robotics Final Project Report.pdf
    ├── Robotic System to Perform Obstacle Avoidance and Color Tracking.pdf
    ├── OA_final_project.py
    └── track_target_T3.py
```

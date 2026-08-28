# AMR Navigation, Vision Odometry & Intelligent Diagnostics

A simulation-based Autonomous Mobile Robot (AMR) project developed using ROS 2 Jazzy, Gazebo, RViz, and Navigation2.

The project focuses on autonomous navigation, real-time robot control, visual odometry, multi-sensor diagnostics, and automated fault analysis without requiring a physical robot.

## Current Status

* Ubuntu 24.04 development environment configured
* ROS 2 Jazzy installed
* Gazebo Harmonic simulation running
* RViz visualization configured
* TurtleBot3 simulation integrated with Nav2
* AMCL localization verified
* Autonomous goal navigation verified
* Nav2 lifecycle and velocity pipeline issues documented

## Project Goals

* Build a custom AMR simulation environment in Gazebo
* Implement autonomous navigation using Nav2
* Create and save maps using SLAM
* Integrate simulated LiDAR, IMU, wheel odometry, and camera sensors
* Implement downward-facing camera visual odometry
* Compare wheel odometry and visual odometry performance
* Detect sensor and navigation anomalies automatically
* Record ROS 2 data using rosbag for system analysis
* Develop intelligent diagnostics for rapid troubleshooting

## Technology Stack

* Ubuntu 24.04 LTS
* ROS 2 Jazzy
* Gazebo Harmonic
* RViz2
* Navigation2
* SLAM Toolbox
* OpenCV
* Python and C++

## Repository Structure

```text
amr-navigation-vision-diagnostics/
├── README.md
├── .gitignore
├── docs/
│   └── Nav2_Gazebo_Troubleshooting_TH.md
└── src/
    └── amr_bringup/
```

## Roadmap

* [x] Install and verify ROS 2 Jazzy
* [x] Run Gazebo and RViz
* [x] Verify Nav2 autonomous navigation
* [x] Document Nav2 lifecycle troubleshooting
* [ ] Create a custom ROS 2 bringup package
* [ ] Create a custom Gazebo world
* [ ] Implement SLAM and map saving
* [ ] Add simulated camera and visual odometry
* [ ] Implement sensor diagnostics
* [ ] Add automated testing and experiment results

## Documentation

Troubleshooting notes are available in the [`docs`](docs/) directory.

## Project Motivation

This project is designed as a robotics software portfolio project demonstrating practical experience with ROS 2, Nav2, Gazebo simulation, localization, real-time control, visual odometry, and intelligent diagnostics.

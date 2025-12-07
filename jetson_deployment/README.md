# Jetson Deployment Scripts

This directory contains placeholder scripts and instructions for deploying Isaac ROS and Nav2 components to an NVIDIA Jetson platform.

## Overview

Deploying ROS 2 applications, especially those leveraging hardware acceleration like Isaac ROS, to an embedded platform like NVIDIA Jetson requires careful management of dependencies, cross-compilation (if applicable), and file transfer. The scripts in this directory provide a starting point for automating this process.

## Contents

*   `deploy_isaac_ros.sh`: Script to deploy Isaac ROS related packages and dependencies.
*   `deploy_nav2.sh`: Script to deploy Nav2 related packages and configurations.

## Setup & Usage Instructions

Before using these scripts, you must:
1.  **Configure `JETSON_IP` and `JETSON_USERNAME`**: Edit the `.sh` scripts to set the correct IP address and username for your Jetson device.
2.  **SSH Access**: Ensure you have passwordless SSH access configured from your development machine to the Jetson (e.g., using SSH keys).
3.  **Cross-compilation (if needed)**: If you are developing on an x86 host and deploying to an ARM64 Jetson, you will need to perform cross-compilation. Ensure your ROS 2 workspace is built for the aarch64 target. This might involve using Docker containers with cross-compilation toolchains.
4.  **Build ROS 2 Workspace**: On your development machine, build your ROS 2 workspace that includes `isaac_ros_examples` and `nav2_bringup_humanoid` packages.

### Running Deployment

To deploy:
```bash
# Example for Isaac ROS
./scripts/deploy_isaac_ros.sh

# Example for Nav2
./scripts/deploy_nav2.sh
```

## Goal

The goal of these scripts is to streamline the process of transferring and configuring the AI-robot brain components onto the NVIDIA Jetson, enabling real-world testing and operation of the humanoid robot.

#!/bin/bash
# Script for deploying Nav2 components to NVIDIA Jetson
#
# This script is a placeholder. Actual deployment will involve:
# 1. Cross-compiling ROS 2 Nav2-related packages for ARM64 architecture if developed on x86.
# 2. Transferring the built workspace to the Jetson.
# 3. Installing Nav2 dependencies on the Jetson.
# 4. Sourcing the workspace and verifying basic functionality.

# --- Configuration ---
JETSON_IP="<JETSON_IP_ADDRESS>"
JETSON_USERNAME="<JETSON_USERNAME>"
WORKSPACE_DIR="/home/${JETSON_USERNAME}/ros2_ws" # Directory on Jetson

# --- Functions ---
log_info() {
    echo "[INFO] $1"
}

log_error() {
    echo "[ERROR] $1" >&2
}

# --- Main Deployment Logic ---
log_info "Starting Nav2 deployment to Jetson at ${JETSON_IP}..."

# 1. Cross-compilation (if needed)
log_info "Checking for cross-compilation requirements..."
if [ "$(uname -m)" = "x86_64" ]; then
    log_info "Detected x86_64 development machine. Cross-compilation may be required."
    log_info "Please ensure your ROS 2 workspace is built for aarch64 (Jetson) target."
    # Example command (requires specific setup like cross_build environment)
    # colcon build --target-platform=aarch64 --packages-select nav2_bringup_humanoid
else
    log_info "Detected ARM64 development machine (or direct build on Jetson). Skipping cross-compilation step."
fi

# 2. Transfer built workspace to Jetson
log_info "Transferring built ROS 2 workspace to Jetson..."
# Ensure 'install' directory exists and contains built packages
if [ -d "ros2_ws/install" ]; then
    rsync -avz --delete ros2_ws/install "${JETSON_USERNAME}@${JETSON_IP}:${WORKSPACE_DIR}/" || { log_error "Failed to transfer workspace!"; exit 1; }
    rsync -avz ros2_ws/src/nav2_bringup_humanoid "${JETSON_USERNAME}@${JETSON_IP}:${WORKSPACE_DIR}/src/" || { log_error "Failed to transfer Nav2 bringup source!"; exit 1; }
    log_info "Workspace transferred successfully."
else
    log_error "ros2_ws/install directory not found. Please build the workspace first."
    exit 1
fi

# 3. Install Nav2 dependencies on Jetson (via SSH)
log_info "Installing Nav2 dependencies on Jetson via SSH..."
ssh "${JETSON_USERNAME}@${JETSON_IP}" << EOF
    sudo apt update
    sudo apt install -y ros-humble-nav2-bringup # Install Nav2 stack
    # Any other specific dependencies for humanoid Nav2 configuration
    echo "Nav2 dependencies installed."
EOF
if [ $? -ne 0 ]; then log_error "Failed to install dependencies on Jetson!"; exit 1; fi

# 4. Verify basic functionality (via SSH)
log_info "Verifying basic Nav2 functionality on Jetson..."
ssh "${JETSON_USERNAME}@${JETSON_IP}" << EOF
    source /opt/ros/humble/setup.bash # Adjust ROS distro if needed
    source ${WORKSPACE_DIR}/install/setup.bash
    # Example verification: run a simple Nav2 launch file or check topic list
    # ros2 launch nav2_bringup_humanoid humanoid_nav2_bringup.launch.py
    # ros2 node list
    echo "Basic Nav2 components verification complete."
EOF
if [ $? -ne 0 ]; then log_error "Failed to verify functionality on Jetson!"; exit 1; fi

log_info "Nav2 deployment to Jetson completed successfully."
exit 0

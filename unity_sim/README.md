# Unity Simulation Project

This directory is intended to host a Unity project for high-fidelity rendering and human-robot interaction in the context of the "Digital Twin" module.

## Setup Instructions:

1.  **Initialize Unity Project**: Open Unity Hub and create a new 3D project, selecting this `unity_sim/` directory as the project location.
2.  **Install Necessary Packages**:
    *   **Unity Robotics Hub**: Follow the official Unity Robotics documentation to install the Robotics OS (ROS) packages for Unity. This typically involves adding packages from a Git URL.
    *   **ROS TCP Connector**: Install this package to enable communication between Unity and ROS 2.
    *   Any other relevant packages for visualization or interaction.

## Goal:

The goal of this project is to provide a visually rich simulation environment that can either run independently or be synchronized with Gazebo physics simulations, allowing for advanced human-robot interaction development.

## Further Setup: Robot Model Import and Synchronization

1.  **Import Robot Model**:
    *   Import your robot model (e.g., `simple_robot` from `gazebo_sim/models/`) into Unity. This can be done by importing the URDF/SDF directly using Unity Robotics tools or by importing a 3D asset (e.g., FBX, USD) if you have one.
    *   Ensure the model is correctly scaled and positioned in your Unity scene.
2.  **Synchronize with Gazebo (via ROS 2)**:
    *   **ROS TCP Connector Setup**: Configure the ROS TCP Connector within your Unity project to connect to your running ROS 2 system.
    *   **Joint State Synchronization**: Develop Unity scripts to subscribe to the `/joint_states` topic from ROS 2 (published by Gazebo) and update the corresponding joints of your robot model in Unity.
    *   **Command Publishing (Optional)**: If you intend to control the robot from Unity, develop scripts to publish commands (e.g., `/cmd_vel`) to ROS 2 topics.
3.  **Verification**:
    *   Launch your Gazebo simulation with the robot model.
    *   Run your Unity project.
    *   Verify that movements or changes in Gazebo are accurately reflected in the Unity visualization.

## Human-Robot Interaction (HRI) Scripts

1.  **Create Interaction Scripts**:
    *   Develop C# scripts in Unity for basic HRI. Examples include:
        *   **Button Presses**: Create UI buttons that, when pressed, publish a ROS 2 command (e.g., to move the robot, change its state).
        *   **UI Display**: Display robot status information (e.g., sensor readings, battery level, current task) from ROS 2 topics onto a Unity UI canvas.
        *   **Teleoperation**: Implement a simple joystick or keyboard control interface within Unity that publishes commands to the robot in Gazebo via ROS 2.
2.  **Integrate into Scene**:
    *   Attach these scripts to appropriate GameObjects in your Unity scene (e.g., a UI Canvas for buttons, the robot model for teleoperation).
    *   Ensure proper event handling and data flow between the Unity UI/controls and the ROS 2 communication layer.
3.  **Verification**:
    *   Run your Gazebo and Unity simulations.
    *   Test the interactive elements in Unity and observe the corresponding robot behavior in both Unity and Gazebo (if synchronized).
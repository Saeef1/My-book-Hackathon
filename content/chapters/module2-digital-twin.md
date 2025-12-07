---
title: "Module 2: The Digital Twin (Gazebo & Unity)"
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Introduction
This module delves into the creation and utilization of digital twins for robotics, leveraging Gazebo for physics-based simulation and Unity for high-fidelity rendering and human-robot interaction. You will learn to build virtual environments, simulate physical phenomena like gravity and collisions, and integrate various sensor models to replicate real-world robotic scenarios.

## Section 1: Gazebo Physics Simulation
Gazebo is a powerful 3D robotics simulator that accurately simulates rigid-body physics, sensor data generation, and various environmental factors. This section will guide you through creating custom worlds, defining robot models using URDF/SDF, and configuring physics properties to achieve realistic robotic behavior.

### Basic Gazebo World Setup

A Gazebo world defines the environment in which your robot will operate. It can include static objects, lighting, and physics properties. Below is a basic `empty.world` file that includes a sun (light source) and a ground plane. We've also added a simple robot and a falling box to demonstrate gravity and collision.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="empty_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="falling_box">
      <pose>0 0 2 0 0 0</pose> <!-- Position the box 2 meters above the ground -->
      <link name="box_link">
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
        <visual>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <include>
      <uri>model://simple_robot</uri>
      <pose>0 0 0.1 0 0 0</pose> <!-- Position the robot on the ground plane -->
    </include>

  </world>
</sdf>
```
**Explanation:**
*   `include` tags are used to bring in pre-defined Gazebo models like the `sun` and `ground_plane`.
*   A `falling_box` model is defined with a `mass` and a `pose` (position and orientation). Its `visual` and `collision` properties define its appearance and physical interaction.
*   The `simple_robot` is also included, which we will define in the next section.

### Simple Robot Model (SDF)

The Simulation Description Format (SDF) is an XML format for describing robots and environments for simulators like Gazebo. Here's a basic `model.sdf` for our `simple_robot`, which is a box with a base link, and will be extended with sensors.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_robot">
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual>
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```
**Explanation:**
*   A `model` named `simple_robot` contains a `link` named `base_link`.
*   `inertial` properties define the mass and inertia, crucial for physics simulation.
*   `visual` defines the graphical representation (a green box).
*   `collision` defines the physical shape for collision detection.

### Physics Engine Configuration

Gazebo uses a physics engine (e.g., ODE, bullet, DART, Simbody) to simulate interactions. You can configure global physics properties in your world file, such as gravity, time step, and solver parameters. For most scenarios, the default physics engine settings are sufficient, but for advanced simulations, tuning these parameters is essential for accuracy and stability.


## Section 2: Unity High-Fidelity Rendering and Interaction
Unity, a versatile real-time 3D development platform, offers advanced rendering capabilities and robust tools for creating interactive environments. This section will focus on importing robot models, setting up visually rich scenes, and developing human-robot interaction interfaces within a high-fidelity simulation.

### Unity Project Setup

To get started with Unity for high-fidelity rendering and human-robot interaction, you need to set up a new Unity project.

1.  **Initialize Unity Project**:
    *   Open Unity Hub and create a new 3D project, selecting your `unity_sim/` directory as the project location.
    *   Give your project a meaningful name (e.g., "DigitalTwinUnitySim").
2.  **Install Necessary Packages**:
    *   **Unity Robotics Hub**: This package provides essential tools and interfaces for connecting Unity with ROS 2. Follow the official Unity Robotics documentation for installation, which typically involves adding packages from a Git URL in Unity's Package Manager.
    *   **ROS TCP Connector**: Install this package to enable real-time communication between your Unity simulation and your ROS 2 system. This is crucial for synchronizing robot states and sending commands.
    *   **Other Relevant Packages**: Depending on your specific needs, you might install additional Unity packages for advanced visualization, UI elements, or specialized interactions.

### Robot Model Import and Synchronization

Once your Unity project is set up, the next step is to import your robot model and establish synchronization with your Gazebo physics simulation (if applicable).

1.  **Import Robot Model**:
    *   Import your robot model (e.g., the `simple_robot` from `gazebo_sim/models/`) into Unity. Unity Robotics packages often provide tools to directly import URDF/SDF files, converting them into Unity GameObjects. Alternatively, you can import 3D asset files (e.g., FBX, USD) if you have them.
    *   Ensure the model is correctly scaled, positioned, and oriented within your Unity scene.
2.  **Synchronize with Gazebo (via ROS 2)**:
    *   **ROS TCP Connector Configuration**: Configure the ROS TCP Connector within your Unity project to connect to your running ROS 2 system. This involves specifying the ROS 2 master URI and other connection settings.
    *   **Joint State Synchronization**: Develop C# scripts in Unity to subscribe to the `/joint_states` topic from ROS 2 (which is typically published by Gazebo or a robot state publisher). These scripts will then update the corresponding joint angles or positions of your robot model in Unity, ensuring visual consistency with the physics simulation.
    *   **Command Publishing (Optional)**: If you intend to control the robot from Unity (e.g., via a UI), develop C# scripts to publish commands (e.g., `geometry_msgs/msg/Twist` for `/cmd_vel`) to ROS 2 topics.
3.  **Verification**:
    *   Launch your Gazebo simulation with the robot model.
    *   Run your Unity project.
    *   Verify that movements or changes in Gazebo are accurately reflected in the Unity visualization.

### Human-Robot Interaction (HRI) Scripts

Unity's robust UI system and scripting capabilities make it an excellent platform for developing human-robot interaction interfaces.

1.  **Create Interaction Scripts**:
    *   Develop C# scripts in Unity for basic HRI elements. Examples include:
        *   **Button Presses**: Create interactive UI buttons that, when pressed, trigger specific robot actions by publishing ROS 2 messages or calling ROS 2 services. For instance, a "Start Mission" button could send a goal to a ROS 2 action server.
        *   **UI Display**: Implement a dynamic UI to display real-time robot status information. This could involve subscribing to ROS 2 topics (e.g., sensor readings, battery level, current task status) and updating Unity UI elements like text fields, sliders, or gauges.
        *   **Teleoperation Interface**: Develop a simple joystick, keyboard, or touch-based control interface within Unity that publishes robot commands (e.g., `/cmd_vel`) to ROS 2 topics, allowing for intuitive remote control.
2.  **Integrate into Scene**:
    *   Attach these scripts to appropriate GameObjects in your Unity scene. For UI elements, they typically attach to UI Canvas GameObjects. For robot controls, they might attach to a dedicated control GameObject or the robot model itself.
    *   Ensure proper event handling and data flow between the Unity UI/controls and the ROS 2 communication layer.
3.  **Verification**:
    *   Run your Gazebo and Unity simulations concurrently.
    *   Test the interactive elements in Unity (e.g., click a button, use the teleoperation interface) and observe the corresponding robot behavior in both the Unity visualization and the Gazebo physics simulation.

### Unity Project Setup

To get started with Unity for high-fidelity rendering and human-robot interaction, you need to set up a new Unity project.

1.  **Initialize Unity Project**:
    *   Open Unity Hub and create a new 3D project, selecting your `unity_sim/` directory as the project location.
    *   Give your project a meaningful name (e.g., "DigitalTwinUnitySim").
2.  **Install Necessary Packages**:
    *   **Unity Robotics Hub**: This package provides essential tools and interfaces for connecting Unity with ROS 2. Follow the official Unity Robotics documentation for installation, which typically involves adding packages from a Git URL in Unity's Package Manager.
    *   **ROS TCP Connector**: Install this package to enable real-time communication between your Unity simulation and your ROS 2 system. This is crucial for synchronizing robot states and sending commands.
    *   **Other Relevant Packages**: Depending on your specific needs, you might install additional Unity packages for advanced visualization, UI elements, or specialized interactions.

### Robot Model Import and Synchronization

Once your Unity project is set up, the next step is to import your robot model and establish synchronization with your Gazebo physics simulation (if applicable).

1.  **Import Robot Model**:
    *   Import your robot model (e.g., the `simple_robot` from `gazebo_sim/models/`) into Unity. Unity Robotics packages often provide tools to directly import URDF/SDF files, converting them into Unity GameObjects. Alternatively, you can import 3D asset files (e.g., FBX, USD) if you have them.
    *   Ensure the model is correctly scaled, positioned, and oriented within your Unity scene.
2.  **Synchronize with Gazebo (via ROS 2)**:
    *   **ROS TCP Connector Configuration**: Configure the ROS TCP Connector within your Unity project to connect to your running ROS 2 system. This involves specifying the ROS 2 master URI and other connection settings.
    *   **Joint State Synchronization**: Develop C# scripts in Unity to subscribe to the `/joint_states` topic from ROS 2 (which is typically published by Gazebo or a robot state publisher). These scripts will then update the corresponding joint angles or positions of your robot model in Unity, ensuring visual consistency with the physics simulation.
    *   **Command Publishing (Optional)**: If you intend to control the robot from Unity (e.g., via a UI), develop C# scripts to publish commands (e.g., `geometry_msgs/msg/Twist` for `/cmd_vel`) to ROS 2 topics.
3.  **Verification**:
    *   Launch your Gazebo simulation with the robot model.
    *   Run your Unity project.
    *   Verify that movements or changes in Gazebo are accurately reflected in the Unity visualization.

### Human-Robot Interaction (HRI) Scripts

Unity's robust UI system and scripting capabilities make it an excellent platform for developing human-robot interaction interfaces.

1.  **Create Interaction Scripts**:
    *   Develop C# scripts in Unity for basic HRI elements. Examples include:
        *   **Button Presses**: Create interactive UI buttons that, when pressed, trigger specific robot actions by publishing ROS 2 messages or calling ROS 2 services. For instance, a "Start Mission" button could send a goal to a ROS 2 action server.
        *   **UI Display**: Implement a dynamic UI to display real-time robot status information. This could involve subscribing to ROS 2 topics (e.g., sensor readings, battery level, current task status) and updating Unity UI elements like text fields, sliders, or gauges.
        *   **Teleoperation Interface**: Develop a simple joystick, keyboard, or touch-based control interface within Unity that publishes robot commands (e.g., `/cmd_vel`) to ROS 2 topics, allowing for intuitive remote control.
2.  **Integrate into Scene**:
    *   Attach these scripts to appropriate GameObjects in your Unity scene. For UI elements, they typically attach to UI Canvas GameObjects. For robot controls, they might attach to a dedicated control GameObject or the robot model itself.
    *   Ensure proper event handling and data flow between the Unity UI/controls and the ROS 2 communication layer.
3.  **Verification**:
    *   Run your Gazebo and Unity simulations concurrently.
    *   Test the interactive elements in Unity (e.g., click a button, use the teleoperation interface) and observe the corresponding robot behavior in both the Unity visualization and the Gazebo physics simulation.

## Section 3: Simulating Sensors
Accurate sensor simulation is crucial for developing and testing robotic perception algorithms. This section will cover the integration and configuration of simulated sensors such as LiDAR, depth cameras, and Inertial Measurement Units (IMUs) within both Gazebo and Unity, and how to bridge their data to ROS 2.

### Integrating Sensors into the Robot Model (SDF)

We will extend our `simple_robot` model to include a LiDAR, a Depth Camera, and an IMU. These sensors are defined as additional links and plugins within the `model.sdf` file.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_robot">
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual>
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name="lidar_link">
      <pose>0.1 0 0.1 0 0 0</pose> <!-- Position relative to base_link -->
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="base_to_lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>

    <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_laser.so">
      <topicName>/scan</topicName>
      <frameName>lidar_link</frameName>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
      <range>
        <min>0.08</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
      <ros>
        <remapping>~/out:=/scan</remapping>
      </ros>
    </plugin>

    <link name="camera_link">
      <pose>0 0.1 0.1 0 0 0</pose> <!-- Position relative to base_link -->
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual>
        <geometry>
          <box>
            <size>0.02 0.05 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
        </material>
      </visual>
      <collision>
        <geometry>
          <box>
            <size>0.02 0.05 0.02</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="base_to_camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>

    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
      <hackBaseline>0.07</hackBaseline>
      <distortionK1>0.0</distortionK1>
      <distortionK2>0.0</distortionK2>
      <distortionK3>0.0</distortionK3>
      <distortionT1>0.0</distortionT1>
      <distortionT2>0.0</distortionT2>
      <Cx>320</Cx>
      <Cy>240</Cy>
      <Fx>277.1935</Fx>
      <Fy>277.1935</Fy>
      <ros>
        <namespace></namespace>
        <remapping>~/image_raw:=/camera/image_raw</remapping>
        <remapping>~/camera_info:=/camera/camera_info</remapping>
      </ros>
    </plugin>

    <link name="imu_link">
      <pose>0 0 0.1 0 0 0</pose> <!-- Position relative to base_link -->
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000001</iyy>
          <iyz>0</iyz>
          <izz>0.000001</izz>
        </inertia>
      </inertial>
      <visual>
        <geometry>
          <box>
            <size>0.01 0.01 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 1 1</ambient>
          <diffuse>1 0 1 1</diffuse>
        </material>
      </visual>
      <collision>
        <geometry>
          <box>
            <size>0.01 0.01 0.01</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="base_to_imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
    </joint>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace></namespace>
        <remapping>~/out:=/imu/data</remapping>
      </ros>
      <initialOrientationAsReference>false</initialOrientationAsReference>
      <update_rate>100.0</update_rate>
      <imu>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
        <gyroscope>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </gyroscope>
        <accelerometer>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </accelerometer>
      </imu>
    </plugin>
  </model>
</sdf>
```
**Explanation:**
*   **LiDAR Sensor**: The `lidar_link` and associated `gazebo_ros_laser` plugin simulate a 2D LiDAR. It publishes `LaserScan` messages on the `/scan` topic.
*   **Depth Camera Sensor**: The `camera_link` and `gazebo_ros_camera` plugin simulate a camera that publishes `Image` and `CameraInfo` messages on `/camera/image_raw` and `/camera/camera_info` respectively.
*   **IMU Sensor**: The `imu_link` and `gazebo_ros_imu_sensor` plugin simulate an Inertial Measurement Unit, publishing `Imu` messages on `/imu/data`.

### Launching Gazebo with the Robot Model and Sensors

To launch Gazebo with our custom world and robot model, we use a ROS 2 launch file. This simplifies the process and allows for easy configuration.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0],
            'share', 'gazebo_ros', 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': LaunchConfiguration('world_file_path')}.items(),
    )

    # Spawn robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'simple_robot',
                                   '-topic', 'robot_description',
                                   '-x', '0', '-y', '0', '-z', '0.2'],
                        output='screen')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': '<robot><link name="world"/><joint name="world_fixed" type="fixed"><parent link="world"/><child link="base_link"/></joint></robot>'}], # Minimal URDF
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file_path',
            default_value=[os.path.join(os.getenv('HOME'), 'my-book', 'gazebo_sim', 'worlds', 'empty.world')],
            description='Path to the Gazebo world file'
        ),
        gazebo,
        spawn_entity,
        robot_state_publisher,
    ])
```

**Run Instruction:**
To launch the simulation, first make sure your ROS 2 workspace is built and sourced:
```bash
# From ros2_ws/
colcon build --packages-select sim_bringup
source install/setup.bash
ros2 launch sim_bringup gazebo.launch.py
```
This will launch Gazebo with the `empty_world` and spawn the `simple_robot` with its sensors.

### Verifying Sensor Data

Once the simulation is running, you can verify that the sensors are publishing data using ROS 2 command-line tools:

*   **LiDAR**:
    ```bash
    ros2 topic echo /scan
    ```
*   **Camera**:
    ```bash
    ros2 topic echo /camera/image_raw
    ```
*   **IMU**:
    ```bash
    ros2 topic echo /imu/data
    ```
You can also visualize the sensor data in RViz2:
```bash
rviz2
```
In RViz2, add a `LaserScan` display for `/scan`, an `Image` display for `/camera/image_raw`, and an `IMU` display for `/imu/data`.

## Examples
This section will provide hands-on examples for setting up Gazebo worlds, creating robot models, integrating sensors, and building interactive Unity scenes.

## Conclusion
By completing this module, you will have a comprehensive understanding of how to build and leverage digital twins using Gazebo and Unity. These skills are fundamental for accelerating robotics development, enabling extensive testing and iteration in a safe and cost-effective virtual environment before deployment to physical robots.

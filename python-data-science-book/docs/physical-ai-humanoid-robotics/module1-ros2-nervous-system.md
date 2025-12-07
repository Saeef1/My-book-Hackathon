---
sidebar_position: 2
---

# Module 1: The Robotic Nervous System (ROS 2)

## Focus
This module introduces ROS 2, the middleware that acts as the central nervous system for modern robots. We will cover the fundamental concepts that enable communication and data transfer between different parts of a robotic system.

---

### ROS 2 Nodes, Topics, and Services

In ROS 2, a robot's software is organized into a network of **Nodes**. Each node is a process that performs a specific task, such as controlling a motor, reading a sensor, or planning a path.

Nodes communicate with each other using three primary mechanisms:

1.  **Topics:** Topics are a publish/subscribe system for continuous data streams. For example, a camera node might publish a stream of images to a `/camera/image_raw` topic, and a computer vision node can subscribe to this topic to process the images.

    ```python
    # Example of a simple publisher in rclpy
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MinimalPublisher(Node):
        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello World: {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1
    ```

2.  **Services:** Services are used for request/response communication. A client node sends a request to a service server, which processes the request and sends back a response. This is useful for tasks that have a clear beginning and end, like "get the robot's current position".

3.  **Actions:** Actions are for long-running tasks that provide feedback. For example, telling a robot to navigate to a specific location is an action. The robot can provide feedback on its progress (e.g., distance to the goal) and a final result (e.g., success or failure).

### Bridging Python Agents to ROS Controllers with `rclpy`

`rclpy` is the Python client library for ROS 2. It allows you to write ROS 2 nodes in Python, making it easy to integrate high-level AI and machine learning code (often written in Python) with the lower-level robot controllers (which might be written in C++).

A common pattern is to have a Python-based "agent" or "brain" node that makes high-level decisions and communicates with other ROS 2 nodes that control the robot's hardware. For instance, a Python agent might use a service to call a "Master Control Program" (MCP) to execute a complex behavior.

```python
# Example of a simple service client in rclpy
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
```

### Understanding URDF for Humanoids

**URDF (Unified Robot Description Format)** is an XML format used to describe all the physical elements of a robot. This includes:

*   **Links:** The rigid parts of the robot (e.g., the torso, upper arm, forearm).
*   **Joints:** The connections between links, which define how they can move relative to each other (e.g., revolute, prismatic, fixed).
*   **Visuals:** The 3D models used to represent the robot in simulators like Gazebo and visualization tools like RViz2.
*   **Collisions:** The simplified shapes used by the physics engine to calculate collisions.
*   **Inertia:** The mass and rotational inertia of each link.

For a humanoid robot, the URDF file is typically very complex, defining dozens of links and joints for the legs, arms, torso, and head. A properly defined URDF is essential for simulation, path planning, and control.

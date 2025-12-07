# ROS 2 Research

## Best Practices for ROS 2 Nodes, Topics, Services, and Actions

This document summarizes best practices for developing robust and efficient ROS 2 applications.

### Nodes

- **Single Responsibility Principle**: Each node should have a single, well-defined purpose. This improves modularity, testability, and reusability.
- **Node Composition**: For performance-critical applications, use node composition to run multiple nodes in a single process. This enables intra-process communication, which avoids message serialization and reduces CPU and memory overhead.
- **Lifecycle Management**: Implement node lifecycle management (Managed Nodes) to control the state of your nodes (e.g., configuring, activating, deactivating, cleaning up). This is crucial for creating robust and predictable systems.

### Topics

- **Use for Continuous Data Streams**: Topics are ideal for continuous data streams, such as sensor data, robot state, and diagnostics.
- **Prefer Standard Messages**: Use standard message types (e.g., `sensor_msgs`, `geometry_msgs`) whenever possible to ensure interoperability with other ROS 2 packages and tools.
- **Granular Topics**: Publish specific data on its own topic. If a consumer needs multiple pieces of data, it can subscribe to multiple topics and use a message filter to synchronize them.
- **Quality of Service (QoS)**: Carefully select QoS policies for each publisher and subscriber to handle network imperfections. For sensor data, a `Best Effort` policy might be acceptable, while for commands, a `Reliable` policy is usually required.

### Services

- **Use for Quick, Synchronous Requests**: Services are designed for request-response interactions where a quick response is expected. They are suitable for tasks like querying the state of a node or triggering a short computation.
- **Avoid Long-Running Operations**: Do not use services for long-running tasks. A long-running service will block the client and can lead to a non-responsive system.
- **Stateless Operations**: A service call should ideally not change the state of the node providing the service. This helps to avoid unexpected side effects.

### Actions

- **Use for Long-Running, Asynchronous Tasks**: Actions are perfect for long-running tasks that need to provide feedback and can be preempted (canceled). Examples include navigation, manipulation, and complex data processing.
- **Provide Feedback**: The action server should provide regular feedback to the action client about the progress of the goal.
- **Handle Cancellation**: The action server must handle goal cancellation requests from the client. This allows for controlled termination of tasks.

## Docusaurus Integration

- **Code Blocks**: Use standard Markdown code blocks with language identifiers for syntax highlighting (e.g., ` ```python`).
- **Callouts**: Docusaurus supports callouts (admonitions) to highlight information. The syntax is `:::note`, `:::tip`, `:::info`, `:::warning`, `:::danger`.
- **Run Instructions**: Custom comments or annotations can be used within code blocks to provide execution instructions, which can be styled with CSS.

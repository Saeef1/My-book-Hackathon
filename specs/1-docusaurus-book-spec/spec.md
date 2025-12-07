# Feature Specification: Module 1: ROS 2 Nodes & Graph

**Feature Branch**: `1-spec-generation`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "if files aren't meeting this requirements change it \"\"\"\nTASK: Produce a feature specification for: Module 1\n(Replace Module 1 with one of: \"Module 1\", \"Module 2\", \"Module 3\", \"Module 4\", \"Capstone\", or a specific chapter like \"Module 1: ROS 2 Nodes & Graph\")\n\nSPEC REQUIREMENTS (exact subsections required, in this order):\n1. Title & Identifier (slug-friendly)\n2. Short summary (1–2 paragraphs)\n3. Problem Statement (what gap this module fills)\n4. Goals & Learning Outcomes (numbered list; align with context7 learning outcomes)\n5. Inputs (MCP context7 items, datasets, hardware, cloud resources)\n6. Actors & Roles (student, instructor, CI system)\n7. Non-functional Requirements (performance, compatibility, reproducibility)\n8. Acceptance Criteria (measurable pass/fail checks — unit tests, simulation tests)\n9. Data & API Contracts (ROS 2 topics/actions/services used; message types)\n10. Implementation Constraints (OS, ROS 2 versions, GPU/Jetson limitations)\n11. Docusaurus Page Layout (frontmatter, sections, code examples, callouts)\n12. Test Plan / CI Steps (automated checks, smoke tests)\n13. Dependencies (other modules/specs)\n14. Deliverables (filenames, paths, expected markdown filenames)\n15. Revision Notes template\n\nEXTRA:\n- For ROS/Gazebo/Isaac items include exact package names or placeholders from context7 (do not invent new packages).\n- For acceptance criteria include at least one runnable check (e.g., `ros2 run <pkg> <node> --ros-args` or simulation headless check).\n\nOUTPUT:\n- Produce the SPEC.md content in markdown.\n- If called for a module list, produce one SPEC.md per module in the same output separated by `---`.\n\"\"\"\n"

## Title & Identifier (slug-friendly)
Module 1: ROS 2 Nodes & Graph

## Short summary (1–2 paragraphs)
This feature specification outlines the requirements for "Module 1: ROS 2 Nodes & Graph," a foundational chapter for a textbook on Physical AI & Humanoid Robotics. It focuses on enabling students to understand and implement core ROS 2 concepts, including nodes, topics, services, and actions, and how they form a computational graph for robotic applications.

The module aims to provide clear, hands-on examples that solidify understanding of ROS 2's distributed architecture and communication mechanisms, preparing learners for more advanced topics in robotics.

## Problem Statement (what gap this module fills)
Many beginners struggle with understanding the fundamental building blocks and communication paradigms of ROS 2. This module fills the gap by providing a structured, step-by-step specification for a chapter that demystifies ROS 2 nodes and the computational graph, offering practical examples to build a strong foundation for robotic software development.

## Goals & Learning Outcomes (numbered list; align with context7 learning outcomes)
1.  Understand the concept of a ROS 2 node and its role in a robotic system.
2.  Be able to create and launch ROS 2 nodes in Python and C++.
3.  Comprehend ROS 2 topics for asynchronous, many-to-many communication.
4.  Implement ROS 2 publishers and subscribers for message exchange.
5.  Understand ROS 2 services for synchronous, request-response communication.
6.  Implement ROS 2 service clients and servers.
7.  Grasp the concept of ROS 2 actions for long-running, goal-oriented tasks.
8.  Bridge Python agents to ROS controllers using `rclpy`.
9.  Understand URDF (Unified Robot Description Format) for humanoids.
10. Be able to identify and visualize the ROS 2 computational graph using tools like `rqt_graph`.

## Inputs (MCP context7 items, datasets, hardware, cloud resources)
*   **Software**: ROS 2 (Humble/Iron recommended), Ubuntu 22.04 LTS, VS Code or similar IDE.
*   **Hardware (Optional for advanced exercises)**: NVIDIA Jetson platform, common robotic sensors (e.g., LiDAR, camera).
*   **Cloud Resources (Optional)**: Cloud-based simulation environments (e.g., AWS RoboMaker) for scalability testing.

## Actors & Roles (student, instructor, CI system)
*   **Student**: Engages with the module content, performs exercises, and completes assignments.
*   **Instructor**: Guides students, answers questions, and evaluates understanding.
*   **CI System**: Automates testing of code examples and exercises for correctness and adherence to ROS 2 best practices.

## Non-functional Requirements (performance, compatibility, reproducibility)
*   **Compatibility**: All code examples and exercises MUST be compatible with ROS 2 Humble Hawksbill and Iron Irwini on Ubuntu 22.04 LTS.
*   **Reproducibility**: All examples and exercises MUST be reproducible on standard development machines with the specified software stack. Docker/containerization support SHOULD be provided for easy setup.
*   **Clarity**: Explanations MUST be clear, concise, and easy to understand for beginners in robotics.
*   **Performance (Code Examples)**: Code examples should demonstrate efficient ROS 2 communication patterns, avoiding unnecessary overhead.

## Acceptance Criteria (measurable pass/fail checks — unit tests, simulation tests)
*   **AC-001**: Students can successfully create and run a basic ROS 2 publisher-subscriber pair in both Python and C++.
    *   **Runnable Check**: `ros2 run <publisher_pkg> <publisher_node> && ros2 run <subscriber_pkg> <subscriber_node>` (verify message receipt)
*   **AC-002**: Students can successfully create and use a ROS 2 service client-server pair.
*   **AC-003**: Students can visualize the ROS 2 computational graph with `rqt_graph` and correctly identify nodes, topics, and services.
*   **AC-004**: All provided code examples compile and execute without errors on the specified ROS 2 distributions.

## Data & API Contracts (ROS 2 topics/actions/services used; message types)
*   **Topics**:
    *   `/chatter` (`std_msgs/msg/String`)
    *   `/cmd_vel` (`geometry_msgs/msg/Twist`)
    *   `/scan` (`sensor_msgs/msg/LaserScan`)
*   **Services**:
    *   `/add_two_ints` (`example_interfaces/srv/AddTwoInts`)
*   **Actions**:
    *   `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)

## Implementation Constraints (OS, ROS 2 versions, GPU/Jetson limitations)
*   **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish).
*   **ROS 2 Version**: Humble Hawksbill or Iron Irwini.
*   **Programming Languages**: Python 3 and C++.
*   **Hardware (for advanced simulations)**: NVIDIA Jetson Nano/Xavier for on-robot deployment exercises; desktop GPU for Gazebo/Isaac Sim.

## Docusaurus Page Layout (frontmatter, sections, code examples, callouts)
The module's Docusaurus page MUST follow the constitution's Module & Chapter Format. It MUST include a YAML frontmatter, clear section headings, and code examples with `run-instruction` comments as per the constitution's Code & Example Rules. Callouts (e.g., `:::note`, `:::warning`) SHOULD be used to highlight important information or potential pitfalls.

## Test Plan / CI Steps (automated checks, smoke tests)
*   **Automated Checks**:
    *   Linting (ament_cpplint, black, flake8) for all code examples.
    *   ROS 2 package compilation (`colcon build`).
    *   Unit tests for core logic within nodes (using `gtest` for C++ and `pytest` for Python).
*   **Smoke Tests**:
    *   Launch basic publisher/subscriber nodes and verify message flow.
    *   Run service client/server examples and verify correct response.
    *   Launch `rqt_graph` in CI to verify graph structure if possible in a headless environment.

## Dependencies (other modules/specs)
*   None (this is a foundational module).

## Deliverables (filenames, paths, expected markdown filenames)
*   `specs/1-spec-generation/spec.md` (this document)
*   `specs/1-spec-generation/checklists/requirements.md` (spec quality checklist)
*   `content/chapters/module1-ros2-nodes-graph.md` (Docusaurus markdown content)
*   `ros2_ws/src/publisher_pkg/` (example ROS 2 publisher package)
*   `ros2_ws/src/subscriber_pkg/` (example ROS 2 subscriber package)
*   `ros2_ws/src/add_two_ints_server/` (example ROS 2 service server package)
*   `ros2_ws/src/add_two_ints_client/` (example ROS 2 service client package)

## Revision Notes template
```markdown
# Revision History for Module 1: ROS 2 Nodes & Graph

**Version**: [e.g., 1.0.0]
**Date**: [YYYY-MM-DD]
**Author**: [Author Name]
**Changes**:
*   [Brief description of changes, e.g., "Initial draft"]
*   [Another change]
```
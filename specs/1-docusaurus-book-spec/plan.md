# Implementation Plan: Module 1: ROS 2 Nodes & Graph

**Branch**: `1-docusaurus-book-spec` | **Date**: 2025-12-06 | **Spec**: [specs/1-docusaurus-book-spec/spec.md](specs/1-docusaurus-book-spec/spec.md)
**Input**: Feature specification from `/specs/1-docusaurus-book-spec/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation strategy for "Module 1: ROS 2 Nodes & Graph," a foundational chapter for a Physical AI & Humanoid Robotics textbook. The primary goal is to develop comprehensive Docusaurus-compatible content and runnable ROS 2 code examples that clarify core ROS 2 concepts: nodes, topics, services, and actions. The technical approach involves creating ROS 2 packages in Python and C++, integrating CI for code quality and testing, and establishing validation steps for both simulated and physical hardware environments to ensure high-quality, reproducible learning material.

## Technical Context

**Language/Version**: Python 3, C++, ROS 2 Humble Hawksbill/Iron Irwini
**Primary Dependencies**: ROS 2 packages (e.g., `rclpy`, `rclcpp`, `std_msgs`, `geometry_msgs`, `example_interfaces`, `nav2_msgs`), `ament_cmake`, `ament_python`
**Storage**: Filesystem for Docusaurus markdown content, ROS 2 packages, and assets
**Testing**: `pytest` (Python), `gtest` (C++), `ament_cpplint`, `black`, `flake8` for linting
**Target Platform**: Ubuntu 22.04 LTS, NVIDIA Jetson Nano/Xavier, desktop GPU for simulation
**Project Type**: Educational module within a Docusaurus static site, involving ROS 2 workspace.
**Performance Goals**: Code examples should run efficiently within typical ROS 2 development environments without significant latency for basic operations.
**Constraints**: Adherence to ROS 2 best practices; compatibility with specified ROS 2 distributions; content clarity for beginners.
**Scale/Scope**: Single Docusaurus module, multiple small ROS 2 example packages.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The plan must adhere to the following:
- **Writing Tone & Style**: Friendly, simple, approachable, short paragraphs, clear explanations. (Compliant)
- **Structure & File Organization**: Content in `/content/chapters`, consistent chapter file structure (Title, Intro, Sections, Examples, Summary). (Compliant)
- **AI Usage & Verification**: No unverified AI text, all generated paragraphs reviewed, claims checked manually, human approval for final text. (To be enforced during content creation)
- **Consistency Rules**: Consistent terminology, friendly/simple style, correct formatting (H1-H3, bullet points, short paragraphs). (To be enforced during content creation)
- **Quality Requirements**: Original content, correct/tested examples, factual accuracy, matching tone/structure/clarity. (To be enforced during content creation and testing)
- **Editorial Guidelines**: Avoid jargon, use examples/analogies, focused sections, logical chapter flow. (To be enforced during content creation)

## Project Structure

### Documentation (this feature)

```text
specs/1-docusaurus-book-spec/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
content/
└── chapters/
    └── module1-ros2-nodes-graph.md  # Docusaurus markdown content

ros2_ws/
├── src/
│   ├── publisher_pkg/             # Example ROS 2 publisher package (Python/C++)
│   │   ├── src/                 # C++ source
│   │   ├── publisher_node.py    # Python node
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── subscriber_pkg/            # Example ROS 2 subscriber package (Python/C++)
│   │   ├── src/
│   │   ├── subscriber_node.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── add_two_ints_server/       # Example ROS 2 service server
│   │   ├── src/
│   │   ├── add_two_ints_server.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── add_two_ints_client/       # Example ROS 2 service client
│       ├── src/
│       ├── add_two_ints_client.py
│       ├── CMakeLists.txt
│       └── package.xml
└── install/
└── log/
└── build/
```

**Structure Decision**: The primary content will reside in `content/chapters` for Docusaurus, with ROS 2 example packages organized within a `ros2_ws/src` directory at the repository root. This allows for clear separation of documentation and runnable code examples.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

## Milestones

1.  **Week 1**:
    *   Complete `research.md` for ROS 2 best practices and Docusaurus integration.
    *   Initial draft of `content/chapters/module1-ros2-nodes-graph.md` (content outline).
2.  **Week 2**:
    *   Implement basic Python ROS 2 publisher and subscriber nodes.
    *   Integrate initial code examples into Docusaurus markdown.
3.  **Week 3**:
    *   Implement basic C++ ROS 2 publisher and subscriber nodes.
    *   Implement ROS 2 service client and server examples (Python/C++).
4.  **Week 4**:
    *   Develop CI workflow for linting, building, and testing ROS 2 packages.
    *   Refine Docusaurus content, add `rqt_graph` visualization instructions.
5.  **Week 5**:
    *   Implement ROS 2 action client and server examples.
    *   Finalize content and code examples for module review.

## Work Breakdown (WBS)

| Task ID | Owner | Task Description | ETA (weeks) | Effort (story points) |
|---------|-------|------------------|-------------|-----------------------|
| PL-001 | LLM | Research ROS 2 best practices for nodes, topics, services, actions. | 0.5 | 3 |
| PL-002 | LLM | Research Docusaurus integration for code examples and callouts. | 0.5 | 3 |
| PL-003 | LLM | Draft `content/chapters/module1-ros2-nodes-graph.md` outline. | 0.5 | 2 |
| DEV-001 | Dev | Implement Python ROS 2 publisher node (`ros2_ws/src/publisher_pkg/publisher_node.py`). | 1 | 5 |
| DEV-002 | Dev | Implement Python ROS 2 subscriber node (`ros2_ws/src/subscriber_pkg/subscriber_node.py`). | 1 | 5 |
| DEV-003 | Dev | Implement C++ ROS 2 publisher node (`ros2_ws/src/publisher_pkg/src/publisher_node.cpp`). | 1 | 5 |
| DEV-004 | Dev | Implement C++ ROS 2 subscriber node (`ros2_ws/src/subscriber_pkg/src/subscriber_node.cpp`). | 1 | 5 |
| DEV-005 | Dev | Implement Python ROS 2 service server (`ros2_ws/src/add_two_ints_server/add_two_ints_server.py`). | 1 | 5 |
| DEV-006 | Dev | Implement Python ROS 2 service client (`ros2_ws/src/add_two_ints_client/add_two_ints_client.py`). | 1 | 5 |
| DEV-007 | Dev | Implement C++ ROS 2 service server (optional, similar to DEV-005). | 1 | 5 |
| DEV-008 | Dev | Implement C++ ROS 2 service client (optional, similar to DEV-006). | 1 | 5 |
| DOC-001 | LLM | Integrate Python publisher/subscriber code examples into Docusaurus markdown. | 0.5 | 3 |
| DOC-002 | LLM | Integrate C++ publisher/subscriber code examples into Docusaurus markdown. | 0.5 | 3 |
| DOC-003 | LLM | Integrate service client/server code examples into Docusaurus markdown. | 0.5 | 3 |
| CI-001 | Dev | Develop GitHub Actions workflow for ROS 2 linting (`ament_cpplint`, `black`, `flake8`). | 1 | 8 |
| CI-002 | Dev | Develop GitHub Actions workflow for ROS 2 package compilation (`colcon build`). | 1 | 8 |
| CI-003 | Dev | Develop GitHub Actions workflow for ROS 2 unit tests (`gtest`, `pytest`). | 1 | 8 |
| CI-004 | Dev | Develop GitHub Actions workflow for Docusaurus preview build. | 1 | 5 |
| VAL-001 | Dev | Create and verify `rqt_graph` visualization instructions. | 0.5 | 3 |
| DOC-004 | LLM | Finalize `content/chapters/module1-ros2-nodes-graph.md` content and formatting. | 1 | 8 |
| DEV-009 | Dev | Implement ROS 2 action client and server examples (Python/C++). | 1.5 | 8 |
| DOC-005 | LLM | Integrate action client/server code examples into Docusaurus markdown. | 0.5 | 3 |

## Page Generation Steps

To create the Docusaurus markdown file for "Module 1: ROS 2 Nodes & Graph":

```bash
mkdir -p content/chapters
touch content/chapters/module1-ros2-nodes-graph.md
# Then, manually edit content/chapters/module1-ros2-nodes-graph.md following the constitution's module format.
```

## Example CI workflow (GitHub Actions)

File: `.github/workflows/ros2-docusaurus-ci.yml`

```yaml
name: ROS 2 Docusaurus CI

on:
  push:
    branches:
      - main
      - '*-spec'
  pull_request:
    branches:
      - main
      - '*-spec'

jobs:
  lint-and-test-ros2:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.6
        with:
          ros-distro: humble # or iron
          package-dependencies: humble
      - name: Install Python dependencies
        run: |
          sudo apt update
          sudo apt install -y python3-pip
          pip install flake8 black
      - name: Build ROS 2 workspace
        run: |
          cd ros2_ws
          rosdep install --from-paths src --ignore-src --rosdistro ${{ env.ROS_DISTRO }} -y
          colcon build --packages-skip subscriber_pkg # Example, adjust as needed
      - name: Run ROS 2 Linting (C++)
        run: |
          cd ros2_ws
          colcon test --packages-select publisher_pkg # Replace with specific packages, e.g., ament_cpplint
      - name: Run ROS 2 Linting (Python)
        run: |
          cd ros2_ws/src
          flake8 .
          black --check .
      - name: Run ROS 2 Unit Tests
        run: |
          cd ros2_ws
          colcon test
          colcon test --event-handlers console_direct+ --verbose
          # Example for specific package:
          # colcon test --packages-select publisher_pkg
          # colcon test --packages-select subscriber_pkg

  build-docusaurus-preview:
    needs: lint-and-test-ros2
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Use Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '18'
      - name: Install dependencies
        run: npm install
      - name: Build Docusaurus
        run: npm run build
      - name: Deploy to GitHub Pages (optional, for preview)
        # Add steps for deploying built Docusaurus site to a preview environment if needed.
        # e.g., using peaceiris/actions-gh-pages@v3
```

## Asset Pipeline

Images for Docusaurus documentation (e.g., `rqt_graph` visualizations, architectural diagrams) will be stored in the `/static/img/` directory within the Docusaurus project.
*   **Naming Convention**: `kebab-case` (e.g., `ros2-graph-example.png`).
*   **Referencing**: Images will be referenced in markdown using relative paths, e.g., `![alt text](/img/ros2-graph-example.png)`.
*   **USD Assets/Sensor Data**: For advanced modules, USD assets (from Isaac Sim) and raw sensor data will be stored in a dedicated `assets/` directory at the repository root, with clear versioning and documentation.

## Sim / Hardware Validation Plan

1.  **Gazebo Headless Simulation**:
    *   Launch ROS 2 nodes and a basic Gazebo simulation (e.g., a simple robot in an empty world) in a headless environment.
    *   Verify communication (topics, services, actions) between ROS 2 nodes and the simulator using `ros2 topic echo`, `ros2 service call`, and `ros2 action send` commands.
    *   **Runnable Check**: `gazebo --headless -s libgazebo_ros_factory.so world.sdf && ros2 launch <robot_pkg> <robot_launch_file>`
2.  **Isaac Sim Headless Simulation**:
    *   Utilize Isaac Sim's headless mode to run more complex robotic scenarios.
    *   Validate the interaction of ROS 2 nodes with the simulated robot and environment, checking for correct sensor data processing and command execution.
    *   **Runnable Check**: `isaac sim --headless --exts ros2_bridge --stage /path/to/robot.usd python script_to_run_sim.py`
3.  **NVIDIA Jetson Deployment (Hardware)**:
    *   Cross-compile ROS 2 packages for the ARM architecture of the Jetson.
    *   Deploy and run ROS 2 nodes directly on a Jetson Nano/Xavier.
    *   Validate real-world performance and functionality, ensuring sensor data is correctly acquired and processed, and actuators respond as expected.
    *   **Runnable Check**: `ssh user@jetson_ip 'source /opt/ros/humble/setup.bash && colcon build && ros2 launch <pkg> <launch_file>'`

## Rollout Strategy

1.  **Draft**: Initial content and code examples are developed and integrated into Docusaurus.
2.  **Internal Review**: The module is reviewed by internal experts for technical accuracy, clarity, and adherence to constitution guidelines.
3.  **Alpha Testing**: A small group of target students or instructors test the module for usability, comprehension, and reproducibility.
4.  **Feedback Integration**: Feedback from alpha testing is incorporated to refine the content and examples.
5.  **Publishing**: The final, reviewed module is published to the Docusaurus website.

## Risk Register

1.  **Risk**: Incompatibility issues with different ROS 2 versions or operating systems.
    *   **Mitigation**: Standardize on ROS 2 Humble/Iron and Ubuntu 22.04. Provide clear setup instructions, Docker images, and CI checks for compatibility.
2.  **Risk**: Code examples become outdated or break due to ROS 2 API changes.
    *   **Mitigation**: Implement robust CI with regular builds and tests. Monitor ROS 2 release notes for breaking changes and proactively update examples.
3.  **Risk**: Difficulty for beginners to set up development environment.
    *   **Mitigation**: Provide detailed, easy-to-follow setup guides, including Dockerfiles for consistent environments. Offer troubleshooting tips and FAQs.
4.  **Risk**: Code examples contain bugs or do not fully illustrate concepts.
    *   **Mitigation**: Thorough internal review, extensive unit and smoke testing in CI, and peer review of code by experienced ROS developers.
5.  **Risk**: Docusaurus site build failures due to markdown errors or broken links.
    *   **Mitigation**: Implement markdown linting in CI. Regularly check for broken links and validate Docusaurus builds in a dedicated CI job.

## Acceptance checklist

- [x] Plan header is complete with title, spec_ref, author, and date.
- [x] Summary provides a 3-5 sentence overview.
- [x] Milestones are listed with relative weeks.
- [x] Work Breakdown (WBS) includes task IDs, owner, ETA, and story points.
- [x] Repo & File Structure clearly defines Docusaurus and ROS 2 workspace paths.
- [x] Page Generation Steps provide clear commands for Docusaurus markdown creation.
- [x] Example CI workflow (GitHub Actions) includes linting, ROS 2 tests, and Docusaurus build.
- [x] Asset Pipeline describes storage and referencing of images.
- [x] Sim / Hardware Validation Plan outlines steps for Gazebo, Isaac Sim, and Jetson.
- [x] Rollout Strategy details the process from draft to publication.
- [x] Risk Register lists top 5 technical risks and mitigations.
- [x] Acceptance checklist is present and tickable.

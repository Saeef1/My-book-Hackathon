[
  {
    "id": "sp.task.0001",
    "title": "Draft content outline for Module 4: Vision-Language-Action (VLA)",
    "purpose": "To create the initial structure and headings for the Docusaurus markdown file, adhering to the constitution's module format.",
    "inputs": ["F:/tester/my-book/.specify/memory/constitution.md", "specs/4-vision-language-action-vla/spec.md"],
    "steps": [
      "Create the `content/chapters/module4-vision-language-action.md` file.",
      "Add the YAML frontmatter as specified in the constitution.",
      "Outline the main sections: Title, Introduction, Sections (e.g., Voice-to-Action, Cognitive Planning with LLMs, Object Identification, Capstone Project), Examples, Summary, following the Module & Chapter Format from the constitution.",
      "Populate initial descriptive text for each section based on the spec's Goals & Learning Outcomes."
    ],
    "expected_output": "A markdown file `content/chapters/module4-vision-language-action.md` with the outline and initial content.",
    "path": "content/chapters/module4-vision-language-action.md",
    "token_chunk_hint": 1000,
    "dependencies": [],
    "acceptance_tests": [
      "Verify `content/chapters/module4-vision-language-action.md` exists and follows the specified Docusaurus module format.",
      "Check for correct YAML frontmatter and section headings."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0002",
    "title": "Implement voice-to-action system using OpenAI Whisper (Python script and ROS 2 node)",
    "purpose": "To enable the robot to process voice commands from natural language.",
    "inputs": ["specs/4-vision-language-action-vla/plan.md"],
    "steps": [
      "Create the directory `llm_integration/scripts/`.",
      "Develop a Python script `llm_integration/scripts/whisper_node.py` that uses OpenAI Whisper (API or local model) to convert audio input to text.",
      "Create a ROS 2 node that subscribes to an audio topic, processes it with Whisper, and publishes the transcribed text to a `std_msgs/msg/String` topic (e.g., `/speech_input`)."
    ],
    "expected_output": "A functional Python ROS 2 node that transcribes speech to text.",
    "path": "llm_integration/scripts/whisper_node.py",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Launch the Whisper ROS 2 node.",
      "Publish audio data to the input topic.",
      "Verify that transcribed text is published to `/speech_input`."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0003",
    "title": "Develop LLM integration for cognitive planning (Python script to translate NL to ROS 2 actions)",
    "purpose": "To enable the robot to understand and plan based on high-level natural language commands.",
    "inputs": ["specs/4-vision-language-action-vla/plan.md"],
    "steps": [
      "Develop a Python script `llm_integration/scripts/cognitive_planner.py` that:
        - Takes a natural language command as input.
        - Uses an LLM (e.g., OpenAI GPT, local LLM) via its API or a library to translate the command into a sequence of ROS 2 actions (e.g., `navigate_to_pose` action goals, `manipulate_object` service calls).
        - Defines a structured output format for ROS 2 actions.",
      "Create a ROS 2 node that subscribes to the transcribed speech topic (`/speech_input`), passes the text to the cognitive planner, and publishes the resulting sequence of ROS 2 actions to a designated topic or directly calls ROS 2 interfaces."
    ],
    "expected_output": "A Python ROS 2 node that performs cognitive planning using an LLM.",
    "path": "llm_integration/scripts/cognitive_planner.py",
    "token_chunk_hint": 1500,
    "dependencies": ["sp.task.0002"],
    "acceptance_tests": [
      "Publish a natural language command to `/speech_input`.",
      "Verify that the cognitive planner node outputs the correct sequence of ROS 2 actions."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0004",
    "title": "Implement basic computer vision for object identification (`ros2_ws/src/object_detection_pkg/`)",
    "purpose": "To enable the robot to identify and locate objects in its environment.",
    "inputs": ["specs/4-vision-language-action-vla/plan.md"],
    "steps": [
      "Create the ROS 2 package `object_detection_pkg` in `ros2_ws/src/`.",
      "Develop a Python ROS 2 node `object_detection_node.py` that:
        - Subscribes to an image topic (e.g., `/camera/image_raw`).
        - Uses a simple computer vision model (e.g., pre-trained OpenCV model, or a basic color/shape detector) to identify a target object.
        - Publishes the detection results to a `sensor_msgs/msg/Detection2DArray` topic (e.g., `/object_detection`)."
    ],
    "expected_output": "A functional Python ROS 2 package for basic object detection.",
    "path": "ros2_ws/src/object_detection_pkg/",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Launch the object detection node and publish an image with the target object.",
      "Verify that detection results are published to `/object_detection`."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0005",
    "title": "Create ROS 2 package `vla_bringup` for integrating Whisper, LLM planning, and ROS 2 actions",
    "purpose": "To provide a unified launch system for all VLA components and the capstone project.",
    "inputs": ["specs/4-vision-language-action-vla/plan.md", "llm_integration/", "ros2_ws/src/object_detection_pkg/"],
    "steps": [
      "Create the ROS 2 package `vla_bringup` in `ros2_ws/src/`.",
      "Create a launch file `ros2_ws/src/vla_bringup/launch/capstone.launch.py` that:
        - Launches the Whisper ROS 2 node.
        - Launches the cognitive planner ROS 2 node.
        - Launches the object detection ROS 2 node.
        - Launches other necessary robot components (e.g., Nav2, robot state publisher) in a simulated environment."
    ],
    "expected_output": "A functional ROS 2 `vla_bringup` package with a capstone launch file.",
    "path": "ros2_ws/src/vla_bringup/",
    "token_chunk_hint": 1500,
    "dependencies": ["sp.task.0002", "sp.task.0003", "sp.task.0004"],
    "acceptance_tests": [
      "`ros2 launch vla_bringup capstone.launch.py` to launch the full VLA system in simulation."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0006",
    "title": "Set up a simulated environment for the capstone project (reusing/extending Module 2/3 assets)",
    "purpose": "To create a suitable virtual space for the autonomous humanoid capstone project.",
    "inputs": ["specs/4-vision-language-action-vla/plan.md", "gazebo_sim/", "isaac_sim_projects/"],
    "steps": [
      "Adapt an existing Gazebo or Isaac Sim scene (from Module 2 or 3) or create a new one to represent a 'room' with obstacles and target objects.",
      "Ensure the environment is compatible with the robot model and navigation stack (Nav2)."
    ],
    "expected_output": "A simulated environment configured for the VLA capstone project.",
    "path": "gazebo_sim/worlds/capstone_room.world",
    "token_chunk_hint": 1000,
    "dependencies": ["sp.task.0005"],
    "acceptance_tests": [
      "Launch the simulated environment and verify robot loading and obstacle presence."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0007",
    "title": "Implement Capstone Project: The Autonomous Humanoid (integrating all VLA components)",
    "purpose": "To develop an end-to-end autonomous humanoid robot capable of responding to voice commands.",
    "inputs": ["ros2_ws/src/vla_bringup/", "llm_integration/", "ros2_ws/src/object_detection_pkg/", "gazebo_sim/", "isaac_sim_projects/"],
    "steps": [
      "Develop a high-level orchestration node within `vla_bringup` that:
        - Listens for transcribed voice commands.
        - Calls the cognitive planner to get a sequence of actions.
        - Executes ROS 2 navigation actions (Nav2).
        - Triggers object identification (object_detection_pkg).
        - Executes manipulation actions (e.g., publishing joint commands or calling a manipulation action server).",
      "Ensure seamless communication and state management between all VLA components."
    ],
    "expected_output": "A fully integrated autonomous humanoid robot system in simulation.",
    "path": "ros2_ws/src/vla_bringup/nodes/orchestration_node.py",
    "token_chunk_hint": 2000,
    "dependencies": ["sp.task.0005", "sp.task.0006"],
    "acceptance_tests": [
      "Launch the full VLA system in simulation.",
      "Provide a voice command (simulated or real).",
      "Observe the robot's planning, navigation, obstacle avoidance, object identification, and manipulation actions."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0008",
    "title": "Integrate all VLA code examples and setup instructions into Docusaurus markdown",
    "purpose": "To document the Vision-Language-Action aspects of the module.",
    "inputs": ["content/chapters/module4-vision-language-action.md", "llm_integration/", "ros2_ws/src/vla_bringup/", "ros2_ws/src/object_detection_pkg/"],
    "steps": [
      "Add detailed instructions for setting up Whisper and LLM integration.",
      "Embed code examples for voice command processing, LLM prompting, and ROS 2 action sequencing.",
      "Include `run-instruction` comments and explanations."
    ],
    "expected_output": "Updated `content/chapters/module4-vision-language-action.md` with integrated content.",
    "path": "content/chapters/module4-vision-language-action.md",
    "token_chunk_hint": 2000,
    "dependencies": ["sp.task.0001", "sp.task.0002", "sp.task.0003", "sp.task.0004", "sp.task.0005", "sp.task.0006", "sp.task.0007"],
    "acceptance_tests": [
      "Manual review of `module4-vision-language-action.md` for completeness and clarity."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0009",
    "title": "Develop GitHub Actions workflow for voice-to-text and LLM planning tests",
    "purpose": "To automate the testing of voice-to-text and LLM cognitive planning components in CI.",
    "inputs": ["specs/4-vision-language-action-vla/plan.md", ".github/workflows/ros2-docusaurus-ci.yml"],
    "steps": [
      "Update `.github/workflows/ros2-docusaurus-ci.yml` to add a new job `vla-component-tests`.",
      "The job should set up a Python environment, install necessary libraries (Whisper, LLM API clients), and run tests for voice-to-text accuracy and LLM planning logic."
    ],
    "expected_output": "Updated GitHub Actions workflow with VLA component testing job.",
    "path": ".github/workflows/ros2-docusaurus-ci.yml",
    "token_chunk_hint": 1500,
    "dependencies": [],
    "acceptance_tests": [
      "Push the workflow file to GitHub and verify that the VLA component tests run successfully."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0010",
    "title": "Develop GitHub Actions workflow for computer vision object detection tests",
    "purpose": "To automate the testing of computer vision object detection components in CI.",
    "inputs": ["specs/4-vision-language-action-vla/plan.md", ".github/workflows/ros2-docusaurus-ci.yml"],
    "steps": [
      "Update `.github/workflows/ros2-docusaurus-ci.yml` to extend `vla-component-tests` or add a new job for object detection.",
      "The job should include steps to run object detection tests against synthetic or real-world image datasets."
    ],
    "expected_output": "Updated GitHub Actions workflow with object detection testing job.",
    "path": ".github/workflows/ros2-docusaurus-ci.yml",
    "token_chunk_hint": 1000,
    "dependencies": ["sp.task.0009"],
    "acceptance_tests": [
      "Push the workflow file to GitHub and verify that the object detection tests run successfully."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0011",
    "title": "Finalize Module 4 content and formatting",
    "purpose": "To conduct a final review of the Docusaurus markdown content, ensuring adherence to constitution guidelines and overall quality.",
    "inputs": ["content/chapters/module4-vision-language-action.md", "specs/4-vision-language-action-vla/spec.md", ".specify/memory/constitution.md"],
    "steps": [
      "Review `content/chapters/module4-vision-language-action.md` against all constitution checks (tone, style, structure, AI usage, consistency, quality, editorial).",
      "Verify all code examples and instructions are correctly formatted and have `run-instruction` comments.",
      "Check for grammatical errors, typos, and logical flow issues.",
      "Ensure all learning outcomes from `spec.md` are adequately addressed."
    ],
    "expected_output": "Finalized `content/chapters/module4-vision-language-action.md` ready for internal review and alpha testing.",
    "path": "content/chapters/module4-vision-language-action.md",
    "token_chunk_hint": 1200,
    "dependencies": ["sp.task.0008"],
    "acceptance_tests": [
      "Manual review by an expert for overall quality and completeness.",
      "Docusaurus build passes without warnings or errors locally."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0012",
    "title": "Develop GitHub Actions workflow for Capstone Project end-to-end simulation test",
    "purpose": "To automate the end-to-end simulation test of the VLA capstone project in CI.",
    "inputs": ["specs/4-vision-language-action-vla/plan.md", ".github/workflows/ros2-docusaurus-ci.yml"],
    "steps": [
      "Update `.github/workflows/ros2-docusaurus-ci.yml` to add a new job `capstone-simulation-test`.",
      "The job should set up the ROS 2 and simulation environment, and run the end-to-end capstone simulation.",
      "Include steps to verify the robot's behavior in response to voice commands."
    ],
    "expected_output": "Updated GitHub Actions workflow with Capstone Project end-to-end simulation test job.",
    "path": ".github/workflows/ros2-docusaurus-ci.yml",
    "token_chunk_hint": 1500,
    "dependencies": ["sp.task.0009", "sp.task.0010"],
    "acceptance_tests": [
      "Push the workflow file to GitHub and verify that the Capstone simulation test runs successfully."
    ],
    "owner": "Developer"
  }
]

# Tasks for Module 4: Vision-Language-Action (VLA)

## Phase 1: Outline and Voice-to-Text

- [ ] sp.task.0001 Draft content outline for Module 4: Vision-Language-Action (VLA) (`content/chapters/module4-vision-language-action.md`)
- [ ] sp.task.0002 Implement voice-to-action system using OpenAI Whisper (Python script and ROS 2 node) (`llm_integration/scripts/whisper_node.py`)

## Phase 2: Cognitive Planning and Object Identification

- [ ] sp.task.0003 Develop LLM integration for cognitive planning (Python script to translate NL to ROS 2 actions) (`llm_integration/scripts/cognitive_planner.py`) (Dependencies: sp.task.0002)
- [ ] sp.task.0004 Implement basic computer vision for object identification (`ros2_ws/src/object_detection_pkg/`)

## Phase 3: Capstone Integration and Simulation

- [ ] sp.task.0005 Create ROS 2 package `vla_bringup` for integrating Whisper, LLM planning, and ROS 2 actions (`ros2_ws/src/vla_bringup/`) (Dependencies: sp.task.0002, sp.task.0003, sp.task.0004)
- [ ] sp.task.0006 Set up a simulated environment for the capstone project (reusing/extending Module 2/3 assets) (`gazebo_sim/worlds/capstone_room.world`) (Dependencies: sp.task.0005)
- [ ] sp.task.0007 Implement Capstone Project: The Autonomous Humanoid (integrating all VLA components) (`ros2_ws/src/vla_bringup/nodes/orchestration_node.py`) (Dependencies: sp.task.0005, sp.task.0006)

## Phase 4: Documentation and CI/CD

- [ ] sp.task.0008 Integrate all VLA code examples and setup instructions into Docusaurus markdown (`content/chapters/module4-vision-language-action.md`) (Dependencies: sp.task.0001, sp.task.0002, sp.task.0003, sp.task.0004, sp.task.0005, sp.task.0006, sp.task.0007)
- [ ] sp.task.0009 Develop GitHub Actions workflow for voice-to-text and LLM planning tests (`.github/workflows/ros2-docusaurus-ci.yml`)
- [ ] sp.task.0010 Develop GitHub Actions workflow for computer vision object detection tests (`.github/workflows/ros2-docusaurus-ci.yml`) (Dependencies: sp.task.0009)
- [ ] sp.task.0011 Finalize Module 4 content and formatting (`content/chapters/module4-vision-language-action.md`) (Dependencies: sp.task.0008)
- [ ] sp.task.0012 Develop GitHub Actions workflow for Capstone Project end-to-end simulation test (`.github/workflows/ros2-docusaurus-ci.yml`) (Dependencies: sp.task.0009, sp.task.0010)

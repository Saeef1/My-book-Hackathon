[
  {
    "id": "sp.task.0001",
    "title": "Research ROS 2 best practices for nodes, topics, services, actions",
    "purpose": "To gather foundational knowledge on ROS 2 communication paradigms and best practices for developing robust robotic applications.",
    "inputs": ["MCP context7"],
    "steps": [
      "Utilize MCP context7 to search for documentation and tutorials on ROS 2 nodes, topics, services, and actions.",
      "Identify common design patterns and recommended practices for each communication type.",
      "Summarize key findings regarding efficient and reliable ROS 2 communication."
    ],
    "expected_output": "A `research.md` file summarizing best practices and design patterns for ROS 2 communication.",
    "path": "specs/1-docusaurus-book-spec/research.md",
    "token_chunk_hint": 800,
    "dependencies": [],
    "acceptance_tests": [
      "Manual review of `research.md` for completeness and accuracy."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0002",
    "title": "Research Docusaurus integration for code examples and callouts",
    "purpose": "To understand how to effectively embed and format code examples, and use Docusaurus callouts for enhanced readability in the textbook.",
    "inputs": ["MCP context7"],
    "steps": [
      "Utilize MCP context7 to search for Docusaurus documentation on embedding code blocks, syntax highlighting, and adding 'run-instruction' comments.",
      "Investigate best practices for using Docusaurus callouts (notes, warnings, tips) to highlight important information.",
      "Document findings on Docusaurus markdown features relevant to the textbook's requirements."
    ],
    "expected_output": "Updated `research.md` or a new section in it, detailing Docusaurus integration techniques.",
    "path": "specs/1-docusaurus-book-spec/research.md",
    "token_chunk_hint": 800,
    "dependencies": [],
    "acceptance_tests": [
      "Manual review of `research.md` for completeness and accuracy regarding Docusaurus features."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0003",
    "title": "Draft content outline for Module 1: ROS 2 Nodes & Graph",
    "purpose": "To create the initial structure and headings for the Docusaurus markdown file, adhering to the constitution's module format.",
    "inputs": ["F:/tester/my-book/.specify/memory/constitution.md", "specs/1-docusaurus-book-spec/spec.md"],
    "steps": [
      "Create the `content/chapters/module1-ros2-nodes-graph.md` file.",
      "Add the YAML frontmatter as specified in the constitution.",
      "Outline the main sections: Title, Introduction, Sections (e.g., ROS 2 Nodes, ROS 2 Topics, ROS 2 Services, ROS 2 Actions), Examples, Summary, following the Module & Chapter Format from the constitution.",
      "Populate initial descriptive text for each section based on the spec's Goals & Learning Outcomes."
    ],
    "expected_output": "A markdown file `content/chapters/module1-ros2-nodes-graph.md` with the outline and initial content.",
    "path": "content/chapters/module1-ros2-nodes-graph.md",
    "token_chunk_hint": 1000,
    "dependencies": ["sp.task.0001", "sp.task.0002"],
    "acceptance_tests": [
      "Verify `content/chapters/module1-ros2-nodes-graph.md` exists and follows the specified Docusaurus module format.",
      "Check for correct YAML frontmatter and section headings."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0004",
    "title": "Implement Python ROS 2 publisher node",
    "purpose": "To create a basic ROS 2 publisher node in Python that sends string messages on a topic, demonstrating asynchronous communication.",
    "inputs": ["specs/1-docusaurus-book-spec/spec.md", "specs/1-docusaurus-book-spec/plan.md"],
    "steps": [
      "Create the ROS 2 package `publisher_pkg` in `ros2_ws/src/`.",
      "Implement `publisher_node.py` within `ros2_ws/src/publisher_pkg/` to publish `std_msgs/msg/String` messages on the `/chatter` topic.",
      "Add a `setup.py` and `package.xml` for the Python package.",
      "Include a `run-instruction` comment in the Python file on how to execute the node."
    ],
    "expected_output": "A functional Python ROS 2 publisher package at `ros2_ws/src/publisher_pkg` containing `publisher_node.py`, `setup.py`, and `package.xml`.",
    "path": "ros2_ws/src/publisher_pkg/publisher_node.py",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Build ROS 2 workspace: `colcon build --packages-select publisher_pkg`",
      "Run node: `ros2 run publisher_pkg publisher_node`",
      "Verify topic: `ros2 topic info /chatter`",
      "Verify messages: `ros2 topic echo /chatter`"
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0005",
    "title": "Implement Python ROS 2 subscriber node",
    "purpose": "To create a basic ROS 2 subscriber node in Python that receives string messages from a topic, demonstrating asynchronous communication.",
    "inputs": ["specs/1-docusaurus-book-spec/spec.md", "specs/1-docusaurus-book-spec/plan.md"],
    "steps": [
      "Create the ROS 2 package `subscriber_pkg` in `ros2_ws/src/`.",
      "Implement `subscriber_node.py` within `ros2_ws/src/subscriber_pkg/` to subscribe to `std_msgs/msg/String` messages on the `/chatter` topic and print them.",
      "Add a `setup.py` and `package.xml` for the Python package.",
      "Include a `run-instruction` comment in the Python file on how to execute the node."
    ],
    "expected_output": "A functional Python ROS 2 subscriber package at `ros2_ws/src/subscriber_pkg` containing `subscriber_node.py`, `setup.py`, and `package.xml`.",
    "path": "ros2_ws/src/subscriber_pkg/subscriber_node.py",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Build ROS 2 workspace: `colcon build --packages-select subscriber_pkg`",
      "Run node: `ros2 run subscriber_pkg subscriber_node`",
      "Verify messages from publisher: `ros2 run publisher_pkg publisher_node` (should see messages in subscriber output)"
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0006",
    "title": "Implement C++ ROS 2 publisher node",
    "purpose": "To create a basic ROS 2 publisher node in C++ that sends string messages on a topic.",
    "inputs": ["specs/1-docusaurus-book-spec/spec.md", "specs/1-docusaurus-book-spec/plan.md"],
    "steps": [
      "Create the ROS 2 package `publisher_pkg` in `ros2_ws/src/` (if not already existing from Python task).",
      "Implement `publisher_node.cpp` within `ros2_ws/src/publisher_pkg/src/` to publish `std_msgs/msg/String` messages on the `/chatter` topic.",
      "Update `CMakeLists.txt` and `package.xml` for the C++ executable.",
      "Include a `run-instruction` comment in the C++ file on how to execute the node."
    ],
    "expected_output": "A functional C++ ROS 2 publisher executable within `ros2_ws/src/publisher_pkg`.",
    "path": "ros2_ws/src/publisher_pkg/src/publisher_node.cpp",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Build ROS 2 workspace: `colcon build --packages-select publisher_pkg`",
      "Run node: `ros2 run publisher_pkg publisher_node_cpp` (assuming executable name)",
      "Verify topic: `ros2 topic info /chatter`",
      "Verify messages: `ros2 topic echo /chatter`"
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0007",
    "title": "Implement C++ ROS 2 subscriber node",
    "purpose": "To create a basic ROS 2 subscriber node in C++ that receives string messages from a topic.",
    "inputs": ["specs/1-docusaurus-book-spec/spec.md", "specs/1-docusaurus-book-spec/plan.md"],
    "steps": [
      "Create the ROS 2 package `subscriber_pkg` in `ros2_ws/src/` (if not already existing from Python task).",
      "Implement `subscriber_node.cpp` within `ros2_ws/src/subscriber_pkg/src/` to subscribe to `std_msgs/msg/String` messages on the `/chatter` topic and print them.",
      "Update `CMakeLists.txt` and `package.xml` for the C++ executable.",
      "Include a `run-instruction` comment in the C++ file on how to execute the node."
    ],
    "expected_output": "A functional C++ ROS 2 subscriber executable within `ros2_ws/src/subscriber_pkg`.",
    "path": "ros2_ws/src/subscriber_pkg/src/subscriber_node.cpp",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Build ROS 2 workspace: `colcon build --packages-select subscriber_pkg`",
      "Run node: `ros2 run subscriber_pkg subscriber_node_cpp` (assuming executable name)",
      "Verify messages from publisher: `ros2 run publisher_pkg publisher_node_cpp` (should see messages in subscriber output)"
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0008",
    "title": "Implement Python ROS 2 service server",
    "purpose": "To create a basic ROS 2 service server in Python that adds two integers.",
    "inputs": ["specs/1-docusaurus-book-spec/spec.md", "specs/1-docusaurus-book-spec/plan.md"],
    "steps": [
      "Create the ROS 2 package `add_two_ints_server` in `ros2_ws/src/`.",
      "Implement `add_two_ints_server.py` within `ros2_ws/src/add_two_ints_server/` to provide the `example_interfaces/srv/AddTwoInts` service.",
      "Add a `setup.py` and `package.xml` for the Python package.",
      "Include a `run-instruction` comment in the Python file on how to execute the node."
    ],
    "expected_output": "A functional Python ROS 2 service server package at `ros2_ws/src/add_two_ints_server`.",
    "path": "ros2_ws/src/add_two_ints_server/add_two_ints_server.py",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Build ROS 2 workspace: `colcon build --packages-select add_two_ints_server`",
      "Run server: `ros2 run add_two_ints_server add_two_ints_server`",
      "Verify service: `ros2 service type /add_two_ints`"
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0009",
    "title": "Implement Python ROS 2 service client",
    "purpose": "To create a basic ROS 2 service client in Python that calls the add_two_ints service.",
    "inputs": ["specs/1-docusaurus-book-spec/spec.md", "specs/1-docusaurus-book-spec/plan.md"],
    "steps": [
      "Create the ROS 2 package `add_two_ints_client` in `ros2_ws/src/`.",
      "Implement `add_two_ints_client.py` within `ros2_ws/src/add_two_ints_client/` to call the `/add_two_ints` service.",
      "Add a `setup.py` and `package.xml` for the Python package.",
      "Include a `run-instruction` comment in the Python file on how to execute the node."
    ],
    "expected_output": "A functional Python ROS 2 service client package at `ros2_ws/src/add_two_ints_client`.",
    "path": "ros2_ws/src/add_two_ints_client/add_two_ints_client.py",
    "token_chunk_hint": 1200,
    "dependencies": ["sp.task.0008"],
    "acceptance_tests": [
      "Build ROS 2 workspace: `colcon build --packages-select add_two_ints_client`",
      "Run server (if not already running): `ros2 run add_two_ints_server add_two_ints_server`",
      "Run client: `ros2 run add_two_ints_client add_two_ints_client`",
      "Verify result: Client should print the sum of two integers."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0010",
    "title": "Integrate Python publisher/subscriber code examples into Docusaurus",
    "purpose": "To embed the Python publisher and subscriber code examples into the Docusaurus markdown, ensuring correct formatting and `run-instruction` comments.",
    "inputs": ["content/chapters/module1-ros2-nodes-graph.md", "ros2_ws/src/publisher_pkg/publisher_node.py", "ros2_ws/src/subscriber_pkg/subscriber_node.py"],
    "steps": [
      "Open `content/chapters/module1-ros2-nodes-graph.md`.",
      "Locate the relevant sections for ROS 2 topics.",
      "Embed the content of `publisher_node.py` and `subscriber_node.py` as code blocks.",
      "Add `run-instruction` comments above each code block, explaining how to execute them from the CLI."
    ],
    "expected_output": "Updated `content/chapters/module1-ros2-nodes-graph.md` with integrated Python code examples.",
    "path": "content/chapters/module1-ros2-nodes-graph.md",
    "token_chunk_hint": 1000,
    "dependencies": ["sp.task.0003", "sp.task.0004", "sp.task.0005"],
    "acceptance_tests": [
      "Manual review of `module1-ros2-nodes-graph.md` for correctly formatted and executable Python code blocks."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0011",
    "title": "Integrate Python service client/server code examples into Docusaurus",
    "purpose": "To embed the Python service client and server code examples into the Docusaurus markdown, ensuring correct formatting and `run-instruction` comments.",
    "inputs": ["content/chapters/module1-ros2-nodes-graph.md", "ros2_ws/src/add_two_ints_server/add_two_ints_server.py", "ros2_ws/src/add_two_ints_client/add_two_ints_client.py"],
    "steps": [
      "Open `content/chapters/module1-ros2-nodes-graph.md`.",
      "Locate the relevant sections for ROS 2 services.",
      "Embed the content of `add_two_ints_server.py` and `add_two_ints_client.py` as code blocks.",
      "Add `run-instruction` comments above each code block, explaining how to execute them from the CLI."
    ],
    "expected_output": "Updated `content/chapters/module1-ros2-nodes-graph.md` with integrated Python service code examples.",
    "path": "content/chapters/module1-ros2-nodes-graph.md",
    "token_chunk_hint": 1000,
    "dependencies": ["sp.task.0003", "sp.task.0008", "sp.task.0009"],
    "acceptance_tests": [
      "Manual review of `module1-ros2-nodes-graph.md` for correctly formatted and executable Python service code blocks."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0012",
    "title": "Develop GitHub Actions workflow for ROS 2 linting",
    "purpose": "To create a CI/CD workflow that lints ROS 2 C++ and Python packages using `ament_cpplint`, `black`, and `flake8`.",
    "inputs": ["specs/1-docusaurus-book-spec/plan.md"],
    "steps": [
      "Create the workflow file `.github/workflows/ros2-docusaurus-ci.yml`.",
      "Define a job `lint-and-test-ros2`.",
      "Add steps to checkout code, setup ROS 2 (humble/iron), and install Python linting tools (`flake8`, `black`).",
      "Include steps for C++ linting (e.g., using `colcon test --packages-select <pkg> --ament-cpplint`) and Python linting (`flake8 .`, `black --check .`)."
    ],
    "expected_output": "A GitHub Actions workflow file `.github/workflows/ros2-docusaurus-ci.yml` with ROS 2 linting jobs.",
    "path": ".github/workflows/ros2-docusaurus-ci.yml",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Push the workflow file to GitHub and verify that the linting job runs successfully."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0013",
    "title": "Develop GitHub Actions workflow for ROS 2 package compilation",
    "purpose": "To create a CI/CD workflow that builds all ROS 2 packages in the workspace using `colcon build`.",
    "inputs": ["specs/1-docusaurus-book-spec/plan.md", ".github/workflows/ros2-docusaurus-ci.yml"],
    "steps": [
      "Update the existing workflow file `.github/workflows/ros2-docusaurus-ci.yml`.",
      "Within the `lint-and-test-ros2` job, add a step to install ROS dependencies (`rosdep install`).",
      "Add a step to build the entire ROS 2 workspace: `colcon build`."
    ],
    "expected_output": "Updated GitHub Actions workflow file `.github/workflows/ros2-docusaurus-ci.yml` with a ROS 2 package compilation step.",
    "path": ".github/workflows/ros2-docusaurus-ci.yml",
    "token_chunk_hint": 1000,
    "dependencies": ["sp.task.0012"],
    "acceptance_tests": [
      "Push the workflow file to GitHub and verify that the ROS 2 package compilation job runs successfully."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0014",
    "title": "Develop GitHub Actions workflow for ROS 2 unit tests",
    "purpose": "To create a CI/CD workflow that runs unit tests for ROS 2 C++ and Python packages using `gtest` and `pytest`.",
    "inputs": ["specs/1-docusaurus-book-spec/plan.md", ".github/workflows/ros2-docusaurus-ci.yml"],
    "steps": [
      "Update the existing workflow file `.github/workflows/ros2-docusaurus-ci.yml`.",
      "Within the `lint-and-test-ros2` job, add a step to run `colcon test` for the entire workspace.",
      "Include examples of how to run tests for specific packages or languages (e.g., `colcon test --packages-select publisher_pkg`)."
    ],
    "expected_output": "Updated GitHub Actions workflow file `.github/workflows/ros2-docusaurus-ci.yml` with ROS 2 unit testing jobs.",
    "path": ".github/workflows/ros2-docusaurus-ci.yml",
    "token_chunk_hint": 1000,
    "dependencies": ["sp.task.0013"],
    "acceptance_tests": [
      "Push the workflow file to GitHub and verify that the ROS 2 unit tests run successfully."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0015",
    "title": "Develop GitHub Actions workflow for Docusaurus preview build",
    "purpose": "To create a CI/CD workflow that builds the Docusaurus site and could optionally deploy a preview.",
    "inputs": ["specs/1-docusaurus-book-spec/plan.md", ".github/workflows/ros2-docusaurus-ci.yml"],
    "steps": [
      "Update the existing workflow file `.github/workflows/ros2-docusaurus-ci.yml`.",
      "Define a new job `build-docusaurus-preview` that depends on `lint-and-test-ros2`.",
      "Add steps to checkout code, setup Node.js, install Docusaurus dependencies (`npm install`), and build the Docusaurus site (`npm run build`).",
      "Include a placeholder for deploying the preview to GitHub Pages or another environment."
    ],
    "expected_output": "Updated GitHub Actions workflow file `.github/workflows/ros2-docusaurus-ci.yml` with a Docusaurus build job.",
    "path": ".github/workflows/ros2-docusaurus-ci.yml",
    "token_chunk_hint": 1200,
    "dependencies": ["sp.task.0014"],
    "acceptance_tests": [
      "Push the workflow file to GitHub and verify that the Docusaurus build job runs successfully."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0016",
    "title": "Create and verify rqt_graph visualization instructions",
    "purpose": "To provide clear instructions on how to use `rqt_graph` to visualize the ROS 2 computational graph.",
    "inputs": ["content/chapters/module1-ros2-nodes-graph.md"],
    "steps": [
      "Open `content/chapters/module1-ros2-nodes-graph.md`.",
      "Locate the relevant section for visualizing the computational graph.",
      "Add detailed instructions on how to install and run `rqt_graph`.",
      "Include example CLI commands for launching `rqt_graph` and interpreting its output."
    ],
    "expected_output": "Updated `content/chapters/module1-ros2-nodes-graph.md` with `rqt_graph` visualization instructions.",
    "path": "content/chapters/module1-ros2-nodes-graph.md",
    "token_chunk_hint": 800,
    "dependencies": ["sp.task.0003"],
    "acceptance_tests": [
      "Manual review of `module1-ros2-nodes-graph.md` for clear and accurate `rqt_graph` instructions."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0017",
    "title": "Finalize Module 1 content and formatting",
    "purpose": "To conduct a final review of the Docusaurus markdown content, ensuring adherence to constitution guidelines and overall quality.",
    "inputs": ["content/chapters/module1-ros2-nodes-graph.md", "specs/1-docusaurus-book-spec/spec.md", ".specify/memory/constitution.md"],
    "steps": [
      "Review `content/chapters/module1-ros2-nodes-graph.md` against all constitution checks (tone, style, structure, AI usage, consistency, quality, editorial).",
      "Verify all code examples are correctly formatted and have `run-instruction` comments.",
      "Check for grammatical errors, typos, and logical flow issues.",
      "Ensure all learning outcomes from `spec.md` are adequately addressed."
    ],
    "expected_output": "Finalized `content/chapters/module1-ros2-nodes-graph.md` ready for internal review and alpha testing.",
    "path": "content/chapters/module1-ros2-nodes-graph.md",
    "token_chunk_hint": 1200,
    "dependencies": ["sp.task.0010", "sp.task.0011", "sp.task.0016", "sp.task.0019"],
    "acceptance_tests": [
      "Manual review by an expert for overall quality and completeness.",
      "Docusaurus build passes without warnings or errors locally."
    ],
    "owner": "LLM"
  },
  {
    "id": "sp.task.0018",
    "title": "Implement ROS 2 action client and server examples (Python/C++)",
    "purpose": "To create functional ROS 2 action client and server examples in Python and C++, demonstrating long-running goal-oriented tasks.",
    "inputs": ["specs/1-docusaurus-book-spec/spec.md", "specs/1-docusaurus-book-spec/plan.md"],
    "steps": [
      "Create necessary ROS 2 packages for action client and server (e.g., `navigate_action_server_pkg`, `navigate_action_client_pkg`).",
      "Implement Python action server (`navigate_action_server.py`) for `nav2_msgs/action/NavigateToPose`.",
      "Implement Python action client (`navigate_action_client.py`) to send goals and process feedback.",
      "Implement C++ action server and client (similar to Python).",
      "Include `run-instruction` comments in all action files."
    ],
    "expected_output": "Functional Python and C++ ROS 2 action client and server packages.",
    "path": "ros2_ws/src/",
    "token_chunk_hint": 1200,
    "dependencies": [],
    "acceptance_tests": [
      "Build ROS 2 workspace.",
      "Run action server and client, verify goal execution and feedback."
    ],
    "owner": "Developer"
  },
  {
    "id": "sp.task.0019",
    "title": "Integrate action client/server code examples into Docusaurus",
    "purpose": "To embed the ROS 2 action client and server code examples into the Docusaurus markdown, ensuring correct formatting and `run-instruction` comments.",
    "inputs": ["content/chapters/module1-ros2-nodes-graph.md", "ros2_ws/src/navigate_action_server_pkg/navigate_action_server.py", "ros2_ws/src/navigate_action_client_pkg/navigate_action_client.py"],
    "steps": [
      "Open `content/chapters/module1-ros2-nodes-graph.md`.",
      "Locate the relevant sections for ROS 2 actions.",
      "Embed the content of action server and client files as code blocks.",
      "Add `run-instruction` comments above each code block."
    ],
    "expected_output": "Updated `content/chapters/module1-ros2-nodes-graph.md` with integrated action code examples.",
    "path": "content/chapters/module1-ros2-nodes-graph.md",
    "token_chunk_hint": 1000,
    "dependencies": ["sp.task.0003", "sp.task.0018"],
    "acceptance_tests": [
      "Manual review of `module1-ros2-nodes-graph.md` for correctly formatted and executable action code blocks."
    ],
    "owner": "LLM"
  }
]

# Tasks for Module 1: ROS 2 Nodes & Graph

## Phase 1: Research and Outline

- [ ] sp.task.0001 Research ROS 2 best practices for nodes, topics, services, actions (`specs/1-docusaurus-book-spec/research.md`)
- [ ] sp.task.0002 Research Docusaurus integration for code examples and callouts (`specs/1-docusaurus-book-spec/research.md`)
- [ ] sp.task.0003 Draft content outline for Module 1: ROS 2 Nodes & Graph (`content/chapters/module1-ros2-nodes-graph.md`) (Dependencies: sp.task.0001, sp.task.0002)

## Phase 2: Core ROS 2 Communication - Python

### User Story: Implement basic ROS 2 publisher-subscriber pair (Python)

- [ ] sp.task.0004 [P] [US1] Implement Python ROS 2 publisher node (`ros2_ws/src/publisher_pkg/publisher_node.py`)
    *   **run-instruction**: `cd ros2_ws && colcon build --packages-select publisher_pkg && ros2 run publisher_pkg publisher_node`
- [ ] sp.task.0005 [P] [US1] Implement Python ROS 2 subscriber node (`ros2_ws/src/subscriber_pkg/subscriber_node.py`)
    *   **run-instruction**: `cd ros2_ws && colcon build --packages-select subscriber_pkg && ros2 run subscriber_pkg subscriber_node`
- [ ] sp.task.0008 [P] [US1] Implement Python ROS 2 service server (`ros2_ws/src/add_two_ints_server/add_two_ints_server.py`)
    *   **run-instruction**: `cd ros2_ws && colcon build --packages-select add_two_ints_server && ros2 run add_two_ints_server add_two_ints_server`
- [ ] sp.task.0009 [US1] Implement Python ROS 2 service client (`ros2_ws/src/add_two_ints_client/add_two_ints_client.py`) (Dependencies: sp.task.0008)
    *   **run-instruction**: `cd ros2_ws && colcon build --packages-select add_two_ints_client && ros2 run add_two_ints_client add_two_ints_client`

## Phase 3: Core ROS 2 Communication - C++

### User Story: Implement basic ROS 2 publisher-subscriber pair (C++)

- [ ] sp.task.0006 [P] [US2] Implement C++ ROS 2 publisher node (`ros2_ws/src/publisher_pkg/src/publisher_node.cpp`)
    *   **run-instruction**: `cd ros2_ws && colcon build --packages-select publisher_pkg && ros2 run publisher_pkg publisher_node_cpp`
- [ ] sp.task.0007 [P] [US2] Implement C++ ROS 2 subscriber node (`ros2_ws/src/subscriber_pkg/src/subscriber_node.cpp`)
    *   **run-instruction**: `cd ros2_ws && colcon build --packages-select subscriber_pkg && ros2 run subscriber_pkg subscriber_node_cpp`

## Phase 4: Documentation Integration

- [ ] sp.task.0010 [P] Integrate Python publisher/subscriber code examples into Docusaurus (`content/chapters/module1-ros2-nodes-graph.md`) (Dependencies: sp.task.0003, sp.task.0004, sp.task.0005)
- [ ] sp.task.0011 [P] Integrate Python service client/server code examples into Docusaurus (`content/chapters/module1-ros2-nodes-graph.md`) (Dependencies: sp.task.0003, sp.task.0008, sp.task.0009)

## Phase 5: CI/CD Workflow

- [ ] sp.task.0012 Develop GitHub Actions workflow for ROS 2 linting (`.github/workflows/ros2-docusaurus-ci.yml`)
- [ ] sp.task.0013 Develop GitHub Actions workflow for ROS 2 package compilation (`.github/workflows/ros2-docusaurus-ci.yml`) (Dependencies: sp.task.0012)
- [ ] sp.task.0014 Develop GitHub Actions workflow for ROS 2 unit tests (`.github/workflows/ros2-docusaurus-ci.yml`) (Dependencies: sp.task.0013)
- [ ] sp.task.0015 Develop GitHub Actions workflow for Docusaurus preview build (`.github/workflows/ros2-docusaurus-ci.yml`) (Dependencies: sp.task.0014)

## Phase 6: Advanced ROS 2 Communication & Finalization

- [ ] sp.task.0018 Implement ROS 2 action client and server examples (Python/C++) (`ros2_ws/src/`)
- [ ] sp.task.0019 Integrate action client/server code examples into Docusaurus (`content/chapters/module1-ros2-nodes-graph.md`) (Dependencies: sp.task.0003, sp.task.0018)
- [ ] sp.task.0016 Create and verify `rqt_graph` visualization instructions (`content/chapters/module1-ros2-nodes-graph.md`) (Dependencies: sp.task.0003)
- [ ] sp.task.0017 Finalize Module 1 content and formatting (`content/chapters/module1-ros2-nodes-graph.md`) (Dependencies: sp.task.0010, sp.task.0011, sp.task.0016, sp.task.0019)

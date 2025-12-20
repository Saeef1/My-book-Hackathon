// Mock data for module cards
// This represents the structure defined in the data-model.md file

const mockModulesData = [
  {
    id: 1,
    number: 1,
    title: "Introduction to Physical AI",
    description: "Overview of Physical AI and humanoid robotics fundamentals.",
    path: "/docs/physical-ai-humanoid-robotics/",
    imageUrl: null
  },
  {
    id: 2,
    number: 2,
    title: "ROS2 Nervous System",
    description: "Understanding the ROS2 framework for robot control and communication.",
    path: "/docs/physical-ai-humanoid-robotics/module1-ros2-nervous-system",
    imageUrl: null
  },
  {
    id: 3,
    number: 3,
    title: "Digital Twin Simulation",
    description: "Creating digital twins for robot simulation and testing.",
    path: "/docs/physical-ai-humanoid-robotics/module2-digital-twin",
    imageUrl: null
  },
  {
    id: 4,
    number: 4,
    title: "AI Robot Brain",
    description: "Developing AI algorithms for robot decision making and control.",
    path: "/docs/physical-ai-humanoid-robotics/module3-ai-robot-brain",
    imageUrl: null
  },
  {
    id: 5,
    number: 5,
    title: "Vision-Language-Action Models",
    description: "Implementing VLA models for robot perception and action.",
    path: "/docs/physical-ai-humanoid-robotics/module4-vla",
    imageUrl: null
  },
  {
    id: 6,
    number: 6,
    title: "Hardware Requirements",
    description: "Understanding the hardware components for humanoid robots.",
    path: "/docs/physical-ai-humanoid-robotics/hardware-requirements",
    imageUrl: null
  }
];

const homepageContent = {
  title: "Python Data Science Book",
  subtitle: "A comprehensive guide to using Python for data science",
  readMorePath: "/docs/physical-ai-humanoid-robotics/", // Updated to point to the physical AI humanoid robotics section
  modules: mockModulesData
};

export { mockModulesData, homepageContent };
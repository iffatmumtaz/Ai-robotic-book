// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 – The Robotic Nervous System',
      collapsible: true,
      collapsed: false,
      items: [
        'module1-ros2/lesson1-ros2-basics',
        'module1-ros2/lesson2-ros2-nodes-topics',
        'module1-ros2/lesson3-ros2-actions-services'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo + Unity – The Digital Twin',
      collapsible: true,
      collapsed: false,
      items: [
        'module2-gazebo-unity/lesson1-digital-twin-concepts',
        'module2-gazebo-unity/lesson2-gazebo-simulations',
        'module2-gazebo-unity/lesson3-unity-robotics'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac – The AI-Robot Brain',
      collapsible: true,
      collapsed: false,
      items: [
        'module3-nvidia-isaac/lesson1-isaac-sim-intro',
        'module3-nvidia-isaac/lesson2-isaac-perception-pipelines',
        'module3-nvidia-isaac/lesson3-isaac-robot-control'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: false,
      items: [
        'module4-vla/lesson1-llm-robot-interface',
        'module4-vla/lesson2-vla-action-sequences',
        'module4-vla/lesson3-advanced-vla-integration'
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      collapsible: true,
      collapsed: false,
      items: [
        'capstone-project/capstone-project-overview'
      ],
    }
  ],
};

module.exports = sidebars;
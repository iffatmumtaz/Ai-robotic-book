# Data Model: Physical AI & Humanoid Robotics Book

## Entities

### 1. Module
- **Description**: A major section of the book, comprising several lessons.
- **Fields**:
    - `id`: Unique identifier (e.g., `module1-ros2`)
    - `title`: Name of the module (e.g., "ROS 2 – The Robotic Nervous System")
    - `lessons`: List of Lesson entities associated with this module (3 lessons per module as per FR-002)
- **Relationships**: Contains Lessons.
- **Validation Rules**: Must contain exactly 3 lessons.

### 2. Lesson
- **Description**: A sub-section within a module, focusing on specific learning objectives and activities.
- **Fields**:
    - `id`: Unique identifier (e.g., `lesson1-ros2-basics`)
    - `title`: Title of the lesson
    - `description`: Brief overview of the lesson
    - `learningGoals`: Key objectives for the learner
    - `activity`: Hands-on lab or coding activity
    - `requiredToolsSoftware`: List of tools and software needed
    - `expectedOutcome`: What learners should achieve
- **Relationships**: Belongs to a Module.
- **Validation Rules**: All fields are mandatory (FR-003).

### 3. Capstone Project
- **Description**: A comprehensive, multi-module project integrating concepts learned throughout the book.
- **Fields**:
    - `title`: "Autonomous Humanoid with voice command → planning → navigation → object detection → manipulation"
    - `architecture`: High-level design of the project
    - `inputOutputFlow`: Description of data and control flow
    - `testingStages`: Phases of testing for the project
- **Relationships**: Integrates concepts from all Modules.
- **Validation Rules**: Must be fully defined with architecture, input/output flow, and testing stages (FR-006, SC-003).

### 4. Humanoid Robot
- **Description**: The primary subject of practical applications, can be physical or simulated.
- **Fields**:
    - `type`: Physical (e.g., Unitree Go2 / Unitree G1) or Simulated (e.g., in Gazebo/Isaac Sim)
    - `capabilities`: Sensing, perception, planning, action
- **Relationships**: Interacts with Simulation Environments and Hardware.

### 5. Simulation Environment
- **Description**: Software platforms for virtual robotics development.
- **Fields**:
    - `name`: (e.g., Gazebo, Unity, NVIDIA Isaac Sim)
    - `version`: Specific version of the simulation software
    - `purpose`: Development, testing, experimentation
- **Relationships**: Used for simulating Humanoid Robots, integrates with ROS 2 and NVIDIA Isaac.
- **Validation Rules**: Must support simulation workflows (FR-005).

### 6. Hardware
- **Description**: Physical components for real-world deployment.
- **Fields**:
    - `type`: (e.g., RTX workstation, Jetson Orin Kit, RealSense cameras, Unitree Go2 / Unitree G1 / proxy robots)
    - `requirements`: Minimum specifications
- **Relationships**: Used for physical deployment of Humanoid Robots.
- **Validation Rules**: Must be referenced with requirements (FR-013).

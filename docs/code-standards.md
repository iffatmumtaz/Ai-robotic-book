# Documentation Standards for Code Examples

## Purpose
This document establishes standards for code examples in the Physical AI & Humanoid Robotics Book to ensure consistency, readability, and technical accuracy across all lessons.

## Syntax Highlighting Standards

### Language Identifiers
Use the following language identifiers for syntax highlighting:

- `python` - Python code (ROS 2 nodes, scripts, LLM interfaces)
- `cpp` - C++ code (ROS 2, Gazebo, Isaac Sim)
- `bash` - Shell commands, terminal instructions
- `yaml` - YAML configuration files (ROS 2, Isaac Sim, Docusaurus)
- `json` - JSON configuration files
- `xml` - XML files (URDF, launch files)
- `javascript` - JavaScript (Docusaurus, web interfaces)
- `typescript` - TypeScript (Docusaurus, web interfaces)
- `dockerfile` - Docker configuration
- `cmake` - CMake build files
- `text` - Plain text that doesn't require highlighting
- `console` - Terminal output or console logs

### Code Block Format
````markdown
\```language
[Code content]
\```
````

## Code Example Structure

### 1. Complete, Runnable Examples
- Provide complete examples that can be run independently when possible
- Include necessary imports/headers at the beginning
- Use realistic variable names that reflect the context
- Include error handling where appropriate for production use

### 2. Minimal Examples for Learning
- Focus on the specific concept being taught
- Remove unnecessary complexity that doesn't support the learning objective
- Use clear, descriptive variable and function names
- Include comments to explain key points

### 3. Progressive Complexity
- Start with simple examples
- Gradually introduce complexity
- Reference earlier examples when building upon concepts
- Use consistent variable names across related examples

## Code Example Components

### 1. File Headers (When Applicable)
```python
#!/usr/bin/env python3
# Example ROS 2 node for Physical AI book
# Chapter: [Chapter Name]
# Lesson: [Lesson Name]
```

### 2. Imports/Dependencies
- Group related imports together
- Add comments for non-obvious imports
- Use standard import order where applicable

### 3. Comments and Documentation
- Use inline comments sparingly and only for non-obvious code
- Use block comments for complex explanations
- Document function/class purposes with docstrings where appropriate
- Use consistent comment style throughout

### 4. Naming Conventions
- Follow language-specific conventions (snake_case for Python, camelCase for JavaScript)
- Use descriptive names that reflect the purpose
- Maintain consistency with terminology used in the book
- Use consistent naming across related examples

## Code Example Categories

### 1. ROS 2 Examples
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

### 2. Gazebo/Isaac Sim Examples
```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### 3. Command Line Examples
```bash
# Install ROS 2 Humble Hawksbill
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 4. Configuration Files
```yaml
# ROS 2 parameter file
/**:
  ros__parameters:
    use_sim_time: true
    robot_model: "unitree_g1"
```

## Quality Standards

### 1. Accuracy
- All code examples must be tested and verified
- Include expected output where relevant
- Specify required dependencies
- Indicate if code is pseudo-code vs. actual executable code

### 2. Readability
- Use proper indentation (4 spaces for Python, 2 spaces for JSON/YAML)
- Break long lines appropriately
- Use whitespace to separate logical sections
- Maintain consistent formatting

### 3. Accessibility
- Use high-contrast syntax highlighting themes
- Ensure code examples are properly formatted for screen readers
- Include alternative text for code diagrams
- Provide text descriptions for complex code structures

### 4. Reproducibility
- Specify required software versions when important
- Include environment setup instructions
- Provide complete examples that can be replicated
- Include troubleshooting tips for common issues

## Common Patterns

### 1. Error Handling
```python
try:
    # ROS 2 initialization
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### 2. Configuration Loading
```python
def load_config(config_path):
    """Load configuration from YAML file."""
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config
    except FileNotFoundError:
        print(f"Configuration file {config_path} not found")
        return None
```

### 3. Resource Management
```python
# Proper cleanup in ROS 2 nodes
def destroy_node(self):
    # Clean up resources before destroying node
    if self.timer:
        self.timer.cancel()
    super().destroy_node()
```

## Validation Checklist

Before including any code example, verify:

- [ ] Uses correct syntax highlighting language identifier
- [ ] Follows language-specific style guides
- [ ] Includes necessary imports/dependencies
- [ ] Is tested and verified to work
- [ ] Includes appropriate comments for clarity
- [ ] Uses consistent naming conventions
- [ ] Matches the lesson's learning objectives
- [ ] Is formatted for readability
- [ ] Includes expected output when relevant
- [ ] Specifies required software versions if important
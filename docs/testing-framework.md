# Testing Framework for Code Examples and Simulation Workflows

## Purpose
This document establishes a comprehensive testing framework to validate code examples and simulation workflows in the Physical AI & Humanoid Robotics Book, ensuring technical accuracy and reproducibility.

## Testing Framework Overview

### 1. Testing Architecture
```
Testing Framework:
├── Unit Testing Layer
│   ├── Code Example Validation
│   ├── Command Execution Testing
│   └── Syntax Validation
├── Integration Testing Layer
│   ├── Simulation Workflow Testing
│   ├── Multi-component Integration
│   └── End-to-End Scenarios
└── Acceptance Testing Layer
    ├── Learning Objective Validation
    ├── Hands-on Lab Verification
    └── Capstone Project Testing
```

### 2. Testing Categories
- **Automated Tests**: Code examples, syntax validation, basic command execution
- **Semi-Automated Tests**: Simulation workflows with predefined scenarios
- **Manual Tests**: Complex integration scenarios, learning outcome validation
- **Performance Tests**: Resource usage, execution time, simulation performance

## Unit Testing Framework

### 1. Code Example Testing

#### A. Python Code Testing
```python
import unittest
import subprocess
import tempfile
import os

class TestPythonCodeExamples(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()

    def test_ros2_publisher_example(self):
        # Example code to test
        code = '''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, 'test_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    rclpy.spin(test_publisher)
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        '''

        # Write code to temporary file
        with open(os.path.join(self.temp_dir, 'test_publisher.py'), 'w') as f:
            f.write(code)

        # Test that code compiles without syntax errors
        result = subprocess.run(['python3', '-m', 'py_compile', os.path.join(self.temp_dir, 'test_publisher.py')],
                              capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, f"Code compilation failed: {result.stderr}")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.temp_dir)
```

#### B. Shell Command Testing
```bash
#!/bin/bash
# Test shell commands from book examples

test_ros2_installation() {
    echo "Testing ROS 2 installation commands..."

    # Test package update command
    if apt update &>/dev/null; then
        echo "✓ Package update command successful"
    else
        echo "✗ Package update command failed"
        return 1
    fi

    # Test ROS 2 setup
    if source /opt/ros/humble/setup.bash &>/dev/null; then
        echo "✓ ROS 2 setup successful"
    else
        echo "✗ ROS 2 setup failed"
        return 1
    fi

    return 0
}

test_workspace_creation() {
    echo "Testing workspace creation commands..."

    # Create temporary workspace
    local temp_ws=$(mktemp -d)
    cd $temp_ws

    # Test workspace directory creation
    mkdir -p src
    if [ -d "src" ]; then
        echo "✓ Workspace src directory created successfully"
    else
        echo "✗ Workspace src directory creation failed"
        return 1
    fi

    # Test colcon build
    if colcon build &>/dev/null; then
        echo "✓ Colcon build successful"
    else
        echo "✗ Colcon build failed"
        return 1
    fi

    cd - &>/dev/null
    rm -rf $temp_ws

    return 0
}
```

### 2. Testing Configuration Files
```python
import yaml
import json
import xml.etree.ElementTree as ET
import unittest

class TestConfigurationFiles(unittest.TestCase):
    def test_yaml_syntax(self):
        """Test YAML configuration file syntax"""
        yaml_content = """
ros__parameters:
  use_sim_time: true
  robot_model: "unitree_g1"
  control_frequency: 100
        """

        try:
            parsed = yaml.safe_load(yaml_content)
            self.assertIsNotNone(parsed)
        except yaml.YAMLError as e:
            self.fail(f"YAML syntax error: {e}")

    def test_json_syntax(self):
        """Test JSON configuration file syntax"""
        json_content = '''
{
    "simulation": {
        "gravity": -9.81,
        "real_time_factor": 1.0,
        "max_step_size": 0.001
    }
}
        '''

        try:
            parsed = json.loads(json_content)
            self.assertIsNotNone(parsed)
        except json.JSONDecodeError as e:
            self.fail(f"JSON syntax error: {e}")

    def test_xml_syntax(self):
        """Test XML configuration file syntax"""
        xml_content = """
<robot name="test_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
        </visual>
    </link>
</robot>
        """

        try:
            root = ET.fromstring(xml_content)
            self.assertIsNotNone(root)
        except ET.ParseError as e:
            self.fail(f"XML syntax error: {e}")
```

## Integration Testing Framework

### 1. Simulation Workflow Testing

#### A. Gazebo Simulation Testing
```python
import unittest
import subprocess
import time
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class TestGazeboSimulation(unittest.TestCase):
    def setUp(self):
        # Start Gazebo simulation in background
        self.gazebo_process = subprocess.Popen([
            'gz', 'sim', '-r', 'empty.sdf'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Wait for Gazebo to start
        time.sleep(5)

    def test_robot_spawn(self):
        """Test that robot can be spawned in simulation"""
        # Test spawning a simple robot
        spawn_cmd = [
            'gz', 'service', '-s', '/world/empty/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '500',
            '--req', 'sdf: "<sdf version=\'1.7\'><model name=\'test_robot\'><link name=\'link\'></link></model></sdf>"'
        ]

        result = subprocess.run(spawn_cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, f"Robot spawn failed: {result.stderr}")

    def tearDown(self):
        # Kill Gazebo process
        self.gazebo_process.terminate()
        self.gazebo_process.wait()
```

#### B. ROS 2 Integration Testing
```python
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class TestROS2Integration(Node):
    def __init__(self):
        super().__init__('test_integration_node')
        self.received_messages = []
        self.subscription = self.create_subscription(
            String,
            'test_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.received_messages.append(msg.data)

class IntegrationTestCase(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.test_node = TestROS2Integration()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.test_node)

        # Start executor in separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()

    def test_publisher_subscriber_integration(self):
        """Test publisher-subscriber communication"""
        publisher = self.test_node.create_publisher(String, 'test_topic', 10)

        # Wait for publisher to be ready
        time.sleep(1)

        # Publish test message
        msg = String()
        msg.data = 'test_message'
        publisher.publish(msg)

        # Wait for message to be received
        time.sleep(1)

        # Check if message was received
        self.assertIn('test_message', self.test_node.received_messages)

    def tearDown(self):
        self.executor.shutdown()
        rclpy.shutdown()
        self.executor_thread.join()
```

## Automated Testing Pipeline

### 1. Continuous Integration Configuration (.github/workflows/test.yml)
```yaml
name: Code Example Testing

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  test-code-examples:
    runs-on: ubuntu-22.04

    steps:
    - uses: actions/checkout@v3

    - name: Setup ROS 2 Humble
      uses: ros-tooling/setup-ros@v0.7
      with:
        required-ros-distributions: humble

    - name: Install dependencies
      run: |
        sudo apt update
        sudo apt install -y python3-colcon-common-extensions
        pip3 install pyyaml

    - name: Run code example tests
      run: |
        python3 -m pytest tests/code_examples/ -v

    - name: Run simulation workflow tests
      run: |
        python3 -m pytest tests/simulation_workflows/ -v
```

### 2. Test Organization Structure
```
tests/
├── code_examples/
│   ├── test_python_examples.py
│   ├── test_shell_commands.py
│   ├── test_config_files.py
│   └── test_build_systems.py
├── simulation_workflows/
│   ├── test_gazebo_workflows.py
│   ├── test_isaac_sim_workflows.py
│   ├── test_unity_workflows.py
│   └── test_hardware_interfaces.py
├── integration_tests/
│   ├── test_module_integration.py
│   ├── test_lesson_workflows.py
│   └── test_capstone_project.py
└── performance_tests/
    ├── test_simulation_performance.py
    ├── test_hardware_performance.py
    └── test_network_performance.py
```

## Testing Tools and Frameworks

### 1. Unit Testing
- **pytest**: Primary testing framework for Python code
- **unittest**: Built-in Python testing framework
- **bash_unit**: For shell script testing
- **goss**: For infrastructure and command testing

### 2. Simulation Testing
- **Gazebo Testing Framework**: Built-in Gazebo test tools
- **Ignition Testing**: For Ignition Gazebo (newer version)
- **Custom ROS 2 Test Nodes**: For ROS 2 specific testing

### 3. Performance Testing
- **ROS 2 Performance Testing**: Built-in tools for measuring performance
- **System Monitoring Tools**: For resource usage monitoring
- **Custom Benchmark Scripts**: For specific performance metrics

## Test Coverage Requirements

### 1. Code Example Coverage
- **100%** of code examples must be tested
- **95%** success rate in clean environments
- **All** syntax and execution errors must be caught
- **Each** example must be tested in isolation

### 2. Command Coverage
- **All** installation commands must be tested
- **All** configuration commands must be tested
- **All** execution commands must be tested
- **Prerequisites** must be validated before execution

### 3. Workflow Coverage
- **All** simulation workflows must be tested
- **All** multi-step processes must be validated
- **Error handling** must be tested
- **Alternative paths** must be validated

## Test Execution Strategy

### 1. Local Development Testing
```bash
# Run all tests locally
python3 -m pytest tests/ -v

# Run specific test file
python3 -m pytest tests/code_examples/test_python_examples.py -v

# Run tests with coverage
python3 -m pytest tests/ --cov=docs/ --cov-report=html

# Run tests in clean Docker container
docker run --rm -v $(pwd):/workspace -w /workspace python:3.10 bash -c "
    pip install -r requirements.txt &&
    python3 -m pytest tests/ -v
"
```

### 2. Clean Environment Testing
```bash
#!/bin/bash
# Script to run tests in clean environment

# Create clean Docker container
docker run --rm -it \
    --name ros2-test-container \
    -v $(pwd):/workspace \
    -w /workspace \
    osrf/ros:humble-desktop-full \
    bash -c "
        cd /workspace &&
        pip3 install pytest pyyaml &&
        python3 -m pytest tests/ -v
    "
```

## Test Reporting and Metrics

### 1. Test Results Format
```json
{
  "timestamp": "2023-12-08T10:30:00Z",
  "test_suite": "code_examples",
  "total_tests": 25,
  "passed": 24,
  "failed": 1,
  "skipped": 0,
  "execution_time": 45.2,
  "environment": "ubuntu-22.04-ros-humble",
  "details": [
    {
      "test_name": "test_ros2_publisher_example",
      "status": "PASS",
      "execution_time": 2.1
    },
    {
      "test_name": "test_invalid_syntax_example",
      "status": "FAIL",
      "error": "SyntaxError: invalid syntax",
      "execution_time": 0.5
    }
  ]
}
```

### 2. Dashboard Metrics
- Overall test pass rate
- Module-specific test results
- Performance metrics over time
- Resource usage statistics
- Test execution time trends

## Quality Gates and Validation

### 1. Pre-Merge Requirements
- All tests must pass before merging
- Code coverage must be >= 95%
- Performance tests must meet thresholds
- No new test failures introduced

### 2. Release Validation
- All modules must pass integration tests
- Performance tests must meet requirements
- Manual validation of learning objectives
- Accessibility compliance verification

## Maintenance and Updates

### 1. Test Maintenance Process
- Regular review of outdated tests
- Updates for new ROS 2 versions
- Maintenance for deprecated features
- Addition of new test scenarios

### 2. Test Evolution Strategy
- Add tests for new content continuously
- Update existing tests for new requirements
- Remove obsolete tests when content is removed
- Expand test coverage based on feedback

## Validation Checklist

Before finalizing the testing framework, verify:

- [ ] All code example types have corresponding tests
- [ ] Simulation workflows are properly tested
- [ ] Testing pipeline is automated and reliable
- [ ] Test coverage metrics are tracked
- [ ] Performance requirements are validated
- [ ] Clean environment testing is implemented
- [ ] Error handling is tested appropriately
- [ ] Documentation for test procedures exists
- [ ] Quality gates are defined and enforced
- [ ] Test reporting and metrics are implemented
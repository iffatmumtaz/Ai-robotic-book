---
sidebar_label: Nodes, Topics, and Communication
title: Lesson 2 - ROS 2 Nodes and Topics
---

# Lesson 2: Understanding Nodes, Topics, and Communication Patterns

## Overview

This lesson dives deeper into the core communication mechanisms in ROS 2. You'll learn about nodes, topics, publishers, subscribers, and how they work together to create distributed robotic systems.

## Learning Goals

By the end of this lesson, you will be able to:
- Create custom ROS 2 nodes with publishers and subscribers
- Understand the publisher-subscriber communication pattern
- Implement message passing between nodes
- Use ROS 2 tools to inspect and debug communication
- Design simple distributed systems using ROS 2 concepts

## Concepts

### Nodes
Nodes are the fundamental building blocks of ROS 2. Each node is a process that performs computation. Multiple nodes work together to form a complete robotic application.

### Topics
Topics are named buses over which nodes exchange messages. They implement a publish-subscribe communication pattern where publishers send data and subscribers receive it.

### Publishers and Subscribers
- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- Multiple publishers and subscribers can exist for the same topic
- Communication is asynchronous and decoupled

### Quality of Service (QoS)
QoS settings control how messages are delivered, including reliability, durability, and history policies.

## Steps

### Step 1: Create a Custom Package

```bash
# Navigate to your workspace
cd ~/ros2_ws/src

# Create a new package for our examples
ros2 pkg create --build-type ament_python my_robot_interfaces

# Navigate to the package directory
cd my_robot_interfaces
```

### Step 2: Create a Publisher Node

Create a new file `my_robot_interfaces/my_robot_interfaces/publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class RobotPublisher(Node):

    def __init__(self):
        super().__init__('robot_publisher')
        self.publisher = self.create_publisher(String, 'robot_commands', 10)
        self.timer = self.create_timer(1.0, self.publish_command)
        self.command_index = 0
        self.commands = [
            "MOVE_FORWARD",
            "TURN_LEFT",
            "MOVE_BACKWARD",
            "TURN_RIGHT",
            "STOP"
        ]

    def publish_command(self):
        msg = String()
        msg.data = self.commands[self.command_index % len(self.commands)]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.command_index += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = RobotPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create a Subscriber Node

Create a new file `my_robot_interfaces/my_robot_interfaces/subscriber.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotSubscriber(Node):

    def __init__(self):
        super().__init__('robot_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Robot subscriber node initialized')

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Here you would typically implement the command execution logic
        self.execute_command(msg.data)

    def execute_command(self, command):
        # Simulate command execution
        if command == "MOVE_FORWARD":
            self.get_logger().info('Moving robot forward...')
        elif command == "TURN_LEFT":
            self.get_logger().info('Turning robot left...')
        elif command == "MOVE_BACKWARD":
            self.get_logger().info('Moving robot backward...')
        elif command == "TURN_RIGHT":
            self.get_logger().info('Turning robot right...')
        elif command == "STOP":
            self.get_logger().info('Stopping robot...')


def main(args=None):
    rclpy.init(args=args)
    subscriber = RobotSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Update setup.py

Update `my_robot_interfaces/setup.py` to make your nodes executable:

```python
from setuptools import setup

package_name = 'my_robot_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Examples for ROS 2 nodes and topics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_publisher = my_robot_interfaces.publisher:main',
            'robot_subscriber = my_robot_interfaces.subscriber:main',
        ],
    },
)
```

### Step 5: Build and Run the Package

```bash
# Go back to the workspace root
cd ~/ros2_ws

# Source ROS 2 and build the package
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_interfaces

# Source the workspace
source install/setup.bash

# Run the publisher in one terminal
ros2 run my_robot_interfaces robot_publisher

# Run the subscriber in another terminal
ros2 run my_robot_interfaces robot_subscriber
```

## Code

### Complete Publisher Example with Error Handling:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class RobustPublisher(Node):

    def __init__(self):
        super().__init__('robust_publisher')

        # Create a QoS profile for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(String, 'robot_commands', qos_profile)
        self.timer = self.create_timer(0.5, self.publish_command)
        self.command_counter = 0

    def publish_command(self):
        msg = String()
        msg.data = f'Command #{self.command_counter}: MOVE_FORWARD'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.command_counter += 1


def main(args=None):
    rclpy.init(args=args)

    try:
        publisher = RobustPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print('Publisher interrupted by user')
    except Exception as e:
        print(f'Publisher error: {e}')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Examples

### Using ROS 2 Command Line Tools

```bash
# List all active topics
ros2 topic list

# Get information about a specific topic
ros2 topic info /robot_commands

# Echo messages from a topic (like our subscriber)
ros2 topic echo /robot_commands data

# Publish a single message to a topic
ros2 topic pub /robot_commands std_msgs/msg/String "data: 'MANUAL_COMMAND'"

# Show bandwidth usage of topics
ros2 topic bw

# Show the rate of messages on topics
ros2 topic hz /robot_commands
```

### Multiple Publishers Example

You can have multiple publishers for the same topic:

```python
# publisher_1.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher1(Node):
    def __init__(self):
        super().__init__('publisher_1')
        self.publisher = self.create_publisher(String, 'multi_topic', 10)
        self.timer = self.create_timer(2.0, self.publish_data)

    def publish_data(self):
        msg = String()
        msg.data = 'Data from Publisher 1'
        self.publisher.publish(msg)
        self.get_logger().info(f'P1: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Publisher1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Best Practices

1. **Use appropriate QoS settings** based on your application requirements
2. **Handle node lifecycle properly** with try/finally blocks
3. **Use descriptive topic names** that clearly indicate their purpose
4. **Implement proper error handling** for robust applications
5. **Test communication patterns** with ROS 2 command-line tools
6. **Use consistent message types** for predictable communication
7. **Consider message frequency** to avoid overwhelming the system
8. **Implement logging** for debugging and monitoring

## Required Tools & Software

- **ROS 2 Distribution**: ROS 2 Humble Hawksbill (or newer LTS)
- **Python**: Version 3.8 or higher
- **Development Environment**: Terminal, Text Editor or IDE
- **System Requirements**: At least 4GB RAM, 20GB free disk space
- **Additional Tools**: colcon build system, ament build system

## Expected Outcome

After completing this lesson, you should:
- Understand how nodes, topics, publishers, and subscribers work
- Be able to create custom ROS 2 packages with communication nodes
- Use ROS 2 command-line tools to inspect and debug communication
- Implement robust publisher-subscriber patterns with proper error handling
- Have a foundation for more complex communication patterns in ROS 2
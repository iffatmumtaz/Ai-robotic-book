---
sidebar_label: Introduction to ROS 2
title: Lesson 1 - ROS 2 Basics
---

# Lesson 1: Introduction to ROS 2 Architecture and Basic Commands

## Overview

This lesson introduces you to the Robot Operating System 2 (ROS 2), the middleware that connects robotic components. You'll learn about ROS 2 architecture, fundamental concepts, and basic commands that form the foundation of robotic development.

## Learning Goals

By the end of this lesson, you will be able to:
- Explain the core concepts of ROS 2 architecture
- Set up a basic ROS 2 workspace
- Execute fundamental ROS 2 commands
- Understand nodes, topics, and basic communication patterns
- Run your first ROS 2 example

## Concepts

ROS 2 (Robot Operating System 2) is not an operating system but a flexible framework for writing robotic software. It provides services designed for a heterogeneous computer cluster such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### Key ROS 2 Concepts:

1. **Nodes**: Processes that perform computation. ROS 2 is designed as a peer-to-peer network of nodes that can exchange messages.
2. **Topics**: Named buses over which nodes exchange messages.
3. **Messages**: Data structures used inside ROS for packaging data between nodes.
4. **Packages**: Basic building blocks of ROS 2 functionality.
5. **Workspaces**: Directories where you modify and build ROS 2 code.

## Steps

### Step 1: Verify ROS 2 Installation

First, let's make sure ROS 2 is properly installed and sourced:

```bash
# Check if ROS 2 is installed
printenv | grep -i ros

# Source the ROS 2 setup script (replace 'humble' with your ROS 2 distribution)
source /opt/ros/humble/setup.bash

# Or if you built from source:
# source ~/ros2_humble/install/setup.bash
```

### Step 2: Create a ROS 2 Workspace

```bash
# Create a workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build
```

### Step 3: Source the Workspace

```bash
# Source the workspace
source install/setup.bash
```

### Step 4: Run Basic ROS 2 Commands

```bash
# List available ROS 2 commands
ros2 --help

# Check ROS 2 daemon status
ros2 daemon status

# Get information about the ROS 2 environment
ros2 doctor
```

## Code

Let's create a simple ROS 2 publisher and subscriber to understand basic communication:

### Publisher Node (Python):

```python
# publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Node (Python):

```python
# subscriber_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Examples

### Running the Publisher and Subscriber

1. Open two terminal windows
2. In both terminals, source your ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```
3. In the first terminal, run the publisher:
   ```bash
   ros2 run demo_nodes_py talker
   ```
4. In the second terminal, run the subscriber:
   ```bash
   ros2 run demo_nodes_py listener
   ```

You should see the publisher sending messages and the subscriber receiving them.

## Best Practices

1. **Always source your ROS 2 environment** before running any ROS 2 commands
2. **Use descriptive names** for nodes, topics, and services
3. **Follow ROS 2 naming conventions** (lowercase with underscores)
4. **Handle node cleanup properly** to avoid resource leaks
5. **Use appropriate QoS settings** for your specific use case
6. **Test components in isolation** before integrating

## Required Tools & Software

- **ROS 2 Distribution**: ROS 2 Humble Hawksbill (or newer LTS) - Ubuntu 22.04
- **Operating System**: Ubuntu 22.04 LTS or equivalent
- **Python**: Version 3.8 or higher
- **Development Environment**: Terminal, Text Editor or IDE
- **System Requirements**: At least 4GB RAM, 20GB free disk space

## Expected Outcome

After completing this lesson, you should:
- Understand the fundamental concepts of ROS 2 architecture
- Be able to set up and source a ROS 2 workspace
- Execute basic ROS 2 commands
- Run simple publisher/subscriber examples
- Have a foundation for more advanced ROS 2 concepts
---
sidebar_label: Gazebo Simulation Environments
title: Lesson 2 - Gazebo Simulation Environments and Tools
---

# Lesson 2: Gazebo Simulation Environments and Tools

## Overview

This lesson provides a comprehensive introduction to Gazebo, a powerful robotics simulation environment. You'll learn how to create, configure, and use Gazebo worlds for robot development and testing, with a focus on humanoid robotics applications.

## Learning Goals

By the end of this lesson, you will be able to:
- Install and configure Gazebo for robotics simulation
- Create and customize Gazebo world files
- Spawn and control robot models in simulation
- Use Gazebo plugins for sensor integration and control
- Connect Gazebo to ROS 2 for integrated simulation workflows
- Implement basic robot control in Gazebo environments

## Concepts

### Gazebo Overview

Gazebo is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used in robotics research and development.

### Key Components of Gazebo:

1. **Physics Engine**: Simulates realistic physics (ODE, Bullet, Simbody)
2. **Rendering Engine**: Provides 3D visualization (OGRE)
3. **Sensor Simulation**: Simulates various sensors (cameras, LIDAR, IMU, etc.)
4. **Model Database**: Repository of robot and object models
5. **Plugins System**: Extensible architecture for custom functionality

### Gazebo-ROS Integration

Gazebo integrates seamlessly with ROS through plugins that allow:
- Publishing sensor data to ROS topics
- Subscribing to ROS topics for robot control
- Spawning and managing robots via ROS services

## Steps

### Step 1: Install Gazebo and Verify Installation

```bash
# For ROS 2 Humble, Gazebo Harmonic is recommended
# Update package list
sudo apt update

# Install Gazebo Harmonic
sudo apt install ros-humble-gazebo-*

# Verify installation
gz --version

# Launch Gazebo GUI
gz sim
```

### Step 2: Create a Basic Gazebo World

Create a world file `my_robot_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_robot_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sky -->
    <include>
      <uri>model://sky</uri>
    </include>

    <!-- Configure physics -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Add lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.4 -0.8</direction>
    </light>

    <!-- Add a simple box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.3 0.3 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### Step 3: Launch Gazebo with Custom World

```bash
# Launch Gazebo with your custom world
gz sim -r my_robot_world.sdf

# Or using ROS 2 launch (if using ROS 2 integration)
ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/my_robot_world.sdf
```

### Step 4: Create a Simple Robot Model

Create a robot model file `simple_robot.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_robot">
    <!-- Robot base link -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.0 0.8 0.0 1</ambient>
          <diffuse>0.0 0.8 0.0 1</diffuse>
        </material>
      </visual>

      <!-- Add a simple wheel -->
      <joint name="wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>wheel</child>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-1.5707</lower>
            <upper>1.5707</upper>
            <effort>10.0</effort>
            <velocity>1.0</velocity>
          </limit>
        </axis>
      </joint>
    </link>

    <link name="wheel">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.001</iyy>
          <iyz>0.0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>

      <collision name="wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

### Step 5: Add ROS 2 Control Plugin

To control the robot from ROS 2, you need to add plugins. Here's an example with a joint state publisher and controller:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_robot_with_plugins">
    <!-- ... (previous model content) ... -->

    <!-- Add a ROS 2 publisher plugin for joint states -->
    <plugin filename="libgazebo_ros_joint_state_publisher.so" name="gazebo_ros_joint_state_publisher">
      <ros>
        <namespace>/simple_robot</namespace>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>wheel_joint</joint_name>
    </plugin>

    <!-- Add a ROS 2 controller plugin -->
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot_description)/config/my_robot_control.yaml</parameters>
    </plugin>
  </model>
</sdf>
```

## Code

### ROS 2 Node to Control Gazebo Robot:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math


class GazeboRobotController(Node):
    """Controller for a robot in Gazebo simulation"""

    def __init__(self):
        super().__init__('gazebo_robot_controller')

        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64, '/simple_robot/wheel_joint/cmd_vel', 10)

        # Publisher for twist commands (if using diff drive plugin)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/simple_robot/cmd_vel', 10)

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/simple_robot/joint_states', self.joint_state_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Robot state
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.target_velocity = 0.5  # rad/s

        self.get_logger().info('Gazebo robot controller initialized')

    def joint_state_callback(self, msg: JointState):
        """Update robot state from joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_joint_velocities[name] = msg.velocity[i]

    def control_loop(self):
        """Main control loop"""
        # Example: Send a velocity command to the wheel joint
        cmd_msg = Float64()
        cmd_msg.data = self.target_velocity
        self.joint_cmd_pub.publish(cmd_msg)

        # Or send twist command for differential drive
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.2  # m/s
        twist_cmd.angular.z = 0.1  # rad/s
        self.cmd_vel_pub.publish(twist_cmd)

        self.get_logger().debug(f'Sent velocity command: {self.target_velocity} rad/s')

    def change_velocity(self, new_velocity):
        """Change target velocity"""
        self.target_velocity = new_velocity
        self.get_logger().info(f'Target velocity changed to: {new_velocity} rad/s')


def main(args=None):
    rclpy.init(args=args)
    controller = GazeboRobotController()

    # Change velocity after 5 seconds
    def change_speed():
        controller.change_velocity(1.0)

    timer = controller.create_timer(5.0, change_speed)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Launch File for Gazebo + Robot:

```xml
<!-- my_robot_gazebo/launch/my_robot_world.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='my_robot_world.sdf')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='my_robot_world.sdf',
        description='Choose one of the world files from `/my_robot_gazebo/worlds`'
    )

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_gazebo'),
                'worlds',
                world
            ]),
            'verbose': 'true'
        }.items()
    )

    # Robot spawn node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )

    # Joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'simple_robot.urdf'
        ])]
    )

    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gazebo,
        spawn_entity,
        joint_state_publisher,
        robot_state_publisher,
    ])
```

## Examples

### Using Gazebo Command Line Tools:

```bash
# List all models in the current world
gz model --info

# Get information about a specific model
gz model --model-name simple_robot --info

# Set model pose
gz model --model-name simple_robot --pose 1 2 0.1 0 0 0

# Apply force to a model
gz model --model-name simple_robot --force 0 0 10

# Get physics information
gz physics --info
```

### Common Gazebo Worlds:

1. **Empty World**: Basic environment with ground plane
2. **Maze World**: For navigation testing
3. **Warehouse World**: For logistics robot testing
4. **City World**: For outdoor navigation scenarios

### Working with Gazebo Models:

```bash
# List available models
ls /usr/share/gazebo-*/models

# Or from the Gazebo Model Database online
# https://app.gazebosim.org/database/models
```

## Best Practices

1. **Physics Configuration**: Adjust physics parameters based on your robot's requirements
2. **Realistic Sensors**: Use accurate sensor models that match your physical sensors
3. **Model Quality**: Balance model complexity with simulation performance
4. **Validation**: Regularly validate simulation results against physical tests
5. **World Design**: Create worlds that match your target deployment environments
6. **Plugin Management**: Use appropriate plugins for your specific use case
7. **Performance**: Optimize models and worlds for real-time simulation
8. **Safety**: Use simulation for testing dangerous scenarios safely

## Required Tools & Software

- **Gazebo**: Gazebo Harmonic (for ROS 2 Humble) or equivalent
- **ROS 2**: For integration with robotics frameworks
- **System Requirements**: OpenGL 2.1+ capable GPU, 8GB+ RAM recommended
- **Modeling Tools**: For creating custom robot models (optional)
- **Development Environment**: Text editor for SDF/URDF files

## Expected Outcome

After completing this lesson, you should:
- Understand how to install and configure Gazebo for robotics simulation
- Be able to create custom Gazebo worlds and robot models
- Know how to connect Gazebo to ROS 2 for integrated workflows
- Have experience controlling robots in Gazebo simulation
- Be prepared to use Gazebo for more complex humanoid robotics applications
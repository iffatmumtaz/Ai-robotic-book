---
sidebar_label: Introduction to NVIDIA Isaac Sim
title: Lesson 1 - Isaac Sim Introduction and Setup
---

# Lesson 1: Introduction to NVIDIA Isaac Sim and Setup

## Overview

This lesson introduces NVIDIA Isaac Sim, a powerful robotics simulation platform built on NVIDIA Omniverse. You'll learn about Isaac Sim's capabilities for robotics development, perception pipeline creation, and AI training, with a focus on humanoid robotics applications.

## Learning Goals

By the end of this lesson, you will be able to:
- Understand the architecture and capabilities of NVIDIA Isaac Sim
- Install and configure Isaac Sim for robotics development
- Navigate the Isaac Sim interface and basic workflows
- Create simple robot simulations in Isaac Sim
- Connect Isaac Sim to ROS 2 for integrated robotics workflows
- Implement basic perception pipelines using Isaac Sim's tools

## Concepts

### NVIDIA Isaac Sim Overview

NVIDIA Isaac Sim is a comprehensive robotics simulation environment that provides:
- High-fidelity physics simulation using PhysX
- Realistic rendering with RTX technology
- Integrated AI training capabilities
- Perception pipeline development tools
- Seamless integration with ROS 2 and Isaac ROS
- Support for complex humanoid robotics scenarios

### Key Features of Isaac Sim:

1. **Omniverse Platform**: Built on NVIDIA's Omniverse for collaborative 3D simulation
2. **RTX Rendering**: Realistic lighting and materials for synthetic data generation
3. **PhysX Physics**: Accurate physics simulation for robotics applications
4. **ROS 2 Bridge**: Native integration with ROS 2 for robotics workflows
5. **Perception Tools**: Synthetic data generation for AI model training
6. **AI Training Environment**: Reinforcement learning and imitation learning support

### Isaac Sim vs Other Simulation Platforms:

- **Graphics Quality**: Superior to Gazebo due to RTX rendering
- **Physics**: Uses PhysX engine (highly accurate for robotics)
- **AI Integration**: Built-in tools for AI model training
- **Performance**: Requires powerful GPU hardware
- **Cost**: Commercial license required for production use

## Steps

### Step 1: System Requirements and Installation

First, ensure your system meets the requirements:

```bash
# System Requirements:
# - NVIDIA GPU with RTX capability (RTX 30xx, RTX 40xx, or professional GPUs)
# - CUDA-compatible GPU (Compute Capability 6.0+)
# - At least 16GB RAM (32GB+ recommended)
# - 100GB+ free disk space
# - Ubuntu 20.04 or 22.04 (or Windows 10/11)

# Check GPU compatibility
nvidia-smi
# Should show CUDA version and GPU info

# Check CUDA installation
nvcc --version
```

### Step 2: Install Isaac Sim

```bash
# Method 1: Using Isaac Sim Docker (Recommended for beginners)
docker pull nvcr.io/nvidia/isaac-sim:4.0.0

# Method 2: Local installation (requires more setup)
# 1. Download Isaac Sim from NVIDIA Developer website
# 2. Extract to desired location
# 3. Install dependencies as per documentation
```

### Step 3: Launch Isaac Sim

```bash
# Using Docker (recommended)
xhost +local:docker
docker run --gpus all -it --rm \
  --network=host \
  --env "DISPLAY" \
  --env "QT_X11_NO_MITSHM=1" \
  --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume "$HOME/.Xauthority:/root/.Xauthority" \
  --volume "/:/isaac-sim/host-root:ro" \
  --privileged \
  --pid=host \
  nvcr.io/nvidia/isaac-sim:4.0.0

# For local installation
./isaac-sim/python.sh -m omni.isaac.kit --exec apps/omni.isaac.sim.python.sh
```

### Step 4: Basic Isaac Sim Interface

Once Isaac Sim is running, familiarize yourself with the interface:

1. **Viewport**: 3D scene view where simulations run
2. **Stage Panel**: Hierarchical view of scene objects
3. **Property Panel**: Object properties and configuration
4. **Timeline**: Animation and simulation controls
5. **Console**: Python command line and logging
6. **Extension Manager**: Manage Isaac Sim extensions

### Step 5: Create a Simple Robot Simulation

Create a basic robot using Python scripting:

```python
# basic_robot.py - Example script for Isaac Sim
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np


class BasicRobotSimulation:
    def __init__(self):
        # Create the world instance
        self.world = World(stage_units_in_meters=1.0)

        # Add a simple robot to the scene
        self._setup_scene()

    def _setup_scene(self):
        # Add a simple robot from the asset library
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("Could not find Isaac Sim assets. Please check your installation.")
            return

        # Add a simple wheeled robot
        robot_path = assets_root_path + "/Isaac/Robots/Carter/carter.modelUSD.usd"
        add_reference_to_stage(
            usd_path=robot_path,
            prim_path="/World/Robot"
        )

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Set initial camera view
        set_camera_view(eye=np.array([2, 2, 2]), target=np.array([0, 0, 0]))

    def run(self):
        # Reset the world
        self.world.reset()

        # Run simulation for a number of steps
        for i in range(1000):
            if i % 100 == 0:
                print(f"Simulation step: {i}")

            # Step the world
            self.world.step(render=True)

            # Add your robot control logic here
            # For now, just let the simulation run with default physics

        print("Simulation completed")


def main():
    sim = BasicRobotSimulation()
    sim.run()


if __name__ == "__main__":
    main()
```

### Step 6: Connect to ROS 2

Isaac Sim provides built-in ROS 2 bridge functionality:

```bash
# In Isaac Sim, enable the ROS 2 Bridge extension:
# Window -> Extensions -> Isaac ROS -> ROS2 Bridge -> Enable

# Then you can use ROS 2 commands to interact with the simulation
# Example: List available topics
ros2 topic list

# Example: Echo robot joint states
ros2 topic echo /isaac_sim/robot/joint_states sensor_msgs/msg/JointState
```

## Code

### Complete Isaac Sim Robot Controller Example:

```python
# isaac_sim_robot_controller.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
import carb


class IsaacSimRobotController:
    def __init__(self):
        # Initialize the world
        self.world = World(stage_units_in_meters=1.0)
        self.robot = None
        self.initialized = False

        # Setup the scene
        self._setup_scene()

    def _setup_scene(self):
        """Setup the simulation scene with robot and environment"""
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets. Please check your installation.")
            return

        # Add a more complex robot (Franka Emika Panda for example)
        robot_path = assets_root_path + "/Isaac/Robots/Franka/franka_instanceable.usd"

        # Add robot to stage
        add_reference_to_stage(
            usd_path=robot_path,
            prim_path="/World/Robot"
        )

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Set camera view
        set_camera_view(eye=np.array([2.5, 2.5, 2.0]), target=np.array([0, 0, 0.5]))

        # Add a simple cube for the robot to interact with
        from omni.isaac.core.objects import DynamicCuboid
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="cube",
                position=np.array([0.5, 0.5, 0.5]),
                size=0.1,
                color=np.array([0.5, 0, 0])
            )
        )

        self.initialized = True

    def initialize_robot(self):
        """Initialize the robot articulation after world reset"""
        if not self.initialized:
            return False

        # Reset the world first
        self.world.reset()

        # Get the robot as an articulation
        self.robot = self.world.scene.get_object("Robot")
        if not isinstance(self.robot, Articulation):
            carb.log_error("Robot is not an articulation")
            return False

        # Initialize the robot
        self.robot.initialize(world_prim=self.world.scene)
        carb.log_info("Robot initialized successfully")
        return True

    def move_robot_joints(self, joint_positions):
        """Move robot joints to specified positions"""
        if self.robot is None:
            carb.log_error("Robot not initialized")
            return False

        # Apply joint positions
        self.robot.apply_articulation_actions(
            ArticulationAction(joint_positions=joint_positions)
        )
        return True

    def get_robot_state(self):
        """Get current robot state"""
        if self.robot is None:
            return None

        # Get joint positions
        joint_positions = self.robot.get_joint_positions()
        joint_velocities = self.robot.get_joint_velocities()

        return {
            'positions': joint_positions,
            'velocities': joint_velocities
        }

    def run_simulation(self, steps=1000):
        """Run the simulation for specified steps"""
        if not self.initialize_robot():
            return

        print("Starting simulation...")

        for i in range(steps):
            # Step the world
            self.world.step(render=True)

            # Example: Move joints periodically
            if i % 100 == 0:
                # Simple joint movement example
                joint_pos = [0.0] * self.robot.num_dof  # Initialize all joints to 0
                if len(joint_pos) > 0:
                    joint_pos[0] = np.sin(i * 0.01)  # Move first joint sinusoidally
                    self.move_robot_joints(joint_pos)

                # Print robot state occasionally
                state = self.get_robot_state()
                if state:
                    print(f"Step {i}: Joint positions = {state['positions'][:3]}...")  # Show first 3 joints

        print("Simulation completed")


def main():
    # Create and run the robot controller
    controller = IsaacSimRobotController()

    # Run the simulation
    controller.run_simulation(steps=2000)

    # Clean up
    controller.world.clear()


if __name__ == "__main__":
    main()
```

### Isaac Sim ROS 2 Integration Example:

```python
# isaac_sim_ros_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time


class IsaacSimROSBridge(Node):
    """ROS 2 node that interfaces with Isaac Sim"""

    def __init__(self):
        super().__init__('isaac_sim_ros_bridge')

        # Publishers for Isaac Sim
        self.joint_state_pub = self.create_publisher(JointState, '/isaac_sim/joint_states', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/isaac_sim/cmd_vel', 10)

        # Subscribers from other ROS nodes
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/robot/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/robot/joint_commands', self.joint_cmd_callback, 10)

        # Timer for publishing simulated sensor data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # 10 Hz

        self.last_cmd_vel = Twist()
        self.last_joint_cmd = JointState()

        self.get_logger().info('Isaac Sim ROS Bridge initialized')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS"""
        self.last_cmd_vel = msg
        self.get_logger().debug(f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')

        # In a real implementation, this would send the command to Isaac Sim
        # For now, we'll just log it
        self.send_cmd_to_isaac_sim(msg)

    def joint_cmd_callback(self, msg):
        """Handle joint commands from ROS"""
        self.last_joint_cmd = msg
        self.get_logger().debug(f'Received {len(msg.position)} joint commands')

        # Send joint commands to Isaac Sim
        self.send_joints_to_isaac_sim(msg)

    def send_cmd_to_isaac_sim(self, cmd_vel):
        """Send velocity command to Isaac Sim (placeholder implementation)"""
        # In a real implementation, this would use Isaac Sim's ROS bridge
        # or direct API calls to control the simulated robot
        pass

    def send_joints_to_isaac_sim(self, joint_cmd):
        """Send joint commands to Isaac Sim (placeholder implementation)"""
        # In a real implementation, this would use Isaac Sim's API
        # to set joint positions in the simulation
        pass

    def publish_sensor_data(self):
        """Publish simulated sensor data from Isaac Sim"""
        # Create and publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3']  # Example joint names
        joint_msg.position = [0.1, 0.2, 0.3]  # Example positions
        joint_msg.velocity = [0.0, 0.0, 0.0]  # Example velocities
        joint_msg.effort = [0.0, 0.0, 0.0]    # Example efforts

        self.joint_state_pub.publish(joint_msg)

    def run_isaac_sim_loop(self):
        """Main loop for Isaac Sim integration (would run in separate thread)"""
        # This would typically run the Isaac Sim simulation loop
        # and handle communication between ROS and Isaac Sim
        pass


def main(args=None):
    rclpy.init(args=args)
    bridge_node = IsaacSimROSBridge()

    # In a real implementation, you'd want to run Isaac Sim in parallel
    # This is a simplified example
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Isaac Sim ROS Bridge interrupted')
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Examples

### Isaac Sim Command Line Tools:

```bash
# Launch Isaac Sim with specific app
./isaac-sim/python.sh -m omni.isaac.kit --exec apps/omni.isaac.sim.python.sh

# Run a specific Python script in Isaac Sim
./isaac-sim/python.sh my_simulation_script.py

# Enable extensions via command line
./isaac-sim/python.sh -c "import omni; omni.kit.app.get_app().extension_manager.set_extension_enabled('omni.isaac.ros2_bridge', True)"
```

### Common Isaac Sim Workflows:

1. **Perception Pipeline Development**: Create synthetic training data for computer vision models
2. **Robot Control Testing**: Test control algorithms in realistic physics simulation
3. **AI Training**: Use reinforcement learning to train robot behaviors
4. **Hardware-in-the-loop**: Connect physical sensors to simulation

### Isaac Sim Asset Library:

Isaac Sim includes a rich library of assets:
- Robots: Franka Emika, Carter, Stretch, etc.
- Environments: Warehouse, Office, Outdoor scenes
- Objects: Household items, tools, obstacles
- Sensors: Cameras, LIDAR, IMU models

## Best Practices

1. **Hardware Requirements**: Ensure adequate GPU resources for realistic rendering
2. **Scene Optimization**: Use Level of Detail (LOD) and occlusion culling for performance
3. **Physics Tuning**: Carefully tune physics parameters for realistic behavior
4. **Asset Management**: Organize custom assets in a structured manner
5. **Extension Usage**: Enable only necessary extensions to improve performance
6. **Scripting**: Use Python scripting for complex simulation behaviors
7. **Validation**: Regularly validate simulation results against physical tests
8. **Documentation**: Maintain clear documentation for Isaac Sim workflows

## Required Tools & Software

- **NVIDIA GPU**: RTX series or professional GPU with CUDA support
- **Isaac Sim**: Latest version from NVIDIA Developer website
- **CUDA**: Appropriate version for your GPU
- **System Requirements**: Ubuntu 20.04/22.04 or Windows 10/11, 32GB+ RAM
- **ROS 2**: For integration with robotics frameworks
- **Development Environment**: Python IDE for scripting

## Expected Outcome

After completing this lesson, you should:
- Understand the capabilities and architecture of NVIDIA Isaac Sim
- Be able to install and configure Isaac Sim for robotics development
- Know how to create basic robot simulations in Isaac Sim
- Understand how to connect Isaac Sim to ROS 2 for integrated workflows
- Be prepared to use Isaac Sim for complex humanoid robotics applications
- Have a foundation for developing perception pipelines and AI training environments
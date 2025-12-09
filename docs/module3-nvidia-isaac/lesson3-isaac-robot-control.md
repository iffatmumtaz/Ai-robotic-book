---
sidebar_label: Isaac Sim Robot Control
title: Lesson 3 - Robot Control and Navigation with Isaac
---

# Lesson 3: Robot Control and Navigation with Isaac Sim

## Overview

This lesson focuses on implementing robot control and navigation systems using NVIDIA Isaac Sim. You'll learn how to create control algorithms, implement navigation behaviors, and connect these systems to ROS 2 for integrated robotics workflows. The lesson emphasizes practical implementation of control systems for humanoid robotics applications.

## Learning Goals

By the end of this lesson, you will be able to:
- Implement robot control algorithms in Isaac Sim
- Create navigation behaviors using path planning and obstacle avoidance
- Integrate control systems with perception for closed-loop operation
- Connect Isaac Sim control systems to ROS 2 navigation stack
- Validate control performance in simulation before real-world deployment
- Understand the principles of robot control for humanoid applications

## Concepts

### Robot Control Fundamentals

Robot control involves generating appropriate commands to achieve desired robot behavior. Key concepts include:

1. **Open-Loop Control**: Commands sent without feedback
2. **Closed-Loop Control**: Commands adjusted based on sensor feedback
3. **PID Control**: Proportional-Integral-Derivative control for precise positioning
4. **Trajectory Following**: Following predefined paths with smooth motion
5. **Impedance Control**: Controlling robot compliance and interaction forces

### Navigation in Robotics

Navigation systems enable robots to move autonomously in environments:

1. **Global Path Planning**: Finding paths from start to goal using known maps
2. **Local Path Planning**: Avoiding obstacles in real-time during navigation
3. **Localization**: Determining robot position in the environment
4. **Mapping**: Creating representations of the environment
5. **SLAM**: Simultaneous Localization and Mapping

### Isaac Sim Control Architecture:

1. **Articulation Control**: Low-level joint control for articulated robots
2. **Differential Drive**: Control for wheeled robots
3. **Leg Control**: Specialized control for legged robots
4. **ROS Bridge**: Integration with ROS control and navigation stacks
5. **AI Control**: Reinforcement learning and imitation learning for complex behaviors

## Steps

### Step 1: Understanding Isaac Sim Control Interfaces

Isaac Sim provides multiple ways to control robots:

```python
# isaac_control_interfaces.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np


class IsaacControlInterfaces:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.robot = None
        self.initialized = False

        self._setup_scene()
        self._initialize_robot()

    def _setup_scene(self):
        """Setup the scene with robot and environment"""
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("Could not find Isaac Sim assets")
            return

        # Add a simple wheeled robot
        robot_path = assets_root_path + "/Isaac/Robots/Carter/carter.modelUSD.usd"
        add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")

        # Add ground plane and obstacles
        self.world.scene.add_default_ground_plane()

        from omni.isaac.core.objects import FixedCuboid
        self.world.scene.add(
            FixedCuboid(
                prim_path="/World/Obstacle",
                name="obstacle",
                position=np.array([1.0, 0.0, 0.5]),
                size=0.5,
                color=np.array([0.8, 0.2, 0.2])
            )
        )

        self.world.reset()

    def _initialize_robot(self):
        """Initialize robot control interface"""
        try:
            # Get robot as articulation
            self.robot = self.world.scene.get_object("Robot")
            if self.robot is None:
                print("Could not find robot in scene")
                return

            # Initialize the articulation
            self.robot.initialize(world_prim=self.world.scene)
            self.initialized = True
            print("Robot initialized successfully")
        except Exception as e:
            print(f"Error initializing robot: {e}")

    def position_control(self, joint_positions):
        """Control robot joints to reach desired positions"""
        if not self.initialized or self.robot is None:
            return False

        try:
            # Apply position commands to all joints
            self.robot.apply_articulation_actions(
                ArticulationAction(joint_positions=joint_positions)
            )
            return True
        except Exception as e:
            print(f"Error in position control: {e}")
            return False

    def velocity_control(self, joint_velocities):
        """Control robot joints with desired velocities"""
        if not self.initialized or self.robot is None:
            return False

        try:
            # Apply velocity commands
            self.robot.apply_articulation_actions(
                ArticulationAction(joint_velocities=joint_velocities)
            )
            return True
        except Exception as e:
            print(f"Error in velocity control: {e}")
            return False

    def effort_control(self, joint_efforts):
        """Control robot joints with desired efforts/torques"""
        if not self.initialized or self.robot is None:
            return False

        try:
            # Apply effort commands
            self.robot.apply_articulation_actions(
                ArticulationAction(joint_efforts=joint_efforts)
            )
            return True
        except Exception as e:
            print(f"Error in effort control: {e}")
            return False

    def get_robot_state(self):
        """Get current robot state"""
        if not self.initialized or self.robot is None:
            return None

        try:
            # Get joint states
            joint_positions = self.robot.get_joint_positions()
            joint_velocities = self.robot.get_joint_velocities()
            joint_efforts = self.robot.get_joint_efforts()

            # Get base pose
            world_pos, world_ori = self.robot.get_world_pose()

            return {
                'positions': joint_positions,
                'velocities': joint_velocities,
                'efforts': joint_efforts,
                'world_position': world_pos,
                'world_orientation': world_ori
            }
        except Exception as e:
            print(f"Error getting robot state: {e}")
            return None


def main():
    control_system = IsaacControlInterfaces()

    # Example: Move robot joints to zero position
    if control_system.initialized:
        zero_positions = [0.0] * control_system.robot.num_dof
        control_system.position_control(zero_positions)

        # Run simulation to see the effect
        for i in range(100):
            control_system.world.step(render=True)

    print("Control interface example completed")


if __name__ == "__main__":
    main()
```

### Step 2: Implementing Differential Drive Control

```python
# differential_drive_control.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.robots import WheeledRobot
import numpy as np


class DifferentialDriveController:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.robot = None
        self.controller = None
        self.initialized = False

        self._setup_scene()
        self._initialize_robot()

    def _setup_scene(self):
        """Setup scene with wheeled robot"""
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("Could not find Isaac Sim assets")
            return

        # Add a wheeled robot (Carter)
        robot_path = assets_root_path + "/Isaac/Robots/Carter/carter.modelUSD.usd"
        add_reference_to_stage(usd_path=robot_path, prim_path="/World/Carter")

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add some obstacles
        from omni.isaac.core.objects import FixedCuboid
        for i in range(3):
            self.world.scene.add(
                FixedCuboid(
                    prim_path=f"/World/Obstacle{i}",
                    name=f"obstacle{i}",
                    position=np.array([i*2.0, 1.0, 0.5]),
                    size=0.5,
                    color=np.array([0.8, 0.2, 0.2])
                )
            )

        self.world.reset()

    def _initialize_robot(self):
        """Initialize the wheeled robot and controller"""
        try:
            # Create wheeled robot object
            self.robot = WheeledRobot(
                prim_path="/World/Carter",
                name="carter",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True
            )

            # Add to world
            self.world.scene.add(self.robot)

            # Initialize controller with parameters
            self.controller = DifferentialController(
                name="simple_control",
                wheel_radius=0.1,  # Carter wheel radius
                wheel_base=0.4,    # Distance between wheels
                max_linear_velocity=1.0,
                max_angular_velocity=np.pi
            )

            # Initialize the world
            self.world.reset()
            self.initialized = True
            print("Differential drive robot initialized successfully")
        except Exception as e:
            print(f"Error initializing robot: {e}")

    def move_robot(self, linear_velocity, angular_velocity, duration=1.0):
        """Move robot with specified linear and angular velocities"""
        if not self.initialized:
            return

        # Calculate number of steps for duration
        steps = int(duration / self.world.get_physics_dt())

        for _ in range(steps):
            # Get current robot state
            current_action = self.controller.forward(
                current_joint_velocities=self.robot.get_joints_state().joint_velocities,
                current_joint_positions=self.robot.get_joints_state().joint_positions,
                joint_efforts=None,
                target_linear_velocity=linear_velocity,
                target_angular_velocity=angular_velocity
            )

            # Apply action to robot
            self.robot.apply_action(current_action)

            # Step the world
            self.world.step(render=True)

    def navigate_to_waypoint(self, target_x, target_y):
        """Navigate robot to a specific waypoint using simple control"""
        if not self.initialized:
            return

        # Get current robot position
        current_pos, _ = self.robot.get_world_pose()
        current_x, current_y = current_pos[0], current_pos[1]

        # Calculate desired direction
        dx = target_x - current_x
        dy = target_y - current_y
        distance = np.sqrt(dx*dx + dy*dy)

        if distance < 0.1:  # Already at target
            print("Already at target position")
            return

        # Calculate target angle
        target_angle = np.arctan2(dy, dx)

        # Get current orientation
        _, current_rot = self.robot.get_world_pose()
        current_angle = np.arctan2(
            2 * (current_rot[3] * current_rot[2] + current_rot[0] * current_rot[1]),
            1 - 2 * (current_rot[1]**2 + current_rot[2]**2)
        )

        # Calculate angular error
        angle_error = target_angle - current_angle
        # Normalize angle to [-pi, pi]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        # Simple proportional control
        linear_vel = min(0.5, distance)  # Slow down as we approach
        angular_vel = 2.0 * angle_error  # Proportional control for orientation

        print(f"Moving to ({target_x}, {target_y}), distance: {distance:.2f}, angle_error: {angle_error:.2f}")

        # Move robot
        self.move_robot(linear_vel, angular_vel, duration=0.5)


def main():
    nav_controller = DifferentialDriveController()

    if nav_controller.initialized:
        # Navigate to a waypoint
        nav_controller.navigate_to_waypoint(2.0, 1.0)

        # Move forward a bit more
        nav_controller.move_robot(linear_velocity=0.5, angular_velocity=0.0, duration=2.0)

    print("Differential drive control example completed")


if __name__ == "__main__":
    main()
```

### Step 3: Implementing Advanced Navigation with Perception Integration

```python
# perception_control_integration.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
import numpy as np
import cv2


class PerceptionControlIntegration:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.robot = None
        self.controller = None
        self.camera = None
        self.initialized = False

        self._setup_scene()
        self._initialize_system()

    def _setup_scene(self):
        """Setup scene with robot, camera, and environment"""
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("Could not find Isaac Sim assets")
            return

        # Add robot
        robot_path = assets_root_path + "/Isaac/Robots/Carter/carter.modelUSD.usd"
        add_reference_to_stage(usd_path=robot_path, prim_path="/World/Carter")

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add objects to detect
        from omni.isaac.core.objects import DynamicCuboid
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Target",
                name="target",
                position=np.array([2.0, 0.0, 0.5]),
                size=0.3,
                color=np.array([0.0, 1.0, 0.0])  # Green target
            )
        )

        self.world.reset()

    def _initialize_system(self):
        """Initialize robot and camera"""
        try:
            # Initialize robot
            self.robot = WheeledRobot(
                prim_path="/World/Carter",
                name="carter",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True
            )
            self.world.scene.add(self.robot)

            # Initialize controller
            self.controller = DifferentialController(
                name="nav_control",
                wheel_radius=0.1,
                wheel_base=0.4,
                max_linear_velocity=1.0,
                max_angular_velocity=np.pi
            )

            # Add camera to robot
            self.camera = Camera(
                prim_path="/World/Carter/Looks/visual/camera",
                frequency=30,
                resolution=(320, 240)
            )

            # If camera path doesn't work, try alternative
            if not self.camera.is_valid():
                self.camera = Camera(
                    prim_path="/World/Carter/camera",
                    frequency=30,
                    resolution=(320, 240)
                )
                self.camera.set_translation(np.array([0.2, 0.0, 0.1]))

            # Reset world
            self.world.reset()
            self.initialized = True
            print("Perception-control system initialized successfully")
        except Exception as e:
            print(f"Error initializing system: {e}")

    def detect_target_in_image(self, image):
        """Detect green target in image using color filtering"""
        if image is None:
            return None

        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Define range for green color
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Minimum area threshold
                # Get bounding box
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w/2
                center_y = y + h/2
                image_width = image.shape[1]

                # Calculate horizontal offset (-1 to 1) where 0 is center
                offset = (center_x - image_width/2) / (image_width/2)

                return {
                    'offset': offset,
                    'area': cv2.contourArea(largest_contour),
                    'bbox': (x, y, w, h)
                }

        return None

    def navigate_with_perception(self, max_steps=500):
        """Navigate robot using perception feedback"""
        if not self.initialized:
            return

        print("Starting perception-based navigation...")

        for step in range(max_steps):
            # Step the world
            self.world.step(render=True)

            # Get camera image
            rgb_image = self.camera.get_rgb()

            if rgb_image is not None:
                # Detect target in image
                target_info = self.detect_target_in_image(rgb_image)

                if target_info:
                    # Target detected - navigate towards it
                    offset = target_info['offset']
                    area = target_info['area']

                    # Calculate control based on target position
                    linear_vel = 0.5  # Move forward
                    angular_vel = -1.5 * offset  # Turn towards target

                    # Slow down as we get closer (based on target size)
                    if area > 5000:  # Target is large (close)
                        linear_vel = 0.2

                    print(f"Step {step}: Target detected, offset: {offset:.2f}, area: {area:.0f}")
                else:
                    # No target detected - search for it
                    linear_vel = 0.0
                    angular_vel = 0.5  # Turn in place to search

                    print(f"Step {step}: Searching for target...")
            else:
                # No image available - use default search behavior
                linear_vel = 0.0
                angular_vel = 0.5

            # Apply control commands
            if self.robot and self.controller:
                try:
                    current_action = self.controller.forward(
                        current_joint_velocities=self.robot.get_joints_state().joint_velocities,
                        current_joint_positions=self.robot.get_joints_state().joint_positions,
                        joint_efforts=None,
                        target_linear_velocity=linear_vel,
                        target_angular_velocity=angular_vel
                    )
                    self.robot.apply_action(current_action)
                except:
                    # If controller fails, try direct velocity control
                    pass

            # Check if we're close to the target
            robot_pos, _ = self.robot.get_world_pose()
            target_pos = np.array([2.0, 0.0, 0.5])
            distance = np.linalg.norm(robot_pos[:2] - target_pos[:2])

            if distance < 0.3:
                print(f"Reached target! Distance: {distance:.2f}")
                break

        print("Navigation completed")


def main():
    perception_nav = PerceptionControlIntegration()

    if perception_nav.initialized:
        perception_nav.navigate_with_perception(max_steps=1000)

    print("Perception-control integration example completed")


if __name__ == "__main__":
    main()
```

### Step 4: ROS 2 Integration for Navigation

```python
# isaac_navigation_ros.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import numpy as np
import math


class IsaacNavigationBridge(Node):
    """Bridge between Isaac Sim navigation and ROS 2"""

    def __init__(self):
        super().__init__('isaac_navigation_bridge')

        # Publishers for navigation data
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Subscribers for navigation commands
        self.nav_goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.nav_goal_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_input', self.cmd_vel_input_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing simulated data
        self.nav_timer = self.create_timer(0.1, self.publish_navigation_data)

        # Robot state tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        self.get_logger().info('Isaac Navigation Bridge initialized')

    def nav_goal_callback(self, msg):
        """Handle navigation goal from ROS"""
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y

        self.get_logger().info(f'Received navigation goal: ({target_x}, {target_y})')

        # In a real implementation, this would send the goal to Isaac Sim
        # For this example, we'll just store it
        self.navigate_to_goal(target_x, target_y)

    def cmd_vel_input_callback(self, msg):
        """Handle velocity commands from ROS"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

        self.get_logger().debug(f'Received cmd_vel: linear={self.linear_vel}, angular={self.angular_vel}')

    def navigate_to_goal(self, target_x, target_y):
        """Navigate to specified goal coordinates"""
        # Calculate direction to goal
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance > 0.1:  # If not already at goal
            target_angle = math.atan2(dy, dx)

            # Calculate angular error
            angle_error = target_angle - self.robot_theta
            # Normalize angle to [-pi, pi]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            # Simple proportional control
            self.linear_vel = min(0.5, distance)  # Move faster when farther away
            self.angular_vel = 2.0 * angle_error   # Turn toward goal

    def publish_navigation_data(self):
        """Publish navigation-related data to ROS"""
        # Update robot position based on current velocities
        dt = 0.1  # Time step from timer
        self.robot_x += self.linear_vel * math.cos(self.robot_theta) * dt
        self.robot_y += self.linear_vel * math.sin(self.robot_theta) * dt
        self.robot_theta += self.angular_vel * dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (convert theta to quaternion)
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, self.robot_theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel

        self.odom_pub.publish(odom_msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

        # Publish laser scan (simulated)
        laser_msg = self._generate_simulated_laser_scan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser_frame'
        self.laser_pub.publish(laser_msg)

    def _generate_simulated_laser_scan(self):
        """Generate simulated laser scan data"""
        laser_msg = LaserScan()
        laser_msg.angle_min = -math.pi / 2
        laser_msg.angle_max = math.pi / 2
        laser_msg.angle_increment = math.pi / 180  # 1 degree
        laser_msg.time_increment = 0.0
        laser_msg.scan_time = 0.1
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0

        # Generate 181 readings for 180 degrees at 1 degree increments
        num_readings = 181
        laser_msg.ranges = [float('inf')] * num_readings

        # Add some simulated obstacles
        for i in range(num_readings):
            angle = laser_msg.angle_min + i * laser_msg.angle_increment

            # Simulate a circular obstacle in front of robot
            obstacle_distance = 2.0  # meters away
            obstacle_size = 1.0      # meters in diameter

            # If this ray hits the obstacle
            if abs(angle) < math.asin(obstacle_size / (2 * obstacle_distance)):
                laser_msg.ranges[i] = obstacle_distance

        return laser_msg


def main(args=None):
    rclpy.init(args=args)
    nav_bridge = IsaacNavigationBridge()

    try:
        rclpy.spin(nav_bridge)
    except KeyboardInterrupt:
        nav_bridge.get_logger().info('Navigation bridge interrupted')
    finally:
        nav_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Code

### Complete Isaac Sim Navigation System with Control and Perception:

```python
# complete_navigation_system.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera, LidarRtx
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import threading
import time


class IsaacNavigationSystem(Node):
    """Complete navigation system combining Isaac Sim with ROS 2"""

    def __init__(self):
        super().__init__('isaac_navigation_system')

        # ROS components
        self.bridge = CvBridge()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/rgb', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Subscribers
        self.nav_goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.nav_goal_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_input', self.cmd_vel_input_callback, 10)

        # Isaac Sim components
        self.world = World(stage_units_in_meters=1.0)
        self.robot = None
        self.controller = None
        self.camera = None
        self.lidar = None
        self.synthetic_data_helper = None

        # Robot state
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Setup Isaac Sim
        self._setup_isaac_sim()

        # Navigation thread
        self.nav_thread = threading.Thread(target=self._navigation_loop, daemon=True)
        self.nav_thread.start()

        self.get_logger().info('Complete Isaac Navigation System initialized')

    def _setup_isaac_sim(self):
        """Setup Isaac Sim environment and sensors"""
        try:
            # Setup scene
            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                self.get_logger().error("Could not find Isaac Sim assets")
                return

            # Add robot
            robot_path = assets_root_path + "/Isaac/Robots/Carter/carter.modelUSD.usd"
            add_reference_to_stage(usd_path=robot_path, prim_path="/World/Carter")

            # Add ground plane
            self.world.scene.add_default_ground_plane()

            # Add obstacles
            from omni.isaac.core.objects import FixedCuboid
            for i in range(5):
                self.world.scene.add(
                    FixedCuboid(
                        prim_path=f"/World/Obstacle{i}",
                        name=f"obstacle{i}",
                        position=np.array([i*1.5 - 3.0, 2.0, 0.5]),
                        size=0.5,
                        color=np.array([0.8, 0.2, 0.2])
                    )
                )

            # Initialize world
            self.world.reset()

            # Initialize robot
            self.robot = WheeledRobot(
                prim_path="/World/Carter",
                name="carter",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True
            )
            self.world.scene.add(self.robot)

            # Initialize controller
            self.controller = DifferentialController(
                name="complete_control",
                wheel_radius=0.1,
                wheel_base=0.4,
                max_linear_velocity=1.0,
                max_angular_velocity=np.pi
            )

            # Add sensors
            self.camera = Camera(
                prim_path="/World/Carter/camera",
                frequency=30,
                resolution=(640, 480)
            )
            self.camera.set_translation(np.array([0.2, 0.0, 0.1]))

            # Initialize synthetic data helper
            self.synthetic_data_helper = SyntheticDataHelper()
            self.synthetic_data_helper.enable_data_type("rgb", device="cpu")

            self.get_logger().info("Isaac Sim navigation system setup completed")

        except Exception as e:
            self.get_logger().error(f"Error in Isaac Sim setup: {e}")

    def nav_goal_callback(self, msg):
        """Handle navigation goal from ROS"""
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        self.get_logger().info(f"Received navigation goal: ({target_x}, {target_y})")

        # In a real implementation, this would initiate path planning
        # For this example, we'll use simple proportional navigation
        self.navigate_to_goal(target_x, target_y)

    def cmd_vel_input_callback(self, msg):
        """Handle velocity commands from ROS"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def navigate_to_goal(self, target_x, target_y):
        """Navigate to specified goal with obstacle avoidance"""
        # Calculate direction to goal
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = np.sqrt(dx*dx + dy*dy)

        if distance > 0.2:  # If not already at goal
            target_angle = np.arctan2(dy, dx)

            # Calculate angular error
            current_pos, current_rot = self.robot.get_world_pose()
            current_angle = np.arctan2(
                2 * (current_rot[3] * current_rot[2] + current_rot[0] * current_rot[1]),
                1 - 2 * (current_rot[1]**2 + current_rot[2]**2)
            )

            angle_error = target_angle - current_angle
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

            # Simple navigation with obstacle avoidance
            self.linear_vel = min(0.5, distance)  # Move faster when farther away
            self.angular_vel = 2.0 * angle_error   # Turn toward goal

    def _navigation_loop(self):
        """Main navigation loop running in separate thread"""
        rate = self.world.get_physics_dt()

        while True:
            try:
                # Step Isaac Sim world
                self.world.step(render=True)

                # Update robot control based on current velocities
                if self.robot and self.controller:
                    try:
                        current_action = self.controller.forward(
                            current_joint_velocities=self.robot.get_joints_state().joint_velocities,
                            current_joint_positions=self.robot.get_joints_state().joint_positions,
                            joint_efforts=None,
                            target_linear_velocity=self.linear_vel,
                            target_angular_velocity=self.angular_vel
                        )
                        self.robot.apply_action(current_action)
                    except Exception as e:
                        self.get_logger().debug(f"Control error: {e}")

                # Get sensor data
                self._publish_sensor_data()

                # Update robot state for ROS
                self._update_robot_state()

                time.sleep(rate)

            except Exception as e:
                self.get_logger().error(f"Error in navigation loop: {e}")
                time.sleep(0.1)  # Brief pause on error

    def _publish_sensor_data(self):
        """Publish sensor data to ROS"""
        # Publish camera image
        try:
            rgb_image = self.camera.get_rgb()
            if rgb_image is not None:
                ros_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'camera_rgb_optical_frame'
                self.camera_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().debug(f"Camera error: {e}")

        # Publish simulated laser scan
        laser_msg = self._generate_simulated_laser_scan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser_frame'
        self.laser_pub.publish(laser_msg)

    def _update_robot_state(self):
        """Update robot state for ROS publishing"""
        try:
            # Get robot pose from Isaac Sim
            pos, rot = self.robot.get_world_pose()
            self.robot_x, self.robot_y = pos[0], pos[1]

            # Convert quaternion to euler for theta
            self.robot_theta = np.arctan2(
                2 * (rot[3] * rot[2] + rot[0] * rot[1]),
                1 - 2 * (rot[1]**2 + rot[2]**2)
            )

            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = float(self.robot_x)
            odom_msg.pose.pose.position.y = float(self.robot_y)
            odom_msg.pose.pose.position.z = float(pos[2])

            # Convert to quaternion
            from tf_transformations import quaternion_from_euler
            quat = quaternion_from_euler(0, 0, float(self.robot_theta))
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]

            # Velocity
            odom_msg.twist.twist.linear.x = float(self.linear_vel)
            odom_msg.twist.twist.angular.z = float(self.angular_vel)

            self.odom_pub.publish(odom_msg)

        except Exception as e:
            self.get_logger().debug(f"State update error: {e}")

    def _generate_simulated_laser_scan(self):
        """Generate simulated laser scan with obstacle detection"""
        laser_msg = LaserScan()
        laser_msg.angle_min = -np.pi / 2
        laser_msg.angle_max = np.pi / 2
        laser_msg.angle_increment = np.pi / 180
        laser_msg.time_increment = 0.0
        laser_msg.scan_time = 0.1
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0

        num_readings = 181
        laser_msg.ranges = [float('inf')] * num_readings

        # In a real implementation, this would use Isaac Sim's LIDAR
        # For this example, we'll simulate based on robot position
        robot_pos, _ = self.robot.get_world_pose() if self.robot else (np.array([0, 0, 0]), np.array([0, 0, 0, 1]))

        for i in range(num_readings):
            angle = laser_msg.angle_min + i * laser_msg.angle_increment
            # Add simulated obstacles based on environment
            if abs(angle) < 0.5:  # Forward direction
                laser_msg.ranges[i] = 3.0  # Simulate obstacle 3m ahead

        return laser_msg


def main(args=None):
    rclpy.init(args=args)
    nav_system = IsaacNavigationSystem()

    try:
        rclpy.spin(nav_system)
    except KeyboardInterrupt:
        nav_system.get_logger().info('Navigation system interrupted')
    finally:
        nav_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Examples

### Navigation in Different Scenarios:

1. **Indoor Navigation**:
   - Use occupancy grid maps for path planning
   - Implement dynamic obstacle avoidance
   - Integrate with SLAM systems for unknown environments

2. **Outdoor Navigation**:
   - Handle uneven terrain and slopes
   - Use GPS and IMU for localization
   - Implement weather-based navigation adjustments

3. **Humanoid Robot Navigation**:
   - Plan paths considering robot's physical constraints
   - Implement safe human-robot interaction protocols
   - Use legged locomotion controllers for complex terrain

### Control Strategies:

```python
# Example: PID Controller for Robot Control
class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative

        self.prev_error = error

        return p_term + i_term + d_term
```

## Best Practices

1. **Safety First**: Always implement safety checks and emergency stops
2. **Modular Design**: Separate perception, planning, and control components
3. **Parameter Tuning**: Carefully tune control parameters for your specific robot
4. **Testing**: Extensively test in simulation before real-world deployment
5. **Documentation**: Maintain clear documentation of control algorithms
6. **Performance Monitoring**: Track control performance metrics
7. **Error Handling**: Implement robust error handling and recovery
8. **Validation**: Validate control systems with various scenarios

## Required Tools & Software

- **Isaac Sim**: Latest version with navigation extensions
- **ROS 2 Navigation Stack**: For advanced navigation capabilities
- **Python Libraries**: NumPy, OpenCV, SciPy for control algorithms
- **System Requirements**: Sufficient computational power for real-time control
- **Development Environment**: Python IDE for control system development

## Expected Outcome

After completing this lesson, you should:
- Understand how to implement robot control systems in Isaac Sim
- Be able to create navigation behaviors with obstacle avoidance
- Know how to integrate perception with control for closed-loop operation
- Have experience with ROS 2 navigation stack integration
- Be prepared to validate control systems in simulation before real-world deployment
- Understand the principles of robot control for humanoid robotics applications
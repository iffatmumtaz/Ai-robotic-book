---
sidebar_label: Actions and Services
title: Lesson 3 - ROS 2 Actions and Services
---

# Lesson 3: Advanced Communication with Actions and Services

## Overview

This lesson covers advanced communication patterns in ROS 2: services and actions. While topics provide asynchronous, decoupled communication, services offer synchronous request-response patterns, and actions provide goal-oriented communication with feedback and status updates.

## Learning Goals

By the end of this lesson, you will be able to:
- Implement ROS 2 services for synchronous request-response communication
- Create and use ROS 2 actions for goal-oriented tasks with feedback
- Understand when to use topics, services, or actions appropriately
- Design complex robotic workflows using these communication patterns
- Handle errors and timeouts in service and action calls

## Concepts

### Services
Services provide synchronous request-response communication between nodes. A service client sends a request and waits for a response from a service server.

- **Synchronous**: Client waits for the response
- **Request-Response**: One request, one response
- **Good for**: Simple queries, configuration changes, immediate responses

### Actions
Actions are designed for long-running tasks that require:
- **Goal**: What the action should do
- **Feedback**: Progress updates during execution
- **Result**: Final outcome when the action completes
- **Status**: Current state of the action (active, succeeded, canceled, etc.)

### When to Use Each Pattern
- **Topics**: Continuous data streams, event notifications
- **Services**: One-time queries, configuration changes
- **Actions**: Long-running tasks, processes with progress tracking

## Steps

### Step 1: Create a Service Example

First, let's create a simple service that calculates the distance between two points:

Create a new file `my_robot_interfaces/my_robot_interfaces/distance_service.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger  # We'll use Trigger for simplicity
import math


class DistanceService(Node):

    def __init__(self):
        super().__init__('distance_service')
        self.srv = self.create_service(
            Trigger,
            'calculate_distance',
            self.calculate_distance_callback
        )
        self.get_logger().info('Distance service server started')

    def calculate_distance_callback(self, request, response):
        # In a real implementation, this would calculate distance
        # between two points provided in the request
        self.get_logger().info('Calculating distance...')
        response.success = True
        response.message = 'Distance calculated successfully'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DistanceService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2: Create a Service Client

Create a client that calls the distance service:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger


class DistanceClient(Node):

    def __init__(self):
        super().__init__('distance_client')
        self.cli = self.create_client(Trigger, 'calculate_distance')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    client = DistanceClient()
    response = client.send_request()

    if response is not None:
        print(f'Result: {response.success}, Message: {response.message}')
    else:
        print('Service call failed')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create an Action Example

For actions, we'll create a navigation action that moves a robot to a goal position with feedback:

First, create the action interface by creating a new package for custom messages:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_msgs
```

Then create the action file `my_robot_msgs/action/NavigateToPose.action`:

```
# Define the goal
geometry_msgs/Point target_pose
geometry_msgs/Quaternion target_orientation

---
# Define the result
bool success
string message

---
# Define the feedback
float32 distance_remaining
float32 progress_percentage
string status_message
```

### Step 4: Create an Action Server

Create `my_robot_interfaces/my_robot_interfaces/navigation_action_server.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

# Assuming you have the NavigateToPose action defined
# For this example, we'll use the built-in FollowJointTrajectory action
# or create a simple custom action
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class NavigationActionServer(Node):

    def __init__(self):
        super().__init__('navigation_action_server')

        # Create action server with a callback group for reentrancy
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            result_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            cancel_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            feedback_pub_qos_profile=rclpy.qos.QoSProfile(depth=10)
        )

        self.get_logger().info('Navigation action server started')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received navigation goal')

        # Simulate navigation process
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = ['joint1', 'joint2']

        # Simulate navigation steps
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation canceled')
                return FollowJointTrajectory.Result()

            # Update feedback
            feedback_msg.desired = JointTrajectoryPoint()
            feedback_msg.desired.time_from_start = Duration(sec=i)
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Navigation progress: {i * 10}%')
            time.sleep(1)  # Simulate work

        # Complete the goal
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0  # SUCCESS
        result.error_string = 'Navigation completed successfully'

        self.get_logger().info('Navigation completed successfully')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()

    try:
        # Use a multi-threaded executor to handle action callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Create an Action Client

Create `my_robot_interfaces/my_robot_interfaces/navigation_action_client.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time

# Using FollowJointTrajectory as an example
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class NavigationActionClient(Node):

    def __init__(self):
        super().__init__('navigation_action_client')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'navigate_to_pose'
        )

    def send_goal(self):
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create the goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2']

        # Add trajectory points
        point = JointTrajectoryPoint()
        point.positions = [1.0, 2.0]
        point.time_from_start.sec = 5
        goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending navigation goal...')

        # Send the goal and get a future
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_string}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationActionClient()

    # Send the goal
    action_client.send_goal()

    # Use a multi-threaded executor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()


if __name__ == '__main__':
    main()
```

## Code

### Complete Service Example with Error Handling:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from rclpy.qos import QoSProfile
import threading
import time


class RobustServiceServer(Node):

    def __init__(self):
        super().__init__('robust_service_server')

        # Create service with custom QoS
        qos_profile = QoSProfile(depth=10)
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback,
            qos_profile=qos_profile
        )

        self.get_logger().info('Robust service server started')

    def add_two_ints_callback(self, request, response):
        try:
            # Simulate processing time
            time.sleep(0.1)

            # Perform the calculation
            result = request.a + request.b

            # Validate the result
            if result > 2**31 - 1:  # Check for overflow
                response.sum = 2**31 - 1
                self.get_logger().warn('Result clamped due to overflow')
            else:
                response.sum = result

            self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')

        except Exception as e:
            self.get_logger().error(f'Service error: {e}')
            response.sum = 0  # Return safe default

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobustServiceServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Service interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Service error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Examples

### Using ROS 2 Command Line Tools for Services and Actions

```bash
# List all services
ros2 service list

# Get information about a specific service
ros2 service info /add_two_ints

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# List all actions
ros2 action list

# Get information about a specific action
ros2 action info /navigate_to_pose

# Send a goal to an action (if available)
ros2 action send_goal /navigate_to_pose control_msgs/action/FollowJointTrajectory "{...}"
```

### Service vs Action Comparison

```python
# Service - Synchronous, request-response
def service_example():
    # Client sends request and waits for response
    response = service_client.call(request)
    # Process response immediately
    process_response(response)

# Action - Asynchronous, goal-oriented with feedback
def action_example():
    # Client sends goal and continues execution
    goal_handle = action_client.send_goal(goal)

    # Can check status or receive feedback
    while not goal_handle.is_done:
        feedback = goal_handle.get_feedback()
        update_ui_with_feedback(feedback)
        time.sleep(0.1)

    # Finally get the result
    result = goal_handle.get_result()
    process_result(result)
```

## Best Practices

1. **Use services for simple, quick operations** that return immediately
2. **Use actions for long-running tasks** that need progress tracking
3. **Implement proper timeout handling** for service calls
4. **Use appropriate QoS settings** for your communication needs
5. **Handle cancellation requests** in action servers gracefully
6. **Provide meaningful feedback** in action implementations
7. **Design robust error handling** for all communication patterns
8. **Consider the execution model** (single-threaded vs multi-threaded)
9. **Validate inputs and outputs** to prevent errors
10. **Log communication events** for debugging

## Required Tools & Software

- **ROS 2 Distribution**: ROS 2 Humble Hawksbill (or newer LTS)
- **Python**: Version 3.8 or higher
- **Development Environment**: Terminal, Text Editor or IDE
- **System Requirements**: At least 4GB RAM, 20GB free disk space
- **Additional Tools**: colcon build system, action interface definition tools

## Expected Outcome

After completing this lesson, you should:
- Understand the differences between topics, services, and actions
- Be able to implement service servers and clients
- Create and use action servers and clients
- Choose the appropriate communication pattern for your use case
- Handle errors and timeouts in service and action implementations
- Have a solid foundation for complex robotic communication architectures
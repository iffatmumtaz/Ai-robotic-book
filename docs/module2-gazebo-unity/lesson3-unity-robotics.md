---
sidebar_label: Unity Robotics Integration
title: Lesson 3 - Unity Robotics Integration and Simulation
---

# Lesson 3: Unity Robotics Integration and Simulation

## Overview

This lesson introduces Unity as a robotics simulation platform, focusing on the Unity Robotics ecosystem including Unity Robotics Hub, ROS# communication bridge, and the Unity Perception package. You'll learn how to set up Unity for robotics simulation and create realistic environments for robot development.

## Learning Goals

By the end of this lesson, you will be able to:
- Install and configure Unity for robotics applications
- Use Unity Robotics Hub for project management
- Implement ROS# communication bridge between Unity and ROS 2
- Create realistic robotics simulation environments in Unity
- Integrate sensors and actuators in Unity simulations
- Connect Unity simulations to ROS 2 for integrated workflows

## Concepts

### Unity in Robotics

Unity is a powerful game engine that has been adapted for robotics simulation and development. It offers:
- High-quality 3D graphics and realistic rendering
- Flexible physics simulation
- Extensive asset library
- Cross-platform deployment capabilities
- Strong community and documentation

### Unity Robotics Ecosystem:

1. **Unity Robotics Hub**: Centralized package management for robotics tools
2. **ROS# (ROS Sharp)**: Communication bridge between Unity and ROS
3. **Unity Perception**: Tools for generating synthetic training data
4. **Unity ML-Agents**: Framework for developing intelligent agents
5. **URDF Importer**: Tool for importing ROS robot models

### Unity vs Gazebo Comparison:

- **Graphics Quality**: Unity offers more advanced rendering capabilities
- **Physics**: Gazebo has more robotics-focused physics simulation
- **Ease of Use**: Unity has a more intuitive visual editor
- **Community**: Gazebo has more robotics-specific community
- **Performance**: Depends on specific use case and configuration

## Steps

### Step 1: Install Unity and Robotics Packages

```bash
# Download Unity Hub from unity.com
# Unity Hub manages Unity installations and projects

# Install Unity Editor (2021.3 LTS or newer recommended)
# During installation, include:
# - Universal Render Pipeline (URP) or High Definition Render Pipeline (HDRP)
# - Visual Studio Tools for Unity (if using Visual Studio)
# - Android Build Support (if targeting mobile)

# After installation, open Unity Hub and install Unity Robotics Edition
# Or manually install the following packages via Unity Package Manager:
# - ROS# (ROS Sharp)
# - Unity Perception
# - ML-Agents (if needed)
```

### Step 2: Set Up a Unity Robotics Project

1. Open Unity Hub
2. Create a new 3D project
3. In the Package Manager (Window > Package Manager):
   - Install "ROS# (ROS Sharp)"
   - Install "Unity Perception"
   - Install "URDF Importer" (for importing ROS robot models)

### Step 3: Configure ROS# Communication Bridge

Create a new C# script `RosConnector.cs`:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;

public class RosConnector : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    public string RosBridgeWebSocketUrl = "ws://192.168.1.100:9090"; // Replace with your ROS bridge address
    public float ReconnectTimeout = 5.0f;

    private RosSocket rosSocket;
    private bool isConnected = false;

    void Start()
    {
        ConnectToRos();
    }

    void ConnectToRos()
    {
        try
        {
            rosSocket = new RosSocket(new RosSharp.WebSocketNetTransport(RosBridgeWebSocketUrl));
            isConnected = true;
            Debug.Log("Connected to ROS bridge: " + RosBridgeWebSocketUrl);
        }
        catch (System.Exception e)
        {
            Debug.LogError("Failed to connect to ROS bridge: " + e.Message);
            Invoke(nameof(ConnectToRos), ReconnectTimeout); // Retry connection
        }
    }

    void OnApplicationQuit()
    {
        rosSocket?.Close();
    }

    public bool IsConnected()
    {
        return isConnected && rosSocket != null;
    }

    public RosSocket GetRosSocket()
    {
        return rosSocket;
    }
}
```

### Step 4: Create a Simple Robot Controller

Create a robot controller script `UnityRobotController.cs`:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;

public class UnityRobotController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public float linearVelocity = 1.0f;
    public float angularVelocity = 1.0f;

    [Header("ROS Topics")]
    public string cmdVelTopic = "/unity_robot/cmd_vel";
    public string odomTopic = "/unity_robot/odom";

    private RosConnector rosConnector;
    private Publisher<Messages.Geometry.Twist> cmdVelPublisher;
    private Subscriber<Messages.Nav.Odometry> odomSubscriber;

    void Start()
    {
        rosConnector = FindObjectOfType<RosConnector>();
        if (rosConnector == null)
        {
            Debug.LogError("ROS Connector not found in scene!");
            return;
        }

        // Initialize ROS publishers and subscribers
        InitializeRosCommunication();
    }

    void InitializeRosCommunication()
    {
        if (rosConnector.IsConnected())
        {
            // Create publisher for velocity commands
            cmdVelPublisher = rosConnector.GetRosSocket().Advertise<Messages.Geometry.Twist>(cmdVelTopic);

            // Create subscriber for odometry
            odomSubscriber = rosConnector.GetRosSocket().Subscribe<Messages.Nav.Odometry>(odomTopic, OnOdometryReceived);

            Debug.Log("ROS communication initialized for Unity robot");
        }
    }

    void OnOdometryReceived(Messages.Nav.Odometry odomMsg)
    {
        // Update robot position based on received odometry
        Vector3 position = new Vector3((float)odomMsg.pose.pose.position.x,
                                      (float)odomMsg.pose.pose.position.y,
                                      (float)odomMsg.pose.pose.position.z);
        Quaternion rotation = new Quaternion((float)odomMsg.pose.pose.orientation.x,
                                           (float)odomMsg.pose.pose.orientation.y,
                                           (float)odomMsg.pose.pose.orientation.z,
                                           (float)odomMsg.pose.pose.orientation.w);

        transform.position = position;
        transform.rotation = rotation;
    }

    public void SendVelocityCommand(float linear, float angular)
    {
        if (cmdVelPublisher != null && rosConnector.IsConnected())
        {
            var twistMsg = new Messages.Geometry.Twist();
            twistMsg.linear = new Messages.Geometry.Vector3 { x = linear, y = 0, z = 0 };
            twistMsg.angular = new Messages.Geometry.Vector3 { x = 0, y = 0, z = angular };

            cmdVelPublisher.Publish(twistMsg);
        }
    }

    void Update()
    {
        // Example: Move robot with keyboard input
        float moveX = Input.GetAxis("Vertical") * linearVelocity * Time.deltaTime;
        float rotateY = Input.GetAxis("Horizontal") * angularVelocity * Time.deltaTime;

        // Send command to ROS
        SendVelocityCommand(moveX, rotateY);

        // Local movement for visual feedback
        transform.Translate(Vector3.forward * moveX);
        transform.Rotate(Vector3.up, rotateY);
    }
}
```

### Step 5: Set Up Unity Perception for Data Generation

Unity Perception allows you to generate synthetic training data with ground truth annotations:

```csharp
using UnityEngine;
using Unity.Perception.GroundTruth;
using Unity.Perception.Randomization;

public class PerceptionSetup : MonoBehaviour
{
    void Start()
    {
        // Enable ground truth data generation
        var labeler = FindObjectOfType<Labeler>();
        if (labeler != null)
        {
            labeler.enabled = true;
            Debug.Log("Perception labeler enabled");
        }

        // Add semantic segmentation labels to objects
        AddSemanticLabels();
    }

    void AddSemanticLabels()
    {
        // Example: Add semantic labels to robot parts
        GameObject[] robotParts = GameObject.FindGameObjectsWithTag("RobotPart");
        foreach (GameObject part in robotParts)
        {
            var semanticSegmentation = part.AddComponent<SemanticSegmentationLabel>();
            semanticSegmentation.labelId = 1; // Robot class
            semanticSegmentation.labelName = "Robot";
        }

        // Add labels to environment objects
        GameObject[] obstacles = GameObject.FindGameObjectsWithTag("Obstacle");
        foreach (GameObject obstacle in obstacles)
        {
            var semanticSegmentation = obstacle.AddComponent<SemanticSegmentationLabel>();
            semanticSegmentation.labelId = 2; // Obstacle class
            semanticSegmentation.labelName = "Obstacle";
        }
    }
}
```

## Code

### Complete Unity Robot Integration Example:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;

public class UnityRobotManager : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "unity_robot";
    public float maxLinearVelocity = 2.0f;
    public float maxAngularVelocity = 1.5f;

    [Header("ROS Communication")]
    public string rosBridgeUrl = "ws://localhost:9090";
    public string cmdVelTopic = "/cmd_vel";
    public string jointStatesTopic = "/joint_states";
    public string laserScanTopic = "/scan";

    private RosConnector rosConnector;
    private UnityRobotController robotController;

    // ROS message publishers
    private Publisher<Messages.Geometry.Twist> cmdVelPublisher;
    private Publisher<Messages.Sensor.JointState> jointStatePublisher;
    private Publisher<Messages.Sensor.LaserScan> laserScanPublisher;

    // ROS message subscribers
    private Subscriber<Messages.Nav.Odometry> odomSubscriber;
    private Subscriber<Messages.Geometry.Twist> cmdVelSubscriber;

    void Start()
    {
        InitializeRobot();
        ConnectToRos();
        InitializeRosCommunication();
    }

    void InitializeRobot()
    {
        robotController = GetComponent<UnityRobotController>();
        if (robotController == null)
        {
            robotController = gameObject.AddComponent<UnityRobotController>();
        }

        Debug.Log($"Unity robot '{robotName}' initialized");
    }

    void ConnectToRos()
    {
        GameObject rosConnectorObj = new GameObject("ROS Connector");
        rosConnector = rosConnectorObj.AddComponent<RosConnector>();
        rosConnector.RosBridgeWebSocketUrl = rosBridgeUrl;
    }

    void InitializeRosCommunication()
    {
        if (rosConnector.IsConnected())
        {
            // Publishers
            cmdVelPublisher = rosConnector.GetRosSocket().Advertise<Messages.Geometry.Twist>($"{robotName}{cmdVelTopic}");
            jointStatePublisher = rosConnector.GetRosSocket().Advertise<Messages.Sensor.JointState>($"{robotName}{jointStatesTopic}");
            laserScanPublisher = rosConnector.GetRosSocket().Advertise<Messages.Sensor.LaserScan>($"{robotName}{laserScanTopic}");

            // Subscribers
            odomSubscriber = rosConnector.GetRosSocket().Subscribe<Messages.Nav.Odometry>($"{robotName}/odom", OnOdometryReceived);
            cmdVelSubscriber = rosConnector.GetRosSocket().Subscribe<Messages.Geometry.Twist>($"{robotName}{cmdVelTopic}", OnVelocityCommandReceived);

            Debug.Log("ROS communication fully initialized for Unity robot");
        }
        else
        {
            Debug.LogError("Failed to initialize ROS communication - not connected to ROS bridge");
        }
    }

    void OnOdometryReceived(Messages.Nav.Odometry odomMsg)
    {
        // Update robot position based on odometry
        Vector3 position = new Vector3((float)odomMsg.pose.pose.position.x,
                                      (float)odomMsg.pose.pose.position.y,
                                      (float)odomMsg.pose.pose.position.z);
        Quaternion rotation = new Quaternion((float)odomMsg.pose.pose.orientation.x,
                                           (float)odomMsg.pose.pose.orientation.y,
                                           (float)odomMsg.pose.pose.orientation.z,
                                           (float)odomMsg.pose.pose.orientation.w);

        transform.position = position;
        transform.rotation = rotation;
    }

    void OnVelocityCommandReceived(Messages.Geometry.Twist twistMsg)
    {
        // Apply velocity command received from ROS
        float linear = (float)twistMsg.linear.x;
        float angular = (float)twistMsg.angular.z;

        // Clamp velocities to prevent excessive movement
        linear = Mathf.Clamp(linear, -maxLinearVelocity, maxLinearVelocity);
        angular = Mathf.Clamp(angular, -maxAngularVelocity, maxAngularVelocity);

        // Apply movement
        transform.Translate(Vector3.forward * linear * Time.deltaTime);
        transform.Rotate(Vector3.up, angular * Time.deltaTime);
    }

    void Update()
    {
        // Publish joint states periodically
        if (jointStatePublisher != null && Time.time % 0.1f < Time.deltaTime) // Every 100ms
        {
            PublishJointStates();
        }

        // Publish laser scan data if available
        if (laserScanPublisher != null)
        {
            PublishLaserScan();
        }
    }

    void PublishJointStates()
    {
        var jointStateMsg = new Messages.Sensor.JointState();
        jointStateMsg.header = new Messages.Standard.Header { stamp = new Messages.Standard.Time() };
        jointStateMsg.name = new[] { "wheel_joint" };
        jointStateMsg.position = new[] { transform.rotation.eulerAngles.y };
        jointStateMsg.velocity = new[] { 0.0 };
        jointStateMsg.effort = new[] { 0.0 };

        jointStatePublisher.Publish(jointStateMsg);
    }

    void PublishLaserScan()
    {
        // Simulate laser scan data
        var laserScanMsg = new Messages.Sensor.LaserScan();
        laserScanMsg.header = new Messages.Standard.Header { stamp = new Messages.Standard.Time() };
        laserScanMsg.angle_min = -Mathf.PI / 2;
        laserScanMsg.angle_max = Mathf.PI / 2;
        laserScanMsg.angle_increment = Mathf.PI / 180; // 1 degree
        laserScanMsg.time_increment = 0.0;
        laserScanMsg.scan_time = 0.0;
        laserScanMsg.range_min = 0.1f;
        laserScanMsg.range_max = 10.0f;

        // Simulate some range data (in a real implementation, this would come from raycasting)
        int numReadings = 181; // 181 readings for 180 degrees at 1 degree increments
        laserScanMsg.ranges = new float[numReadings];
        for (int i = 0; i < numReadings; i++)
        {
            laserScanMsg.ranges[i] = Random.Range(0.5f, 5.0f); // Simulated distances
        }

        laserScanPublisher.Publish(laserScanMsg);
    }
}
```

## Examples

### Unity ROS Bridge Setup:

```bash
# On the ROS side, start the ROS bridge
# Install rosbridge_suite
sudo apt install ros-humble-rosbridge-suite

# Launch the WebSocket bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Unity Perception Pipeline:

```csharp
// Example of setting up a perception pipeline
using UnityEngine;
using Unity.Perception.GroundTruth;
using Unity.Simulation;

public class PerceptionPipeline : MonoBehaviour
{
    void Start()
    {
        // Configure the perception camera
        var perceptionCamera = GetComponent<PerceptionCamera>();
        perceptionCamera.captureRgbImages = true;
        perceptionCamera.captureSegmentationLabels = true;
        perceptionCamera.captureDepth = true;

        // Set up annotation settings
        var annotationManager = FindObjectOfType<AnnotationManager>();
        annotationManager.enabled = true;
    }
}
```

### URDF Import Workflow:

1. Import the URDF Importer package in Unity
2. Prepare your URDF files with proper mesh references
3. Use the URDF Importer window to import your robot
4. The importer will create a Unity GameObject hierarchy matching your URDF structure

## Best Practices

1. **Performance Optimization**: Use appropriate Level of Detail (LOD) systems for complex robots
2. **Physics Tuning**: Configure Unity physics to match real-world behavior
3. **Communication Reliability**: Implement proper error handling and reconnection logic
4. **Scene Organization**: Keep Unity scenes well-organized for complex robotics projects
5. **Asset Management**: Use Unity's addressable asset system for large robotics environments
6. **Testing**: Regularly validate Unity simulation against physical robot behavior
7. **Documentation**: Maintain clear documentation for Unity-ROS integration
8. **Version Control**: Use appropriate version control strategies for Unity projects

## Required Tools & Software

- **Unity Hub**: Latest version for project management
- **Unity Editor**: 2021.3 LTS or newer with URP/HDRP
- **ROS 2**: Humble Hawksbill or equivalent
- **ROS# (ROS Sharp)**: For Unity-ROS communication
- **Unity Perception Package**: For synthetic data generation
- **URDF Importer**: For importing ROS robot models
- **System Requirements**: Modern GPU with DirectX 11+ support, 16GB+ RAM recommended

## Expected Outcome

After completing this lesson, you should:
- Understand how to set up Unity for robotics applications
- Be able to create Unity projects that communicate with ROS 2
- Know how to import and control robot models in Unity
- Have experience with Unity Perception for data generation
- Be prepared to use Unity for complex humanoid robotics simulation
- Understand when to choose Unity vs Gazebo for different robotics applications
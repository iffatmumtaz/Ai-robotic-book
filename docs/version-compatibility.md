---
sidebar_position: 6
title: "Version Compatibility Guidelines"
description: "Version compatibility guidelines for ROS 2, Gazebo, Isaac Sim, and other tools used in the Physical AI & Humanoid Robotics Book"
---

# Version Compatibility Guidelines

This document provides comprehensive version compatibility guidelines for all tools, frameworks, and platforms used throughout the Physical AI & Humanoid Robotics Book, ensuring stable and reproducible development environments.

## Overview

The Physical AI & Humanoid Robotics Book uses several complex software frameworks that have specific compatibility requirements. This guide ensures that readers can set up and maintain working environments with compatible versions.

## Core Frameworks Version Matrix

### ROS 2 Ecosystem

| Component | Recommended Version | Minimum Version | Maximum Version | Notes |
|-----------|-------------------|----------------|----------------|-------|
| ROS 2 Distribution | Humble Hawksbill (22.04) | Humble | Iron | LTS support until May 2027 |
| Python | 3.10.x | 3.8 | 3.11 | ROS 2 Humble officially supports 3.8-3.10 |
| rclpy | 4.2.0+ | 4.0.0 | 5.x | Python client library |
| rclcpp | 22.0.0+ | 20.0.0 | 24.x | C++ client library |
| Navigation2 | 1.2.0+ | 1.0.0 | 2.x | Navigation stack for mobile robots |
| MoveIt2 | 2.7.0+ | 2.5.0 | 3.x | Motion planning framework |

### Simulation Platforms

| Component | Recommended Version | Minimum Version | Maximum Version | Notes |
|-----------|-------------------|----------------|----------------|-------|
| Gazebo Garden | 7.x | 6.x | 8.x | Recommended for ROS 2 Humble |
| Gazebo Harmonic | 8.x | 7.x | Latest | Future-proof option |
| Isaac Sim | 2023.1.1+ | 2022.2.0 | Latest LTS | NVIDIA's robotics simulator |
| Unity | 2022.3 LTS | 2021.3 | 2023.x | Recommended for robotics projects |

### Development Tools

| Component | Recommended Version | Minimum Version | Maximum Version | Notes |
|-----------|-------------------|----------------|----------------|-------|
| Docker | 20.10+ | 19.03 | Latest | Container runtime |
| NVIDIA Container Toolkit | 1.11+ | 1.0 | Latest | GPU support in containers |
| CUDA | 11.8+ | 11.4 | 12.x | Required for Isaac Sim |
| cuDNN | 8.6+ | 8.0 | 9.x | Deep learning primitives |
| OpenCV | 4.6+ | 4.5 | 5.x | Computer vision library |

## Hardware-Specific Requirements

### Jetson Platforms

| Component | Jetson Orin AGX | Jetson Orin NX | Jetson Orin Nano | Notes |
|-----------|----------------|----------------|------------------|-------|
| JetPack | 5.1.3+ | 5.1.3+ | 5.1.3+ | Includes L4T, CUDA, OpenCV |
| CUDA | 11.4+ | 11.4+ | 11.4+ | Match JetPack version |
| OpenCV | 4.5.4+ | 4.5.4+ | 4.5.4+ | Pre-installed with JetPack |
| Python | 3.8.10 | 3.8.10 | 3.8.10 | Fixed with JetPack |

### Robot Platforms

#### Unitree Robots
- **Go2**: SDK 1.0+, Firmware 1.0.0+
- **G1**: SDK 1.0+, Firmware 1.0.0+
- **Network**: ROS 2 Humble compatible drivers

#### RealSense Cameras
- **SDK**: 2.53+ (latest recommended)
- **Firmware**: Latest stable for each camera model
- **ROS 2 Package**: realsense2_camera 4.0.x

## Compatibility Verification

### ROS 2 Humble Compatibility Check

```bash
# Verify ROS 2 Humble installation
echo $ROS_DISTRO
# Should output: humble

# Check ROS 2 version
ros2 --version
# Should output: ros2 foxy, galactic, rolling, or humble (depending on installation)

# Verify Python compatibility
python3 --version
# Should be 3.8, 3.9, or 3.10 for Humble

# Check installed packages
dpkg -l | grep ros-humble
```

### Gazebo Compatibility Check

```bash
# Check Gazebo version
gz --version
# For Garden: Should show version 7.x

# Alternative (for older Gazebo)
gazebo --version
# Check compatibility with ROS 2

# Verify ROS 2 Gazebo packages
dpkg -l | grep ros-humble-gazebo
```

### Isaac Sim Compatibility Check

```bash
# Check Isaac Sim installation
ls -la /isaac-sim/
# Should show Isaac Sim installation directory

# Verify CUDA compatibility
nvidia-smi
nvcc --version
# CUDA version should match Isaac Sim requirements

# Test Isaac Sim launch
./python.sh -c "import omni; print('Isaac Sim Python OK')"
```

## Version Management Strategies

### 1. Docker-Based Environments

Use Docker containers to ensure consistent environments:

```dockerfile
# Dockerfile for ROS 2 Humble + Gazebo Garden
FROM osrf/ros:humble-desktop-full

# Install Gazebo Garden
RUN apt-get update && apt-get install -y \
    gz-garden \
    ros-humble-ign-ros2-control \
    ros-humble-ign-transport \
    && rm -rf /var/lib/apt/lists/*

# Set environment
ENV ROS_DISTRO=humble
ENV GZ_VERSION=garden

# Verify installation
RUN ros2 --version
RUN gz --version
```

### 2. Virtual Environment Management

```bash
# Create Python virtual environment with specific version
python3.10 -m venv ~/ros2_env
source ~/ros2_env/bin/activate

# Install specific versions of packages
pip install rclpy==4.2.1
pip install opencv-python==4.6.0.66
pip install openai==0.27.0  # For LLM integration

# Freeze requirements for reproducibility
pip freeze > requirements-humble.txt
```

### 3. Package Lock Files

Create lock files with exact versions:

```txt
# requirements-humble.lock
rclpy==4.2.1
ros2pkg==0.3.19
opencv-python==4.6.0.66
numpy==1.21.6
transforms3d==0.4.1
openai==0.27.0
torch==1.13.1+cu117
torchaudio==0.13.1+cu117
torchvision==0.14.1+cu117
```

## Migration Guidelines

### From Earlier ROS 2 Versions

#### Rolling → Humble Migration
```bash
# 1. Backup current workspace
cp -r ~/ros2_ws ~/ros2_ws_backup

# 2. Update sources.list to point to humble
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# 3. Update and install humble
sudo apt update
sudo apt install ros-humble-desktop

# 4. Update workspace
cd ~/ros2_ws
rm -rf build install log
colcon build
```

#### Foxy → Humble Migration
- API changes in rclpy and rclcpp
- Updated message definitions
- Changed parameter handling
- New QoS profiles

### Simulation Platform Migration

#### Gazebo Classic → Garden Migration
```bash
# Update package.xml
# Change ign-* dependencies to gz-* dependencies
# Update launch files to use gz-* executables
# Update code to use gz::* namespaces instead of ignition::*
```

## Troubleshooting Version Conflicts

### Common Version Issues

#### Problem: Package Version Conflicts
**Symptoms**: Dependency resolution failures, version mismatch errors
**Solutions**:
```bash
# Clean package cache
sudo apt clean
sudo apt autoclean

# Remove conflicting packages
sudo apt remove ros-*

# Reinstall with correct versions
sudo apt update
sudo apt install ros-humble-desktop
```

#### Problem: Python Package Conflicts
**Symptoms**: Import errors, missing modules, version mismatches
**Solutions**:
```bash
# Use virtual environments
python3 -m venv ~/ros2_env
source ~/ros2_env/bin/activate

# Install specific versions
pip install --force-reinstall package_name==specific.version

# Check installed versions
pip list | grep package_name
```

### Verification Scripts

#### Version Compatibility Script
```bash
#!/bin/bash
# version_check.sh - Verify all components are compatible

echo "=== ROS 2 Environment Check ==="
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"

if [ "$ROS_DISTRO" != "humble" ]; then
    echo "ERROR: Expected ROS_DISTRO=humble, got $ROS_DISTRO"
    exit 1
fi

echo "=== Python Version Check ==="
PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "Python: $PYTHON_VERSION"

if [[ $PYTHON_VERSION =~ ^3\.[8-9]|^3\.1[0-1] ]]; then
    echo "Python version is compatible"
else
    echo "ERROR: Python version $PYTHON_VERSION is not compatible with ROS 2 Humble"
    exit 1
fi

echo "=== Gazebo Check ==="
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz --version)
    echo "Gazebo: $GZ_VERSION"
else
    echo "WARNING: Gazebo not found"
fi

echo "=== CUDA Check ==="
if command -v nvidia-smi &> /dev/null; then
    CUDA_VERSION=$(nvcc --version | grep "release" | cut -d' ' -f6 | cut -c2-)
    echo "CUDA: $CUDA_VERSION"

    if [[ $CUDA_VERSION =~ ^11\.[4-9]|^11\.1[0-9]|^12\.[0-9] ]]; then
        echo "CUDA version is compatible"
    else
        echo "WARNING: CUDA version $CUDA_VERSION may not be compatible with Isaac Sim"
    fi
else
    echo "WARNING: CUDA not found"
fi

echo "=== All checks completed ==="
```

## Best Practices

### 1. Version Pinning
- Pin specific versions in all configuration files
- Use lock files for reproducible builds
- Document version requirements in setup guides

### 2. Regular Updates
- Schedule regular environment updates
- Test new versions in isolated environments first
- Maintain compatibility matrices

### 3. Documentation
- Keep version requirements updated in documentation
- Document breaking changes during migrations
- Provide clear upgrade paths

### 4. Testing
- Test compatibility before releasing updates
- Use CI/CD to verify version compatibility
- Maintain test environments with different configurations

## Support Windows

### ROS 2 Humble Support Timeline
- Release Date: May 2022
- End of Life: May 2027 (5-year support cycle)
- Active Development: Until May 2026
- Security Updates: Until May 2027

### Platform Support Status
- **Supported**: ROS 2 Humble, Gazebo Garden, Isaac Sim 2023.x
- **Legacy**: ROS 2 Foxy, Gazebo Classic, Isaac Sim 2022.x
- **Experimental**: ROS 2 Iron, Gazebo Harmonic

## Emergency Procedures

### Downgrade Process
If newer versions cause issues:

1. **Backup Current State**:
   ```bash
   tar -czf ~/workspace-backup.tar.gz ~/ros2_ws
   pip freeze > ~/current-packages.txt
   ```

2. **Downgrade ROS 2**:
   ```bash
   sudo apt remove ros-*
   sudo apt install ros-humble-desktop=xxxxx  # specific version
   ```

3. **Restore Packages**:
   ```bash
   pip install -r ~/current-packages.txt
   ```

This version compatibility guide ensures that all components used in the Physical AI & Humanoid Robotics Book work together harmoniously, providing a stable foundation for learning and development.
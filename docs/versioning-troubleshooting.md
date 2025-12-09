# Versioning Guidelines and Troubleshooting for Software Compatibility

## Purpose
This document provides comprehensive versioning guidelines and troubleshooting steps to help readers resolve software compatibility issues when following the Physical AI & Humanoid Robotics Book, ensuring consistent and reproducible results across different environments.

## Versioning Guidelines

### 1. Semantic Versioning Approach
The book follows semantic versioning principles for all software components:

- **Major Version (X.0.0)**: Breaking changes, incompatible API changes
- **Minor Version (0.Y.0)**: New features, backward-compatible
- **Patch Version (0.0.Z)**: Bug fixes, backward-compatible

### 2. Software Stack Version Matrix

| Component | Recommended Version | Minimum Version | Maximum Version | Notes |
|-----------|-------------------|----------------|----------------|-------|
| Ubuntu | 22.04 LTS | 20.04 LTS | 24.04 LTS | Long-term support recommended |
| ROS 2 | Humble Hawksbill | Humble | Iron | LTS support until May 2027 |
| Python | 3.10.x | 3.8 | 3.11 | ROS 2 Humble official support |
| Gazebo | Garden | Fortress | Harmonic | Recommended for ROS 2 Humble |
| NVIDIA Isaac Sim | 2023.1.1+ | 2022.2.0 | Latest LTS | Check CUDA compatibility |
| Docker | 20.10+ | 19.03 | Latest | Required for containerized workflows |
| Node.js | 18.x LTS | 16.x | 20.x LTS | For Docusaurus documentation |
| CUDA | 11.8+ | 11.4 | 12.0 | For NVIDIA GPU acceleration |

### 3. Version Pinning Strategy

#### A. Python Dependencies
Create `requirements.txt` with specific versions:
```txt
rclpy==3.2.1
ros2cli==0.18.13
setuptools==65.5.0
colcon-common-extensions==0.2.1
```

#### B. ROS 2 Package Dependencies
In `package.xml`:
```xml
<depend>rosidl_default_generators</depend>
<depend>builtin_interfaces</depend>
<exec_depend version_gte="0.8.0">rclpy</exec_depend>
```

#### C. Docker Images
Use specific tags instead of `latest`:
```dockerfile
FROM osrf/ros:humble-desktop-full@sha256:abc123...
```

## Troubleshooting Framework

### 1. Issue Classification System
- **Critical**: Prevents any progress (e.g., ROS 2 installation failure)
- **High**: Blocks major functionality (e.g., simulation won't start)
- **Medium**: Affects specific features (e.g., specific sensor driver)
- **Low**: Minor issues (e.g., documentation typos)

### 2. Troubleshooting Process
```
Problem Identification:
├── Error Message Analysis
├── Environment Verification
├── Version Compatibility Check
└── Dependency Resolution

Solution Application:
├── Quick Fixes (common solutions)
├── Detailed Troubleshooting (step-by-step)
├── Alternative Approaches
└── Workaround Implementation

Verification:
├── Solution Effectiveness
├── Side Effect Assessment
└── Documentation Update
```

## Common Compatibility Issues and Solutions

### 1. ROS 2 Installation Issues

#### Issue: Package Dependencies Conflict
**Symptoms**:
```
The following packages have unmet dependencies:
ros-humble-desktop : Depends: ros-humble-some-package but it is not going to be installed
```

**Solution**:
```bash
# Clean package cache
sudo apt clean
sudo apt autoremove

# Update package lists
sudo apt update

# Fix broken dependencies
sudo apt --fix-broken install

# Try ROS 2 installation again
sudo apt install ros-humble-desktop
```

#### Issue: Repository Key Issues
**Symptoms**:
```
The following signatures couldn't be verified because the public key is not available
```

**Solution**:
```bash
# Add ROS 2 repository key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Python Environment Issues

#### Issue: Python Version Mismatch
**Symptoms**:
```
ERROR: Package 'some-package' requires Python '>=3.10', but you have '3.8.10'
```

**Solution**:
```bash
# Check current Python version
python3 --version

# Install correct Python version if needed
sudo apt install python3.10 python3.10-venv python3.10-dev

# Create virtual environment with correct Python version
python3.10 -m venv ~/ros2_env
source ~/ros2_env/bin/activate

# Install packages in virtual environment
pip install --upgrade pip
pip install rclpy ros2cli
```

#### Issue: Package Conflicts
**Symptoms**:
```
ERROR: pip's dependency resolver does not currently take into account all the packages that are installed.
```

**Solution**:
```bash
# Create clean virtual environment
python3 -m venv ~/clean_env
source ~/clean_env/bin/activate

# Install packages in correct order
pip install --upgrade pip setuptools
pip install rclpy  # Install ROS 2 packages first
pip install other-dependencies  # Then other dependencies
```

### 3. Gazebo Simulation Issues

#### Issue: GPU Driver Compatibility
**Symptoms**:
```
Error: Couldn't create OpenGL context
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
```

**Solution**:
```bash
# Check GPU and driver
nvidia-smi
glxinfo | grep "OpenGL renderer"

# Install appropriate drivers
sudo apt install nvidia-driver-535  # Or appropriate version for your GPU

# Reboot system
sudo reboot

# If still having issues, try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
```

#### Issue: Gazebo Version Conflicts
**Symptoms**:
```
Error [App.cc:152] Console bridge flags have changed, please update your code
```

**Solution**:
```bash
# Check Gazebo version
gz --version

# Uninstall conflicting versions
sudo apt remove gz-*

# Install correct version for ROS 2 Humble
sudo apt install ros-humble-gazebo-*
```

### 4. NVIDIA Isaac Sim Issues

#### Issue: CUDA Version Mismatch
**Symptoms**:
```
CUDA driver version is insufficient for CUDA runtime version
```

**Solution**:
```bash
# Check CUDA versions
nvidia-smi  # Driver version
nvcc --version  # Runtime version

# Install matching versions
# For Isaac Sim 2023.1.1, ensure CUDA 11.8+
sudo apt install nvidia-driver-535
sudo apt install cuda-toolkit-11-8
```

#### Issue: Docker Container Issues
**Symptoms**:
```
docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]].
```

**Solution**:
```bash
# Install NVIDIA Container Toolkit
sudo apt install nvidia-container-toolkit

# Configure Docker
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker
sudo systemctl restart docker

# Test GPU access
docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu20.04 nvidia-smi
```

## Version Management Best Practices

### 1. Container-Based Development
```dockerfile
# Dockerfile with pinned versions
FROM ubuntu:22.04

# Install specific ROS 2 version
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop=0.18.1-1* \
    python3-rosdep2=0.21.0-1* \
    python3-rosinstall=0.11.1-1* \
    python3-vcstool=0.3.1-1* \
    && rm -rf /var/lib/apt/lists/*
```

### 2. Virtual Environment Management
```bash
#!/bin/bash
# version-controlled environment setup

# Create environment with specific Python version
python3.10 -m venv ros2_env
source ros2_env/bin/activate

# Install specific versions of packages
pip install -r requirements.lock.txt

# Source ROS 2 with specific version
source /opt/ros/humble/setup.bash

# Verify versions
python3 --version
ros2 --version
```

### 3. Package Lock Files
Create `requirements.lock.txt` with pinned versions:
```txt
# Generated on 2023-12-08
# For ROS 2 Humble with Ubuntu 22.04
rclpy==3.2.1
ros2cli==0.18.13
setuptools==65.5.0
colcon-common-extensions==0.2.1
launch==1.0.5
launch_ros==0.24.1
```

## Diagnostic Tools and Commands

### 1. Environment Verification Script
```bash
#!/bin/bash
# Environment verification script

echo "=== System Information ==="
echo "OS: $(lsb_release -d | cut -f2)"
echo "Kernel: $(uname -r)"
echo "Architecture: $(uname -m)"

echo -e "\n=== ROS 2 Information ==="
if command -v ros2 &> /dev/null; then
    echo "ROS 2: $(ros2 --version)"
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "ROS_ROOT: $ROS_ROOT"
else
    echo "ROS 2: NOT INSTALLED"
fi

echo -e "\n=== Python Information ==="
echo "Python 3: $(python3 --version)"
echo "Pip: $(pip --version)"
if [ -n "$VIRTUAL_ENV" ]; then
    echo "Virtual Environment: $VIRTUAL_ENV"
else
    echo "Virtual Environment: NOT ACTIVATED"
fi

echo -e "\n=== GPU Information ==="
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,driver_version,cuda_version --format=csv
else
    echo "NVIDIA GPU: NOT DETECTED"
fi

echo -e "\n=== Docker Information ==="
if command -v docker &> /dev/null; then
    echo "Docker: $(docker --version)"
    docker version --format "Client: {{.Client.Version}}, Server: {{.Server.Version}}"
else
    echo "Docker: NOT INSTALLED"
fi
```

### 2. Dependency Checker
```python
#!/usr/bin/env python3
# Dependency checker script

import sys
import subprocess
import importlib

REQUIRED_PACKAGES = [
    ('rclpy', '3.0.0'),
    ('ros2cli', '0.18.0'),
    ('setuptools', '60.0.0'),
]

def check_python_package(package_name, min_version):
    try:
        module = importlib.import_module(package_name)
        if hasattr(module, '__version__'):
            version = module.__version__
            print(f"✓ {package_name}: {version}")
            return True
        else:
            print(f"⚠ {package_name}: Version unknown")
            return True
    except ImportError:
        print(f"✗ {package_name}: NOT INSTALLED")
        return False

def check_system_command(command):
    result = subprocess.run(['which', command], capture_output=True, text=True)
    if result.returncode == 0:
        print(f"✓ {command}: available at {result.stdout.strip()}")
        return True
    else:
        print(f"✗ {command}: NOT FOUND")
        return False

def main():
    print("=== Python Package Check ===")
    for package, min_version in REQUIRED_PACKAGES:
        check_python_package(package, min_version)

    print("\n=== System Command Check ===")
    commands_to_check = ['ros2', 'colcon', 'python3', 'docker', 'gz']
    for cmd in commands_to_check:
        check_system_command(cmd)

if __name__ == '__main__':
    main()
```

## Recovery and Rollback Procedures

### 1. Complete Environment Reset
```bash
#!/bin/bash
# Complete environment reset script

echo "WARNING: This will remove all ROS 2 installations and configurations."
read -p "Are you sure? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    # Remove ROS 2 packages
    sudo apt remove ros-*
    sudo apt autoremove

    # Remove ROS 2 repositories
    sudo rm /etc/apt/sources.list.d/ros2.list

    # Remove ROS 2 environment variables
    sed -i '/ROS_/d' ~/.bashrc
    sed -i '/setup.bash/d' ~/.bashrc

    # Remove workspace
    rm -rf ~/ros2_ws

    echo "Environment reset complete. Please reinstall ROS 2."
else
    echo "Reset cancelled."
fi
```

### 2. Workspace Recovery
```bash
#!/bin/bash
# Workspace recovery script

WORKSPACE_DIR="${1:-~/ros2_ws}"

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Workspace does not exist: $WORKSPACE_DIR"
    exit 1
fi

cd $WORKSPACE_DIR

# Clean build artifacts
rm -rf build/ install/ log/

# Rebuild with specific settings
colcon build --symlink-install --packages-up-to $(find src -name "package.xml" -exec dirname {} \; | xargs basename -a | tr '\n' ' ')

# Source the workspace
source install/setup.bash

echo "Workspace rebuilt successfully."
```

## Version Migration Guide

### 1. ROS 2 Humble to Iron Migration
```bash
# Before migration
source /opt/ros/humble/setup.bash
ros2 pkg list > humble_packages.txt

# Migration process
sudo apt remove ros-humble-*
sudo apt install ros-iron-desktop

# Update workspace
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build

# Verify migration
source /opt/ros/iron/setup.bash
echo $ROS_DISTRO  # Should output 'iron'
```

### 2. Ubuntu 20.04 to 22.04 Migration
- Perform Ubuntu distribution upgrade
- Reinstall ROS 2 for new Ubuntu version
- Update all configuration files
- Rebuild all workspaces

## Automated Troubleshooting

### 1. Self-Healing Scripts
```bash
#!/bin/bash
# Automated troubleshooting script

fix_ros2_setup() {
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        echo "ROS 2 Humble not found, attempting to install..."
        sudo apt update
        sudo apt install ros-humble-desktop
    fi

    # Source ROS 2
    source /opt/ros/humble/setup.bash
}

fix_workspace() {
    if [ -d "~/ros2_ws" ]; then
        cd ~/ros2_ws
        if [ ! -d "install" ]; then
            colcon build
        fi
        source install/setup.bash
    fi
}

# Run fixes
fix_ros2_setup
fix_workspace

echo "Automated fixes applied."
```

### 2. Configuration Validator
```python
#!/usr/bin/env python3
# Configuration validator

import os
import subprocess
from pathlib import Path

def validate_ros2_environment():
    """Validate ROS 2 environment configuration"""
    issues = []

    # Check if ROS 2 is installed
    if not Path('/opt/ros/humble/setup.bash').exists():
        issues.append("ROS 2 Humble not installed")

    # Check if environment is sourced
    if 'ROS_DISTRO' not in os.environ:
        issues.append("ROS environment not sourced")
    elif os.environ['ROS_DISTRO'] != 'humble':
        issues.append(f"Wrong ROS distribution: {os.environ['ROS_DISTRO']}")

    # Check workspace
    workspace = Path.home() / 'ros2_ws'
    if not workspace.exists():
        issues.append("ROS 2 workspace not found")
    elif not (workspace / 'install' / 'setup.bash').exists():
        issues.append("Workspace not built")

    return issues

def main():
    issues = validate_ros2_environment()

    if issues:
        print("Issues found in ROS 2 environment:")
        for issue in issues:
            print(f"  - {issue}")
        return 1
    else:
        print("ROS 2 environment is properly configured.")
        return 0

if __name__ == '__main__':
    exit(main())
```

## Support Resources

### 1. Official Documentation Links
- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Gazebo Documentation: http://gazebosim.org/
- Isaac Sim Documentation: https://docs.omniverse.nvidia.com/isaac-sim/latest/
- Ubuntu ROS Installation: https://wiki.ros.org/humble/Installation/Ubuntu-Install-Debians

### 2. Community Support Channels
- ROS Answers: https://answers.ros.org/
- ROS Discourse: https://discourse.ros.org/
- Gazebo Community: https://community.gazebosim.org/
- NVIDIA Developer Forums: https://forums.developer.nvidia.com/

## Validation Checklist

Before finalizing versioning and troubleshooting guidance, verify:

- [ ] All recommended versions are currently available and stable
- [ ] Common compatibility issues are addressed
- [ ] Troubleshooting steps are clear and actionable
- [ ] Recovery procedures are documented
- [ ] Diagnostic tools are provided
- [ ] Version migration paths are clear
- [ ] Automated solutions are available where appropriate
- [ ] Support resources are referenced
- [ ] Environment verification procedures are included
- [ ] Rollback procedures are documented
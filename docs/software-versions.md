# Software Versions and Compatibility Guide

## Purpose
This document provides recommended software versions and compatibility information for the Physical AI & Humanoid Robotics Book, ensuring learners can successfully follow along with consistent and tested configurations.

## Recommended Software Stack

### 1. Operating System
- **Ubuntu Linux 22.04 LTS (Jammy Jellyfish)** - Primary development environment
  - Kernel: 5.15.x or higher
  - Architecture: x86_64 (for development), ARM64 (for Jetson deployment)
  - Minimum RAM: 16GB for simulation development
  - Recommended RAM: 32GB for intensive simulation work

### 2. ROS 2 Distribution
- **ROS 2 Humble Hawksbill** (Recommended)
  - Version: 2.7.x series
  - Release: May 2022
  - Support: Until May 2027 (long-term support)
  - Installation: Desktop or Desktop Full variant

- **Alternative: ROS 2 Iron Irwini**
  - Version: 2.10.x series
  - Release: May 2023
  - Support: Until May 2025
  - Use only if Humble is unavailable

### 3. Simulation Environments

#### Gazebo Garden
- **Version**: 7.x series
- **Release**: March 2023
- **Compatibility**: ROS 2 Humble
- **Installation**: From Ubuntu packages or source

#### NVIDIA Isaac Sim
- **Version**: 2023.1.1 (or latest LTS)
- **Requirements**:
  - NVIDIA GPU with CUDA 11.8+ support
  - RTX series recommended (2080 Ti, 3080, 4090, A5000, A6000)
  - Minimum VRAM: 11GB
  - Recommended VRAM: 24GB+
- **Container**: Docker-based installation recommended

#### Unity Robotics
- **Version**: 2022.3 LTS
- **Requirements**:
  - Unity Hub for version management
  - Unity Robotics Package 0.9.x
  - Unity Simulation Package 0.5.x

### 4. Programming Languages and Tools

#### Python
- **Version**: 3.10.x series (recommended)
- **Compatibility**: ROS 2 Humble officially supports 3.10
- **Package Manager**: pip 22.0+ or conda 23.x+

#### C++
- **Standard**: C++17 (minimum)
- **Compilers**: GCC 11.x or Clang 12.x+
- **Build System**: CMake 3.22+ (minimum)

#### JavaScript/TypeScript
- **Node.js**: 18.x LTS or 20.x LTS
- **npm**: 9.x+ (bundled with Node.js)
- **Docusaurus**: 3.0+ (latest stable)

### 5. Hardware-Specific Software

#### Jetson Orin
- **JetPack**: 5.1.3 or higher
- **CUDA**: 11.4+ (included in JetPack)
- **OpenCV**: 4.5.x (pre-installed in JetPack)
- **ROS 2**: Available via Debian packages

#### RealSense Cameras
- **Librealsense**: 2.53.x series
- **ROS 2 Package**: realsense2_camera 4.0.x
- **Firmware**: Latest stable for each camera model

### 6. Large Language Model Integration

#### OpenAI API
- **Library**: openai 1.0+ (Python)
- **API Version**: Latest stable
- **Alternative**: Local models via Ollama or similar

#### Alternative Local Models
- **Ollama**: 0.1.x+ (for local LLM serving)
- **Model Requirements**: 8GB+ VRAM for basic models, 24GB+ for larger models

## Version Compatibility Matrix

| Component | ROS 2 Humble | ROS 2 Iron | Notes |
|-----------|--------------|------------|-------|
| Ubuntu 22.04 | ✅ | ✅ | Recommended |
| Ubuntu 20.04 | ❌ | ❌ | EOL for ROS 2 support |
| Python 3.8 | ❌ | ❌ | End-of-life |
| Python 3.10 | ✅ | ✅ | Recommended |
| Python 3.11 | ✅ | ✅ | Supported |
| Gazebo Garden | ✅ | ✅ | Recommended |
| Gazebo Fortress | ✅ | ✅ | Alternative |
| NVIDIA Isaac Sim | ✅ | ✅ | Compatible |

## Installation Recommendations

### 1. Development Environment Setup
```bash
# ROS 2 Humble Installation
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-base
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
```

### 2. Simulation Software
```bash
# Gazebo Garden
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo

# NVIDIA Isaac Sim (Docker)
sudo apt install docker.io
sudo usermod -a -G docker $USER
# Download Isaac Sim from NVIDIA developer portal
```

### 3. Development Tools
```bash
# Python environment
sudo apt install python3.10-dev python3-pip python3-venv
python3 -m venv ~/ros2_env
source ~/ros2_env/bin/activate
pip install --upgrade pip
pip install colcon-common-extensions vcstool

# Docusaurus for book development
npm install -g @docusaurus/core @docusaurus/module-type-aliases @docusaurus/types
```

## Version Management Best Practices

### 1. Containerization
- Use Docker for consistent environments
- Pin specific image versions in Dockerfiles
- Use docker-compose for multi-service setups

### 2. Virtual Environments
- Use Python virtual environments for package isolation
- Pin package versions in requirements.txt
- Use conda environments for complex dependencies

### 3. ROS 2 Workspace Management
- Create separate workspaces for different projects
- Use wstool for source management
- Maintain separate branches for different ROS 2 versions if needed

## Troubleshooting Common Version Issues

### 1. Package Compatibility
- Check ROS 2 package compatibility before installation
- Use `rosdep check` to verify dependencies
- Consult ROS 2 package documentation for version requirements

### 2. GPU Driver Issues
- Ensure NVIDIA drivers are compatible with CUDA version
- Use `nvidia-smi` to check driver/CUDA compatibility
- Reinstall drivers if version conflicts occur

### 3. Library Conflicts
- Use virtual environments to avoid system-wide conflicts
- Check for conflicting Python packages
- Use `apt policy` to check package version priorities

## Version Update Strategy

### 1. Minor Updates
- Test in isolated environment first
- Verify compatibility with existing code
- Update documentation with new version-specific notes

### 2. Major Updates
- Plan migration path carefully
- Update all dependent components
- Thoroughly test all functionality
- Update all examples and code snippets

## Validation Checklist

Before recommending any software version, verify:

- [ ] Officially supported by the respective project
- [ ] Compatible with ROS 2 Humble (primary target)
- [ ] Tested with hardware requirements
- [ ] Long-term support or stable release
- [ ] Adequate documentation available
- [ ] Community support available
- [ ] Performance meets requirements
- [ ] Installation process is documented
- [ ] Dependencies are resolved
- [ ] Security updates are available
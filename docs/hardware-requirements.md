# Hardware Requirements for Physical AI & Humanoid Robotics

## Purpose
This document outlines the hardware requirements for the Physical AI & Humanoid Robotics Book, covering both simulation development and physical deployment scenarios.

## Development Hardware Requirements

### Minimum Specifications (Simulation Development)
- **CPU**: Intel i7-10700K or AMD Ryzen 7 3700X (8 cores, 16 threads)
- **RAM**: 32GB DDR4-3200MHz
- **GPU**: NVIDIA RTX 3070 (8GB VRAM) or equivalent
- **Storage**: 1TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS
- **Network**: Gigabit Ethernet, 802.11ac WiFi

### Recommended Specifications (Optimal Simulation Performance)
- **CPU**: Intel i9-12900K or AMD Ryzen 9 5900X (16+ cores, 32+ threads)
- **RAM**: 64GB DDR4-3600MHz or DDR5
- **GPU**: NVIDIA RTX 4090 (24GB VRAM) or RTX A6000 (48GB VRAM)
- **Storage**: 2TB+ NVMe SSD (fast read/write for simulation assets)
- **OS**: Ubuntu 22.04 LTS
- **Network**: Gigabit Ethernet (10GbE recommended for robotics networking)

## Simulation-Specific Hardware

### GPU Requirements for Simulation
| Simulation Platform | Minimum GPU | Recommended GPU | VRAM Required | Notes |
|-------------------|-------------|-----------------|---------------|-------|
| Gazebo Garden | GTX 1060 6GB | RTX 3080 12GB | 6GB+ | Physics simulation |
| NVIDIA Isaac Sim | GTX 1080 8GB | RTX 4090 24GB | 8GB+ | AI/ML integration |
| Unity Robotics | GTX 1070 8GB | RTX 3080 12GB | 8GB+ | Rendering quality |
| General Robotics | GTX 1060 6GB | RTX 3070 8GB | 6GB+ | Visualization |

### Specialized Hardware for Perception
- **RealSense D455**: RGB-D camera for depth perception
- **RealSense L515**: LiDAR camera for 360° scanning
- **StereoLabs ZED 3**: Stereo vision for depth perception
- **FLIR cameras**: For thermal imaging (optional)

## Physical Deployment Hardware

### Jetson Orin Development Kit
- **Model**: NVIDIA Jetson AGX Orin Developer Kit (64GB)
- **CPU**: 12-core ARM v8.4 64-bit CPU
- **GPU**: 2048-core NVIDIA Ampere architecture GPU
- **RAM**: 64GB LPDDR5 memory
- **Storage**: 32GB eMMC storage
- **Connectivity**:
  - 2x 2.5GbE Ethernet
  - Wi-Fi 6 (802.11ax)
  - Bluetooth 5.2
- **Power**: 15W to 60W operation modes
- **Operating System**: JetPack 5.1.3 (based on Ubuntu 22.04)

### Alternative Jetson Platforms
- **Jetson Orin NX**: 8GB/16GB RAM options (lower power)
- **Jetson Orin Nano**: 4GB/8GB RAM options (budget alternative)
- **Jetson AGX Xavier**: Legacy option (if Orin unavailable)

### Robot Platforms

#### Unitree Go2
- **Type**: Quadruped robot
- **Onboard Compute**: NVIDIA Jetson Orin NX
- **Sensors**:
  - Depth camera
  - IMU
  - Force/torque sensors
- **Connectivity**: Wi-Fi, Ethernet
- **Battery**: 2.5 hours operation
- **Payload**: 5kg

#### Unitree G1
- **Type**: Humanoid robot
- **Onboard Compute**: NVIDIA Jetson Orin series
- **Sensors**:
  - Multiple cameras
  - LiDAR
  - IMU
  - Force/torque sensors
- **Connectivity**: Wi-Fi, Ethernet
- **Battery**: 1-2 hours operation
- **Height**: 1m
- **Weight**: 32kg

#### Proxy Robot Alternatives
For learners without access to expensive robots:
- **TurtleBot 4**: ROS 2 compatible, affordable
- **Clearpath Jackal**: UGV platform for navigation
- **Intel RealSense Tracking Camera T265**: For pose estimation
- **Custom Arduino/Raspberry Pi platforms**: For basic robotics

## Network Requirements

### Development Network
- **Router**: Gigabit capable (1000BASE-T)
- **Switch**: Managed switch recommended for robotics networking
- **Access Point**: 802.11ac or 802.11ax for wireless robotics
- **Cables**: Cat6 or better for reliable connections

### Robot Communication
- **Latency**: &lt;10ms for real-time control
- **Bandwidth**: 100Mbps minimum, 1Gbps recommended
- **Reliability**: Wired connection preferred for critical operations
- **Security**: WPA3 or equivalent for wireless communication

## Peripheral Hardware

### Input Devices
- **Keyboard**: Mechanical keyboard for development
- **Mouse**: Precision mouse for 3D modeling
- **Graphics Tablet**: For drawing diagrams and annotations (optional)

### Output Devices
- **Monitor**: 27" 1440p or 4K for development
- **Audio**: Headphones/speakers for audio feedback
- **Printer**: For printing diagrams and documentation (optional)

### Storage Solutions
- **Backup Drive**: 2TB+ external drive for project backups
- **Network Storage**: NAS for shared robotics assets (optional)
- **Cloud Storage**: For version control and collaboration

## Budget Considerations

### Complete Development Setup (Recommended)
- **Workstation**: $3,000-$5,000
- **GPU**: $1,500-$2,500
- **Jetson Orin**: $1,500-$2,000
- **RealSense Cameras**: $300-$800
- **Total**: $6,300-$10,300

### Budget Development Setup (Minimum)
- **Workstation**: $1,500-$2,500
- **GPU**: $500-$1,000
- **Jetson Orin**: $1,500-$2,000
- **RealSense Cameras**: $150-$300
- **Total**: $3,650-$5,800

### Simulation-Only Setup
- **Workstation**: $2,000-$3,000
- **GPU**: $1,000-$1,500
- **RealSense Cameras**: $150-$300
- **Total**: $3,150-$4,800

## Hardware Compatibility Testing

### Pre-Purchase Verification
- Check ROS 2 package compatibility with hardware
- Verify driver availability for Ubuntu 22.04
- Confirm power requirements and cooling needs
- Validate network interface compatibility

### Setup Validation Checklist
- [ ] All hardware components boot successfully
- [ ] GPU drivers properly installed and tested
- [ ] RealSense cameras detected and functional
- [ ] Network interfaces configured correctly
- [ ] Power requirements met with margin
- [ ] Cooling adequate for sustained operation
- [ ] All required ports available and accessible

## Alternative Hardware Options

### Cloud-Based Development
- **AWS RoboMaker**: Cloud robotics simulation
- **Azure IoT Hub**: Cloud robotics services
- **Google Cloud**: AI/ML services for robotics
- **Limitations**: Network latency for real-time control

### Container-Based Simulation
- **Docker**: Isolated simulation environments
- **NVIDIA Container Toolkit**: GPU acceleration in containers
- **Benefits**: Consistent environments, easier deployment
- **Requirements**: Compatible host hardware

## Hardware Procurement Guide

### Priority Order for Purchases
1. **Development workstation** with adequate CPU/RAM
2. **GPU** for simulation and AI workloads
3. **Jetson Orin** for physical deployment
4. **RealSense cameras** for perception
5. **Robot platform** (if budget allows)
6. **Network equipment** for reliable communication
7. **Peripherals** for enhanced development experience

### Vendor Recommendations
- **NVIDIA**: For Jetson platforms and GPUs
- **Intel**: For RealSense cameras
- **Unitree**: For humanoid/quaduped robots
- **Dell/HP**: For development workstations
- **Amazon/B&H**: For general hardware components

## Maintenance and Support

### Hardware Maintenance
- Regular cleaning of dust from GPU and CPU coolers
- Periodic driver updates for GPU and cameras
- Backup of critical system configurations
- Thermal monitoring during intensive simulations

### Support Resources
- **NVIDIA Developer Program**: For Jetson and GPU support
- **Intel RealSense Community**: For camera support
- **ROS Discourse**: For hardware compatibility questions
- **Unitree Support**: For robot platform issues

## Validation Checklist

Before finalizing hardware requirements, verify:

- [ ] All components are currently available for purchase
- [ ] Specifications meet minimum requirements for ROS 2 Humble
- [ ] Budget aligns with educational goals
- [ ] Alternative options provided for different budgets
- [ ] Power and cooling requirements are realistic
- [ ] Network requirements support robotics applications
- [ ] Hardware is compatible with Ubuntu 22.04 LTS
- [ ] Drivers are available and stable
- [ ] Performance requirements are validated
- [ ] Support options are available for each component
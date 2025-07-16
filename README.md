# LowLightSLAM

A research platform for evaluating visual SLAM performance in low-light environments, specifically designed for search and rescue robotics applications. This work integrates state-of-the-art low-light image enhancement techniques with visual SLAM systems to improve localization and mapping capabilities in challenging lighting conditions.

## Overview

This repository contains a fully integrated ROS workspace that combines:
- **ORB-SLAM3**: Visual SLAM system with tested and verified configuration
- **Zero-DCE Enhancement**: Real-time low-light image enhancement 
- **Gazebo Simulation**: Realistic robot simulation environment with TurtleBot3
- **ROS Integration**: Custom wrapper enabling seamless integration between components

## Research Context

This work is part of ongoing thesis research focused on developing robust visual SLAM systems for search and rescue operations. The primary objective is to enhance robot navigation capabilities in low-light environments commonly encountered in disaster scenarios, such as collapsed buildings, underground spaces, or nighttime operations.

### Current Status (July 16, 2025)
✅ **Completed:**
- ORB-SLAM3 + ROS + Gazebo integration working reliably
- Zero-DCE real-time enhancement implemented in simulation
- Comprehensive dependency management and build system
- Tested configuration on Ubuntu 20.04 with ROS Noetic

🚧 **In Progress:**
- Integration of additional low-light enhancement models (SCI, other LLIEs)
- YOLO-based person detection for search and rescue scenarios
- Performance evaluation framework for different lighting conditions

🎯 **Planned:**
- Multi-model enhancement comparison and evaluation
- Real-world testing with physical robot platforms
- Search and rescue scenario validation

## Repository Structure

```
LowLightSLAM/
├── src/                          # ROS packages
│   ├── orb_slam3_ros_wrapper/    # ORB-SLAM3 ROS wrapper with custom configurations
│   ├── turtlebot3_simulations/   # TurtleBot3 Gazebo simulation package
│   └── zero_dce_ros/             # Zero-DCE real-time enhancement package
├── ORB_SLAM3/                    # Tested and verified ORB-SLAM3 implementation
├── Zero-DCE_extension/           # Zero-DCE enhancement module (submodule)
├── install_orbslam3_deps.sh      # Dependency installation script
├── verify_orbslam3_deps.sh       # Dependency verification script
├── ORB_SLAM3_WORKING_CONFIG.md   # Detailed configuration documentation
├── build/                        # Build files (auto-generated)
├── devel/                        # Development files (auto-generated)
└── logs/                         # Log files (auto-generated)
```

## System Requirements

**Tested Environment:**
- **OS**: Ubuntu 20.04 LTS
- **ROS**: Noetic
- **Hardware**: x86_64 architecture with CUDA support (optional)

**Core Dependencies:**
- OpenCV 4.2.0+ (tested with 4.2.0)
- Eigen3 3.3.7+
- Pangolin (visualization library)
- Intel RealSense SDK (optional, for physical cameras)
- Python 3.8+ with PyTorch for Zero-DCE

## Installation

### Quick Start

1. **Clone the repository:**
```bash
git clone --recursive https://github.com/sduckworth234/LowLightSLAM.git
cd LowLightSLAM
```

2. **Install ORB-SLAM3 dependencies (recommended):**
```bash
./install_orbslam3_deps.sh
```

3. **Verify installation:**
```bash
./verify_orbslam3_deps.sh
```

### Detailed Installation

#### Step 1: System Dependencies
```bash
# Update system
sudo apt update && sudo apt upgrade

# Install ROS Noetic dependencies
sudo apt install ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters \
                 ros-noetic-image-transport ros-noetic-compressed-image-transport

# Install TurtleBot3 simulation packages
sudo apt install ros-noetic-turtlebot3* ros-noetic-gazebo-ros-pkgs
```

#### Step 2: ORB-SLAM3 Dependencies
The provided script `install_orbslam3_deps.sh` automatically installs:
- OpenCV 4.2.0+ with development headers
- Eigen3 3.3.7+ 
- Pangolin (built from source for optimal compatibility)
- Intel RealSense SDK (for camera support)
- Essential build tools (CMake, GCC, etc.)

#### Step 3: Python Dependencies for Zero-DCE
```bash
# Install PyTorch (CPU version)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Install additional dependencies
pip3 install numpy opencv-python Pillow scipy
```

#### Step 4: Build ORB-SLAM3
```bash
cd ORB_SLAM3
./build.sh
cd ..
```

#### Step 5: Build ROS Workspace
```bash
catkin_make
source devel/setup.bash
```

## Usage Examples

### Basic ORB-SLAM3 Testing

**Monocular SLAM with EuRoC dataset:**
```bash
# Terminal 1: Launch ORB-SLAM3 node
roslaunch orb_slam3_ros_wrapper euroc_mono.launch

# Terminal 2: Play dataset
rosbag play MH_01_easy.bag
```

### TurtleBot3 Simulation with Low-Light Enhancement

**Complete simulation pipeline:**
```bash
# Terminal 1: Launch Gazebo simulation
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch

# Terminal 2: Launch Zero-DCE enhancement node
roslaunch zero_dce_ros zero_dce_enhancement.launch

# Terminal 3: Launch ORB-SLAM3 with enhanced images
roslaunch orb_slam3_ros_wrapper turtlebot3_enhanced.launch

# Terminal 4: Control robot (optional)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Real Robot Integration

**For physical robot deployment:**
```bash
# Terminal 1: Launch camera driver
roslaunch realsense2_camera rs_camera.launch

# Terminal 2: Launch enhancement
roslaunch zero_dce_ros zero_dce_enhancement.launch

# Terminal 3: Launch SLAM
roslaunch orb_slam3_ros_wrapper real_robot_enhanced.launch
```

## Configuration

### Camera Calibration
Camera parameters are located in:
```
src/orb_slam3_ros_wrapper/config/
├── EuRoC.yaml          # EuRoC dataset parameters
├── TUM_mono.yaml       # TUM dataset parameters
└── RealSense.yaml      # RealSense camera parameters
```

### Enhancement Parameters
Zero-DCE enhancement settings:
```
src/zero_dce_ros/config/zero_dce_params.yaml
```

### ORB-SLAM3 Configuration
SLAM parameters can be adjusted in:
```
src/orb_slam3_ros_wrapper/config/ORB_SLAM3/
```

## Performance Considerations

- **Real-time Performance**: Zero-DCE enhancement adds ~50-100ms latency depending on image resolution
- **Memory Usage**: Approximately 2-4GB RAM for complete pipeline
- **GPU Acceleration**: CUDA support significantly improves enhancement speed
- **Image Resolution**: 640x480 recommended for real-time operation

## Future Development

### Planned Enhancements
1. **Additional Low-Light Models**: Integration of SCI (Self-Calibrating Illumination) and other state-of-the-art LLIE methods
2. **Person Detection**: YOLO-based human detection for search and rescue applications
3. **Multi-Sensor Fusion**: Integration with LiDAR and thermal imaging
4. **Evaluation Framework**: Comprehensive performance assessment tools

### Research Applications
- Search and rescue robotics in low-light environments
- Underground exploration and mapping
- Disaster response and recovery operations
- Indoor navigation in power-limited scenarios

## Troubleshooting

### Common Issues

**ORB-SLAM3 Build Failures:**
- Run `./verify_orbslam3_deps.sh` to check dependencies
- Ensure all Thirdparty libraries build successfully
- Check `ORB_SLAM3_WORKING_CONFIG.md` for known working versions

**ROS Integration Issues:**
- Verify ROS environment: `echo $ROS_DISTRO`
- Source workspace: `source devel/setup.bash`
- Check topic connections: `rostopic list`

**Zero-DCE Performance:**
- For slow enhancement, reduce image resolution
- Install CUDA version of PyTorch for GPU acceleration
- Monitor system resources during operation

### Getting Help

For detailed troubleshooting and configuration information, see:
- `ORB_SLAM3_WORKING_CONFIG.md` - Complete dependency analysis
- `DEVELOPER_GUIDE.md` - Development and contribution guidelines
- GitHub Issues - Community support and bug reports

## Contributing

This is an active research project. Contributions are welcome, particularly:
- Additional low-light enhancement models
- Performance optimizations
- Real-world testing and validation
- Documentation improvements

Please follow standard academic software development practices and cite relevant work appropriately.

## Citation

If you use this work in your research, please cite:

```bibtex
@misc{lowlightslam2025,
  title={LowLightSLAM: Enhanced Visual SLAM for Search and Rescue in Low-Light Environments},
  author={[Your Name]},
  year={2025},
  note={Thesis Research - University Project},
  howpublished={\url{https://github.com/sduckworth234/LowLightSLAM}}
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **ORB-SLAM3**: UZ-SLAMLab for the robust visual SLAM framework
- **Zero-DCE**: Li et al. for the efficient low-light enhancement method
- **TurtleBot3**: ROBOTIS for the simulation platform
- **ROS Community**: For the extensive robotics middleware

---

**Status**: Active development for thesis research (2025)  
**Last Updated**: July 16, 2025

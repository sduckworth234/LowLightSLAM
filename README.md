# LowLightSLAM

A comprehensive ROS workspace for low-light SLAM research, combining ORB-SLAM3 with low-light image enhancement techniques for improved visual SLAM performance in challenging lighting conditions.

## 🚀 Overview

This repository contains a complete ROS catkin workspace that integrates:
- **ORB-SLAM3**: State-of-the-art visual SLAM system
- **Zero-DCE Enhancement**: Deep learning-based low-light image enhancement
- **TurtleBot3 Simulation**: Robot simulation environment
- **Custom ROS Wrapper**: Modified ORB-SLAM3 ROS interface with enhanced features

## 📁 Repository Structure

```
LowLightSLAM/
├── src/                          # ROS packages
│   ├── orb_slam3_ros_wrapper/    # Modified ORB-SLAM3 ROS wrapper
│   ├── turtlebot3_simulations/   # TurtleBot3 simulation package
│   └── zero_dce_ros/             # Zero-DCE ROS integration package
├── ORB_SLAM3/                    # ORB-SLAM3 submodule
├── Zero-DCE_extension/           # Zero-DCE enhancement submodule
├── build/                        # Build files (auto-generated)
├── devel/                        # Development files (auto-generated)
└── logs/                         # Log files (auto-generated)
```

## 🛠️ Installation

### Prerequisites

- Ubuntu 18.04/20.04 with ROS Melodic/Noetic
- OpenCV 4.x
- Eigen3
- Pangolin
- Python 3.6+
- CUDA (optional, for GPU acceleration)

### Clone the Repository

```bash
git clone --recursive https://github.com/sduckworth234/LowLightSLAM.git
cd LowLightSLAM
```

### Install Dependencies

#### ORB_SLAM3 Dependencies (Recommended)
For the best compatibility with our tested and working ORB_SLAM3 setup:

```bash
# Install ORB_SLAM3 specific dependencies (OpenCV, Eigen3, Pangolin, RealSense)
./install_orbslam3_deps.sh
```

#### System Dependencies  
```bash
# Install ROS dependencies
sudo apt update
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-message-filters ros-$ROS_DISTRO-image-transport

# Install OpenCV dependencies
sudo apt install libopencv-dev python3-opencv

# Install Eigen3
sudo apt install libeigen3-dev

# Install other dependencies
sudo apt install libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
### Build ORB-SLAM3

ORB_SLAM3 is included as a tested, working version in this repository.

```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

The build script will:
- Build all Thirdparty libraries (DBoW2, g2o, Sophus)  
- Extract the vocabulary file automatically
- Build the main ORB_SLAM3 library

**Note**: This ORB_SLAM3 version has been tested and verified working. See `ORB_SLAM3_WORKING_CONFIG.md` for detailed configuration information.

### Build the ROS Workspace

```bash
cd /path/to/LowLightSLAM
catkin_make
source devel/setup.bash
```

### Python Dependencies for Zero-DCE

```bash
pip3 install torch torchvision numpy opencv-python Pillow
```

## 🚗 Usage

### 1. Basic ORB-SLAM3 with Camera

```bash
# Terminal 1: Launch the ORB-SLAM3 node
roslaunch orb_slam3_ros_wrapper euroc_mono.launch

# Terminal 2: Play your dataset or start camera
rosbag play your_dataset.bag
```

### 2. TurtleBot3 Simulation with Low-Light Enhancement

```bash
# Terminal 1: Launch TurtleBot3 simulation
roslaunch turtlebot3_simulations turtlebot3_house.launch

# Terminal 2: Launch Zero-DCE enhancement node
roslaunch zero_dce_ros zero_dce_enhancement.launch

# Terminal 3: Launch ORB-SLAM3 with enhanced images
roslaunch orb_slam3_ros_wrapper turtlebot3_enhanced.launch

# Terminal 4: Control the robot
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### 3. Real Robot with Low-Light Enhancement

```bash
# Terminal 1: Launch camera driver
roslaunch your_camera_driver camera.launch

# Terminal 2: Launch Zero-DCE enhancement
roslaunch zero_dce_ros zero_dce_enhancement.launch

# Terminal 3: Launch ORB-SLAM3
roslaunch orb_slam3_ros_wrapper real_robot_enhanced.launch
```

## 📊 Available Launch Files

### ORB-SLAM3 ROS Wrapper
- `euroc_mono.launch` - EuRoC dataset monocular SLAM
- `euroc_stereo.launch` - EuRoC dataset stereo SLAM
- `turtlebot3_enhanced.launch` - TurtleBot3 with image enhancement
- `real_robot_enhanced.launch` - Real robot with image enhancement

### Zero-DCE ROS
- `zero_dce_enhancement.launch` - Image enhancement node
- `zero_dce_params.launch` - Enhancement with custom parameters

## 🔧 Configuration

### Camera Calibration
Update camera calibration files in:
```
src/orb_slam3_ros_wrapper/config/
```

### Zero-DCE Parameters
Modify enhancement parameters in:
```
src/zero_dce_ros/config/zero_dce_params.yaml
```

### ORB-SLAM3 Parameters
Adjust SLAM parameters in:
```
src/orb_slam3_ros_wrapper/config/ORB_SLAM3/
```

## 📈 Performance Tips

1. **GPU Acceleration**: Enable CUDA for Zero-DCE enhancement
2. **Image Resolution**: Lower resolution improves real-time performance
3. **Enhancement Strength**: Adjust based on lighting conditions
4. **ORB Features**: Tune ORB feature extraction parameters for enhanced images

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📝 Citation

If you use this work in your research, please cite:

```bibtex
@misc{lowlightslam2024,
  title={LowLightSLAM: Enhanced Visual SLAM for Low-Light Environments},
  author={Your Name},
  year={2024},
  publisher={GitHub},
  howpublished={\url{https://github.com/sduckworth234/LowLightSLAM}}
}
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) by UZ-SLAMLab
- [Zero-DCE](https://github.com/Li-Chongyi/Zero-DCE_extension) by Li-Chongyi
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) by ROBOTIS
- Original ORB-SLAM3 ROS wrapper contributors

## 🐛 Known Issues

- GPU memory requirements for Zero-DCE enhancement
- Real-time performance depends on hardware capabilities
- Camera synchronization in stereo setups

## 📞 Support

For questions and support, please open an issue on GitHub or contact [your-email@domain.com].

---

**Happy SLAM-ing in low light! 🌙🤖**

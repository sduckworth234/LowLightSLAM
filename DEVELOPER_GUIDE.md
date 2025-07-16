# Developer Guide - LowLightSLAM

This guide provides detailed information for developers working with the LowLightSLAM workspace.

## 🏗️ Architecture Overview

### System Components

1. **ORB-SLAM3 Core**: Visual SLAM system for tracking and mapping
2. **Zero-DCE Enhancement**: Low-light image enhancement neural network
3. **ROS Integration**: Wrapper nodes for seamless ROS communication
4. **Simulation Environment**: TurtleBot3 Gazebo simulation

### Data Flow

```
Camera/Simulation → Zero-DCE Enhancement → ORB-SLAM3 → Map/Trajectory Output
                      ↓
                 Enhanced Images (optional bypass)
```

## 📁 Package Details

### orb_slam3_ros_wrapper

**Purpose**: ROS interface for ORB-SLAM3 with enhanced features

**Key Files**:
- `src/orb_slam3_ros_wrapper_mono.cpp` - Monocular SLAM node
- `src/orb_slam3_ros_wrapper_stereo.cpp` - Stereo SLAM node
- `launch/` - Launch configurations
- `config/` - Camera calibration and SLAM parameters

**Topics**:
- **Subscribes**: `/camera/image_raw`, `/camera/image_rect`
- **Publishes**: `/orb_slam3/camera_pose`, `/orb_slam3/map_points`, `/orb_slam3/trajectory`

**Services**:
- `SaveMap.srv` - Save map to file

### zero_dce_ros

**Purpose**: ROS wrapper for Zero-DCE low-light enhancement

**Key Files**:
- `scripts/zero_dce_node.py` - Main enhancement node
- `src/zero_dce_wrapper.cpp` - C++ wrapper (if available)
- `models/` - Pre-trained Zero-DCE models
- `config/zero_dce_params.yaml` - Enhancement parameters

**Topics**:
- **Subscribes**: `/camera/image_raw`
- **Publishes**: `/camera/image_enhanced`

**Parameters**:
- `enhancement_strength` - Enhancement intensity (0.0-1.0)
- `model_path` - Path to Zero-DCE model
- `input_size` - Input image dimensions

### turtlebot3_simulations

**Purpose**: Simulation environment for testing

**Included Packages**:
- `turtlebot3_gazebo` - Gazebo simulation worlds
- `turtlebot3_fake` - Fake robot for testing without Gazebo

## 🔧 Development Setup

### Building from Source

1. **Clone with submodules**:
   ```bash
   git clone --recursive https://github.com/yourusername/LowLightSLAM.git
   ```

2. **Install dependencies**:
   ```bash
   ./install.sh
   ```

3. **Build workspace**:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

### Development Workflow

1. **Make changes** to source code
2. **Build specific package**:
   ```bash
   catkin_make --only-pkg-with-deps package_name
   ```
3. **Test changes**:
   ```bash
   roslaunch package_name test_launch.launch
   ```

## 🧪 Testing

### Unit Tests

Run individual package tests:
```bash
catkin_make run_tests_orb_slam3_ros_wrapper
catkin_make run_tests_zero_dce_ros
```

### Integration Tests

Test complete system:
```bash
# Terminal 1: Launch simulation
roslaunch turtlebot3_gazebo turtlebot3_house.launch

# Terminal 2: Launch enhancement
roslaunch zero_dce_ros zero_dce_enhancement.launch

# Terminal 3: Launch SLAM
roslaunch orb_slam3_ros_wrapper turtlebot3_enhanced.launch

# Terminal 4: Move robot and verify tracking
rostopic echo /orb_slam3/camera_pose
```

### Performance Benchmarking

Monitor system performance:
```bash
# CPU/Memory usage
htop

# ROS topic rates
rostopic hz /camera/image_raw
rostopic hz /camera/image_enhanced
rostopic hz /orb_slam3/camera_pose

# Bag file recording for analysis
rosbag record -a -o test_session
```

## 🔧 Configuration

### Camera Calibration

1. **Calibrate your camera** using ROS camera_calibration package
2. **Update calibration file** in `config/camera_calibration.yaml`
3. **Modify launch file** to use correct calibration

### ORB-SLAM3 Parameters

Key parameters in `config/ORB_SLAM3/settings.yaml`:

```yaml
# Feature extraction
nFeatures: 1000
scaleFactor: 1.2
nLevels: 8

# Tracking
minFrames: 0
maxFrames: 20

# Local mapping
KeyFrame.nMapPoints: 100
```

### Zero-DCE Parameters

Parameters in `config/zero_dce_params.yaml`:

```yaml
# Enhancement settings
enhancement_strength: 0.8
adaptive_enhancement: true
preserve_colors: true

# Performance settings
use_gpu: true
batch_size: 1
input_size: [512, 512]
```

## 🐛 Debugging

### Common Issues

1. **ORB-SLAM3 initialization failure**:
   - Check camera calibration
   - Verify image topics
   - Ensure sufficient features in first frames

2. **Zero-DCE memory errors**:
   - Reduce input image size
   - Check GPU memory availability
   - Disable GPU acceleration if needed

3. **ROS communication issues**:
   - Verify topic names with `rostopic list`
   - Check message types with `rostopic info`
   - Monitor message rates with `rostopic hz`

### Debug Tools

```bash
# ROS graph visualization
rqt_graph

# Image visualization
rqt_image_view

# Parameter monitoring
rqt_reconfigure

# Log analysis
rqt_console
```

### Logging

Enable debug logging:
```bash
export ROSCONSOLE_MIN_SEVERITY=DEBUG
roslaunch your_package your_launch.launch
```

## 📊 Performance Optimization

### CPU Optimization

1. **Reduce image resolution** for real-time performance
2. **Adjust ORB feature count** based on scene complexity
3. **Enable multi-threading** where possible

### GPU Optimization

1. **Use CUDA** for Zero-DCE enhancement
2. **Optimize batch size** for your GPU memory
3. **Consider TensorRT** for inference acceleration

### Memory Management

1. **Monitor memory usage** with `htop` or `free -h`
2. **Implement image buffering** for smoother processing
3. **Clear old map points** in long-running sessions

## 🤝 Contributing Guidelines

### Code Style

- **C++**: Follow Google C++ Style Guide
- **Python**: Follow PEP 8
- **ROS**: Follow ROS coding standards

### Pull Request Process

1. **Fork** the repository
2. **Create feature branch**: `git checkout -b feature/my-feature`
3. **Make changes** with proper testing
4. **Update documentation** if needed
5. **Submit pull request** with detailed description

### Testing Requirements

- All new features must include unit tests
- Integration tests for system-level changes
- Performance benchmarks for optimization changes

## 📚 Additional Resources

- [ORB-SLAM3 Paper](https://arxiv.org/abs/2007.11898)
- [Zero-DCE Paper](https://arxiv.org/abs/2001.06826)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

## 📞 Getting Help

- **GitHub Issues**: For bugs and feature requests
- **ROS Answers**: For ROS-specific questions
- **Stack Overflow**: For general programming questions

Happy developing! 🚀

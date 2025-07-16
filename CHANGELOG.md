# Changelog

All notable changes to the LowLightSLAM project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial repository structure with comprehensive documentation
- Git submodules for ORB-SLAM3 and Zero-DCE dependencies
- Automated installation and setup scripts
- Developer guide with debugging and optimization tips

### Changed
- Updated ORB-SLAM3 ROS wrapper to use relative paths for submodule
- Modified CMakeLists.txt to work with workspace-local ORB-SLAM3

### Fixed
- Path dependencies for ORB-SLAM3 integration
- Git submodule configuration for reproducible builds

## [1.0.0] - 2024-07-16

### Added
- Complete ROS catkin workspace for low-light SLAM research
- ORB-SLAM3 integration as git submodule
- Zero-DCE low-light enhancement as git submodule  
- Modified orb_slam3_ros_wrapper with enhanced features
- TurtleBot3 simulation support
- Custom zero_dce_ros package for ROS integration
- Comprehensive README with installation and usage instructions
- MIT license for open source distribution
- .gitignore configured for ROS workspace
- Installation script for automated dependency setup
- Setup script for new repository clones

### Technical Details
- ORB-SLAM3 from UZ-SLAMLab repository
- Zero-DCE extension from Li-Chongyi repository
- ROS Melodic/Noetic compatibility
- OpenCV 4.x integration
- Pangolin visualization support
- Python 3.6+ compatibility

### Documentation
- Complete README with usage examples
- Developer guide for contributors
- Installation instructions for all dependencies
- Launch file documentation
- Configuration parameter explanations

---

## Template for Future Releases

## [X.Y.Z] - YYYY-MM-DD

### Added
- New features

### Changed
- Changes in existing functionality

### Deprecated
- Soon-to-be removed features

### Removed
- Now removed features

### Fixed
- Bug fixes

### Security
- Vulnerability fixes

#!/bin/bash

# ORB_SLAM3 Dependency Installation Script
# This script installs all dependencies required for ORB_SLAM3 based on your working setup
# Tested on Ubuntu 20.04 with ROS Noetic

set -e  # Exit on any error

echo "=================================================="
echo "ORB_SLAM3 Dependency Installation Script"
echo "=================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_error "This script should not be run as root (don't use sudo)"
   exit 1
fi

# Update package lists
print_status "Updating package lists..."
sudo apt update

# Install basic build tools
print_status "Installing basic build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    pkg-config \
    software-properties-common

# Install OpenCV 4.2+ dependencies and OpenCV
print_status "Installing OpenCV dependencies..."
sudo apt install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    python3-opencv

# Check OpenCV version
OPENCV_VERSION=$(pkg-config --modversion opencv4 2>/dev/null || pkg-config --modversion opencv 2>/dev/null || echo "unknown")
print_status "OpenCV version detected: $OPENCV_VERSION"

if [[ "$OPENCV_VERSION" < "4.2" ]] && [[ "$OPENCV_VERSION" != "unknown" ]]; then
    print_warning "OpenCV version $OPENCV_VERSION is older than required 4.2. You may need to build OpenCV from source."
fi

# Install Eigen3
print_status "Installing Eigen3..."
sudo apt install -y libeigen3-dev

# Check Eigen3 version
EIGEN_VERSION=$(pkg-config --modversion eigen3 2>/dev/null || echo "unknown")
print_status "Eigen3 version detected: $EIGEN_VERSION"

# Install other essential dependencies
print_status "Installing additional dependencies..."
sudo apt install -y \
    libgl1-mesa-dev \
    libglew-dev \
    libglfw3-dev \
    libxkbcommon-dev \
    libwayland-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev

# Install RealSense dependencies (since your setup uses it)
print_status "Installing Intel RealSense dependencies..."
# Add Intel RealSense repository key
wget -qO - https://librealsense.intel.com/Debian/apt-repo/conf/librealsense.pgp | sudo apt-key add -

# Add repository
echo 'deb https://librealsense.intel.com/Debian/apt-repo focal main' | sudo tee /etc/apt/sources.list.d/librealsense.list

# Update and install RealSense
sudo apt update
sudo apt install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

# Install Pangolin (build from source since it's not in apt)
print_status "Installing Pangolin dependencies..."
sudo apt install -y \
    libepoxy-dev \
    libegl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev

# Check if Pangolin is already installed
if [ -f "/usr/local/lib/cmake/Pangolin/PangolinConfig.cmake" ]; then
    print_success "Pangolin is already installed at /usr/local"
else
    print_status "Building and installing Pangolin from source..."
    
    # Create temporary directory
    TEMP_DIR=$(mktemp -d)
    cd "$TEMP_DIR"
    
    # Clone Pangolin
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    
    # Build Pangolin
    mkdir build
    cd build
    cmake ..
    make -j$(nproc)
    
    # Install Pangolin
    sudo make install
    
    # Update library cache
    sudo ldconfig
    
    # Clean up
    cd /
    rm -rf "$TEMP_DIR"
    
    print_success "Pangolin installed successfully"
fi

# Install ROS dependencies (if ROS is installed)
if command -v rospack >/dev/null 2>&1; then
    print_status "ROS detected. Installing ROS dependencies for ORB_SLAM3..."
    sudo apt install -y \
        ros-$ROS_DISTRO-cv-bridge \
        ros-$ROS_DISTRO-tf \
        ros-$ROS_DISTRO-message-filters \
        ros-$ROS_DISTRO-image-transport
else
    print_warning "ROS not detected. ROS dependencies will be skipped."
fi

# Verify installations
print_status "Verifying installations..."

# Check OpenCV
if pkg-config --exists opencv4 || pkg-config --exists opencv; then
    print_success "OpenCV is installed"
else
    print_error "OpenCV installation failed"
fi

# Check Eigen3
if pkg-config --exists eigen3; then
    print_success "Eigen3 is installed"
else
    print_error "Eigen3 installation failed"
fi

# Check Pangolin
if [ -f "/usr/local/lib/libpangolin.so" ]; then
    print_success "Pangolin is installed"
else
    print_error "Pangolin installation failed"
fi

# Check RealSense
if pkg-config --exists realsense2; then
    print_success "Intel RealSense is installed"
else
    print_warning "Intel RealSense may not be properly installed"
fi

print_status "=================================================="
print_success "ORB_SLAM3 dependency installation completed!"
print_status "=================================================="

echo ""
print_status "Next steps:"
echo "1. Navigate to the ORB_SLAM3 directory"
echo "2. Run: ./build.sh"
echo "3. The build script will:"
echo "   - Build Thirdparty libraries (DBoW2, g2o, Sophus)"
echo "   - Extract vocabulary files"
echo "   - Build ORB_SLAM3"
echo ""
print_status "Your working configuration versions:"
echo "- OpenCV: $OPENCV_VERSION"
echo "- Eigen3: $EIGEN_VERSION"
echo "- RealSense: $(pkg-config --modversion realsense2 2>/dev/null || echo 'unknown')"
echo "- Pangolin: Installed from source"

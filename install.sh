#!/bin/bash

# LowLightSLAM Installation Script
# This script installs all dependencies required for the LowLightSLAM workspace

set -e

echo "🚀 Starting LowLightSLAM installation..."

# Check if ROS is installed
if ! command -v roscore &> /dev/null; then
    echo "❌ ROS is not installed. Please install ROS first."
    exit 1
fi

echo "✅ ROS found: $(rosversion -d)"

# Install system dependencies
echo "📦 Installing system dependencies..."
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    libopencv-dev \
    python3-opencv \
    libeigen3-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    python3-pip

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
pip3 install --user torch torchvision numpy opencv-python Pillow

# Check if Pangolin is installed
if ! pkg-config --exists pangolin; then
    echo "📊 Installing Pangolin..."
    cd /tmp
    if [ -d "Pangolin" ]; then
        rm -rf Pangolin
    fi
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig
    echo "✅ Pangolin installed successfully"
else
    echo "✅ Pangolin already installed"
fi

echo "🏗️ Building ORB-SLAM3..."
cd ORB_SLAM3
chmod +x build.sh
./build.sh

echo "🏗️ Building ROS workspace..."
cd ..
catkin_make

echo "✅ Installation completed successfully!"
echo ""
echo "🎉 To use the workspace, run:"
echo "   source devel/setup.bash"
echo ""
echo "📖 Check the README.md for usage examples."

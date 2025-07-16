#!/bin/bash

# Quick verification script for LowLightSLAM workspace
# This script performs basic checks to ensure the workspace is properly set up

set -e

echo "🔍 Verifying LowLightSLAM workspace setup..."

# Check if we're in the right directory
if [ ! -f "README.md" ] || [ ! -d "src" ]; then
    echo "❌ Please run this script from the LowLightSLAM root directory"
    exit 1
fi

# Check submodules
echo "📦 Checking submodules..."
if [ ! -d "ORB_SLAM3" ] || [ ! -d "Zero-DCE_extension" ]; then
    echo "❌ Submodules not found. Run 'git submodule update --init --recursive'"
    exit 1
fi
echo "✅ Submodules present"

# Check ROS packages
echo "📁 Checking ROS packages..."
required_packages=("orb_slam3_ros_wrapper" "zero_dce_ros" "turtlebot3_simulations")
for pkg in "${required_packages[@]}"; do
    if [ ! -d "src/$pkg" ]; then
        echo "❌ Package $pkg not found in src/"
        exit 1
    fi
done
echo "✅ All required packages present"

# Check if workspace was built
echo "🏗️ Checking if workspace was built..."
if [ ! -d "devel" ] || [ ! -f "devel/setup.bash" ]; then
    echo "⚠️  Workspace not built. Run 'catkin_make' to build"
else
    echo "✅ Workspace appears to be built"
fi

# Check ORB_SLAM3 build
echo "🔧 Checking ORB-SLAM3 build..."
if [ ! -f "ORB_SLAM3/lib/libORB_SLAM3.so" ]; then
    echo "⚠️  ORB-SLAM3 not built. Run './ORB_SLAM3/build.sh' to build"
else
    echo "✅ ORB-SLAM3 appears to be built"
fi

# Check Python dependencies
echo "🐍 Checking Python dependencies..."
python3 -c "import torch, cv2, numpy" 2>/dev/null && echo "✅ Python dependencies OK" || echo "⚠️  Python dependencies missing"

# Test ROS environment
if command -v roscore &> /dev/null; then
    echo "✅ ROS is available"
    if [ -f "devel/setup.bash" ]; then
        source devel/setup.bash
        if rospack find orb_slam3_ros_wrapper &> /dev/null; then
            echo "✅ ROS packages can be found"
        else
            echo "⚠️  ROS packages not found in path"
        fi
    fi
else
    echo "⚠️  ROS not found in PATH"
fi

echo ""
echo "🎉 Verification complete!"
echo ""
echo "Next steps:"
echo "1. If any warnings appeared, follow the suggested fixes"
echo "2. Source the workspace: source devel/setup.bash"
echo "3. Test with a launch file: roslaunch orb_slam3_ros_wrapper euroc_mono.launch"

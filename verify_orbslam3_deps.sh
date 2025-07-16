#!/bin/bash

# ORB_SLAM3 Dependency Verification Script
# Verifies that all required dependencies for ORB_SLAM3 are properly installed

set -e

echo "=================================================="
echo "ORB_SLAM3 Dependency Verification"
echo "=================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_check() {
    echo -e "${BLUE}[CHECK]${NC} $1"
}

print_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

print_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Counter for failed checks
FAILED=0

# Check OpenCV
print_check "Checking OpenCV..."
if pkg-config --exists opencv4; then
    OPENCV_VERSION=$(pkg-config --modversion opencv4)
    print_pass "OpenCV 4.x found: $OPENCV_VERSION"
elif pkg-config --exists opencv; then
    OPENCV_VERSION=$(pkg-config --modversion opencv)
    print_pass "OpenCV found: $OPENCV_VERSION"
else
    print_fail "OpenCV not found"
    FAILED=$((FAILED + 1))
fi

# Check Eigen3
print_check "Checking Eigen3..."
if pkg-config --exists eigen3; then
    EIGEN_VERSION=$(pkg-config --modversion eigen3)
    print_pass "Eigen3 found: $EIGEN_VERSION"
else
    print_fail "Eigen3 not found"
    FAILED=$((FAILED + 1))
fi

# Check Pangolin
print_check "Checking Pangolin..."
if [ -f "/usr/local/lib/libpangolin.so" ] && [ -f "/usr/local/lib/cmake/Pangolin/PangolinConfig.cmake" ]; then
    print_pass "Pangolin found (installed from source)"
elif ldconfig -p | grep -q pangolin; then
    print_pass "Pangolin found (system installation)"
else
    print_fail "Pangolin not found"
    FAILED=$((FAILED + 1))
fi

# Check RealSense (optional)
print_check "Checking Intel RealSense (optional)..."
if pkg-config --exists realsense2; then
    REALSENSE_VERSION=$(pkg-config --modversion realsense2)
    print_pass "Intel RealSense found: $REALSENSE_VERSION"
else
    print_warn "Intel RealSense not found (optional - only needed for RealSense cameras)"
fi

# Check build tools
print_check "Checking build tools..."
if command -v cmake >/dev/null 2>&1; then
    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
    print_pass "CMake found: $CMAKE_VERSION"
else
    print_fail "CMake not found"
    FAILED=$((FAILED + 1))
fi

if command -v make >/dev/null 2>&1; then
    print_pass "Make found"
else
    print_fail "Make not found"
    FAILED=$((FAILED + 1))
fi

if command -v g++ >/dev/null 2>&1; then
    GCC_VERSION=$(g++ --version | head -n1 | cut -d' ' -f4)
    print_pass "G++ found: $GCC_VERSION"
else
    print_fail "G++ not found"
    FAILED=$((FAILED + 1))
fi

# Check if ORB_SLAM3 directory exists
print_check "Checking ORB_SLAM3 directory..."
if [ -d "ORB_SLAM3" ]; then
    print_pass "ORB_SLAM3 directory found"
    
    # Check if build script exists
    if [ -f "ORB_SLAM3/build.sh" ]; then
        print_pass "ORB_SLAM3 build script found"
    else
        print_fail "ORB_SLAM3 build script not found"
        FAILED=$((FAILED + 1))
    fi
    
    # Check Thirdparty directories
    for lib in DBoW2 g2o Sophus; do
        if [ -d "ORB_SLAM3/Thirdparty/$lib" ]; then
            print_pass "Thirdparty/$lib directory found"
        else
            print_fail "Thirdparty/$lib directory not found"
            FAILED=$((FAILED + 1))
        fi
    done
    
    # Check Vocabulary
    if [ -f "ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz" ]; then
        print_pass "ORB vocabulary archive found"
    else
        print_fail "ORB vocabulary archive not found"
        FAILED=$((FAILED + 1))
    fi
    
else
    print_fail "ORB_SLAM3 directory not found - are you in the LowLightSLAM root directory?"
    FAILED=$((FAILED + 1))
fi

# Check ROS (optional)
print_check "Checking ROS (optional)..."
if command -v rospack >/dev/null 2>&1; then
    ROS_DISTRO=$(rospack find rosgraph | cut -d'/' -f4)
    print_pass "ROS found: $ROS_DISTRO"
else
    print_warn "ROS not found (optional - only needed for ROS examples)"
fi

echo ""
echo "=================================================="
if [ $FAILED -eq 0 ]; then
    print_pass "All essential dependencies are installed!"
    echo ""
    echo "You can now build ORB_SLAM3:"
    echo "  cd ORB_SLAM3"
    echo "  ./build.sh"
else
    print_fail "$FAILED essential dependencies are missing!"
    echo ""
    echo "Run the dependency installation script:"
    echo "  ./install_orbslam3_deps.sh"
fi
echo "=================================================="

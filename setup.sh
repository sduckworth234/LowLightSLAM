#!/bin/bash

# LowLightSLAM Setup Script
# Run this script after cloning the repository to initialize submodules and build

set -e

echo "🚀 Setting up LowLightSLAM workspace..."

# Initialize and update submodules
echo "📦 Initializing git submodules..."
git submodule init
git submodule update --recursive

echo "✅ Submodules initialized successfully"

# Check if dependencies should be installed
read -p "Do you want to install system dependencies? [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🔧 Running installation script..."
    ./install.sh
else
    echo "⚠️  Skipping dependency installation."
    echo "   Make sure to install dependencies manually or run ./install.sh later"
fi

echo "✅ Setup completed!"
echo ""
echo "🎉 Your LowLightSLAM workspace is ready!"
echo ""
echo "Next steps:"
echo "1. Source the workspace: source devel/setup.bash"
echo "2. Check the README.md for usage examples"
echo "3. Test with: roslaunch orb_slam3_ros_wrapper euroc_mono.launch"

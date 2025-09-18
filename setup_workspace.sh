#!/bin/bash
# TurtleBot Workspace Setup Script

echo "🤖 TurtleBot Autonomous Navigation Workspace Setup"
echo "=================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if ROS2 is installed
check_ros2() {
    echo -e "${BLUE}Checking ROS2 installation...${NC}"
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}❌ ROS2 not found! Please install ROS2 first.${NC}"
        echo "   Visit: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    else
        echo -e "${GREEN}✅ ROS2 found${NC}"
        ros2 --version
    fi
}

# Check dependencies
check_dependencies() {
    echo -e "${BLUE}Checking Python dependencies...${NC}"

    # Check Python packages
    python3 -c "import numpy, scipy, matplotlib" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Core Python packages available${NC}"
    else
        echo -e "${YELLOW}⚠️  Installing core Python packages...${NC}"
        pip3 install numpy scipy matplotlib
    fi

    # Check for Open3D
    python3 -c "import open3d" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Open3D available${NC}"
    else
        echo -e "${YELLOW}⚠️  Installing Open3D...${NC}"
        pip3 install open3d
    fi

    # Check for sklearn
    python3 -c "import sklearn" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Scikit-learn available${NC}"
    else
        echo -e "${YELLOW}⚠️  Installing Scikit-learn...${NC}"
        pip3 install scikit-learn
    fi
}

# Build workspace
build_workspace() {
    echo -e "${BLUE}Building TurtleBot workspace...${NC}"

    # Clean previous build if requested
    if [ "$1" = "clean" ]; then
        echo -e "${YELLOW}🧹 Cleaning previous build...${NC}"
        rm -rf build install log
    fi

    # Build
    colcon build --packages-select turtlebot_pilot --cmake-args -DCMAKE_BUILD_TYPE=Release

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Build successful!${NC}"
    else
        echo -e "${RED}❌ Build failed!${NC}"
        exit 1
    fi
}

# Setup environment
setup_environment() {
    echo -e "${BLUE}Setting up environment...${NC}"

    # Source the workspace
    source install/setup.bash

    # Create convenience aliases
    echo -e "${YELLOW}💡 Adding convenience commands to ~/.bashrc${NC}"

    cat >> ~/.bashrc << 'EOF'

# TurtleBot Workspace Aliases
alias tb_ws='cd ~/turtlebot_ws && source install/setup.bash'
alias tb_build='cd ~/turtlebot_ws && colcon build --packages-select turtlebot_pilot && source install/setup.bash'
alias tb_nav='ros2 launch turtlebot_pilot navigator.launch.py'
alias tb_explore='ros2 launch turtlebot_pilot explorer.launch.py'
alias tb_plot='ros2 run turtlebot_pilot p3_plot.py'
EOF
}

# Print usage information
print_usage() {
    echo -e "${GREEN}🚀 Workspace setup complete!${NC}"
    echo ""
    echo -e "${BLUE}Quick Start Commands:${NC}"
    echo "  tb_ws              - Navigate to workspace and source environment"
    echo "  tb_build           - Build the workspace"
    echo "  tb_nav             - Launch autonomous navigation"
    echo "  tb_explore         - Launch exploration mode"
    echo "  tb_plot            - Start trajectory plotting"
    echo ""
    echo -e "${BLUE}Manual Commands:${NC}"
    echo "  colcon build --packages-select turtlebot_pilot"
    echo "  source install/setup.bash"
    echo "  ros2 launch turtlebot_pilot navigator.launch.py"
    echo ""
    echo -e "${YELLOW}💡 Restart your terminal or run 'source ~/.bashrc' to use aliases${NC}"
}

# Main execution
main() {
    echo -e "${BLUE}Starting workspace setup...${NC}"

    # Navigate to workspace directory
    cd "$(dirname "$0")"

    check_ros2
    check_dependencies
    build_workspace $1
    setup_environment
    print_usage

    echo -e "${GREEN}🎉 TurtleBot workspace is ready!${NC}"
}

# Handle command line arguments
case "$1" in
    clean)
        main clean
        ;;
    help|--help|-h)
        echo "Usage: $0 [clean|help]"
        echo "  clean  - Clean build before building"
        echo "  help   - Show this help message"
        ;;
    *)
        main
        ;;
esac
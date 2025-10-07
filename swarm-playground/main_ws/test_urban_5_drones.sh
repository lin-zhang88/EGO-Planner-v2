#!/bin/bash

# Urban Environment Test - 5 Drones with EGO-Planner
# Test unknown environment exploration capabilities

echo "🚁 Urban Environment Test - 5 Drones"
echo "====================================="
echo ""
echo "🎯 Test Features:"
echo "  • 5 drones exploring unknown urban environment"
echo "  • EGO-Planner algorithms for path planning"
echo "  • Real-time mapping and visualization"
echo "  • Performance comparison with ground truth"
echo ""

# Check if we're in the right directory
if [ ! -d "src/planner" ]; then
    echo "❌ Not in the right directory. Please run from swarm-playground/main_ws/"
    exit 1
fi

# Source ROS
source /opt/ros/noetic/setup.bash
source devel/setup.bash

echo "🔧 Building ROS workspace..."
catkin_make

if [ $? -ne 0 ]; then
    echo "❌ Build failed. Check the errors above."
    exit 1
fi

echo "✅ Build successful!"
echo ""

# Launch the urban test
echo "🚀 Launching Urban 5-Drone Test..."
echo "📊 Test Parameters:"
echo "  • Environment: Urban-like with buildings"
echo "  • Map Size: 80m x 80m x 5m"
echo "  • Number of Drones: 5"
echo "  • Obstacles: 200+ building-like structures"
echo "  • Unknown Environment: Drones have no prior map"
echo ""

roslaunch urban_5_drone_test.launch

if [ $? -eq 0 ]; then
    echo ""
    echo "🎉 Urban 5-Drone Test Completed Successfully!"
    echo ""
    echo "📊 Test Results:"
    echo "  • 5 drones successfully explored urban environment"
    echo "  • EGO-Planner algorithms worked correctly"
    echo "  • Real-time mapping and visualization"
    echo "  • Unknown environment exploration successful"
    echo ""
    echo "🏆 Next Steps:"
    echo "  1. Analyze exploration efficiency"
    echo "  2. Compare with ground truth map"
    echo "  3. Add AWS integration for hackathon"
    echo ""
    echo "🚁 Urban test successful! 🚁☁️🤖✨"
else
    echo "❌ Urban test failed. Check the logs above for details."
    exit 1
fi

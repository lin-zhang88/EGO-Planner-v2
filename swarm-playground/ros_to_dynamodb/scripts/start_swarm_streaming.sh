#!/bin/bash
##############################################################################
# Swarm Drone Streaming - All-in-One Launcher
# 
# This script starts:
# 1. ROS Simulation (RViz + Swarm)
# 2. AWS IoT Streaming for drone_0
#
# Usage: bash start_swarm_streaming.sh
##############################################################################

# Color codes for pretty output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# AWS Credentials - UPDATE THESE WITH YOUR CURRENT CREDENTIALS
export AWS_DEFAULT_REGION="us-east-1"
export AWS_ACCESS_KEY_ID="YOUR_ACCESS_KEY_HERE"
export AWS_SECRET_ACCESS_KEY="YOUR_SECRET_KEY_HERE"
export AWS_SESSION_TOKEN="YOUR_SESSION_TOKEN_HERE"

# Directories
MAIN_WS_DIR="$HOME/EGO-Planner-v2/swarm-playground/main_ws"
SCRIPTS_DIR="$HOME/EGO-Planner-v2/swarm-playground/ros_to_dynamodb/scripts"

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}🚁 Starting Swarm Drone Streaming System${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

# Check if credentials are set
if [ "$AWS_ACCESS_KEY_ID" = "YOUR_ACCESS_KEY_HERE" ]; then
    echo -e "${RED}❌ ERROR: AWS credentials not configured!${NC}"
    echo ""
    echo -e "${YELLOW}Please edit this script and update the AWS credentials:${NC}"
    echo "  AWS_ACCESS_KEY_ID"
    echo "  AWS_SECRET_ACCESS_KEY"
    echo "  AWS_SESSION_TOKEN"
    echo ""
    exit 1
fi

echo -e "${GREEN}✓ AWS credentials configured${NC}"
echo ""

# Function to kill all background processes on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}🛑 Shutting down all processes...${NC}"
    kill $(jobs -p) 2>/dev/null
    wait
    echo -e "${GREEN}✓ Shutdown complete${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Step 1: Source ROS workspace
echo -e "${BLUE}[1/4] Setting up ROS workspace...${NC}"
cd "$MAIN_WS_DIR" || exit 1

# Source ROS noetic
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/noetic/setup.zsh 2>/dev/null

# Source workspace (try both bash and zsh)
if [ -f "devel/setup.zsh" ]; then
    source devel/setup.zsh
    echo -e "${GREEN}✓ Sourced workspace (zsh)${NC}"
elif [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo -e "${GREEN}✓ Sourced workspace (bash)${NC}"
else
    echo -e "${RED}❌ ERROR: Workspace not built!${NC}"
    echo "Run: cd $MAIN_WS_DIR && catkin_make"
    exit 1
fi

# Step 2: Launch RViz
echo ""
echo -e "${BLUE}[2/4] Starting RViz...${NC}"
roslaunch ego_planner rviz.launch > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo -e "${GREEN}✓ RViz started (PID: $RVIZ_PID)${NC}"
sleep 3

# Step 3: Launch Swarm Simulation
echo ""
echo -e "${BLUE}[3/4] Starting swarm simulation...${NC}"
roslaunch ego_planner swarm.launch > /tmp/swarm.log 2>&1 &
SWARM_PID=$!
echo -e "${GREEN}✓ Swarm simulation started (PID: $SWARM_PID)${NC}"
sleep 5

# Step 4: Start AWS IoT Streaming
echo ""
echo -e "${BLUE}[4/4] Starting AWS IoT streaming...${NC}"
cd "$SCRIPTS_DIR" || exit 1
python3 swarm_aware_stream.py /drone_0_pcl_render_node/cloud drone_0 &
STREAM_PID=$!
echo -e "${GREEN}✓ Streaming started (PID: $STREAM_PID)${NC}"

# Show status
echo ""
echo -e "${BLUE}======================================================================${NC}"
echo -e "${GREEN}🎉 All systems running!${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""
echo -e "${YELLOW}Running Processes:${NC}"
echo "  • RViz:            PID $RVIZ_PID"
echo "  • Swarm Sim:       PID $SWARM_PID"
echo "  • AWS Streaming:   PID $STREAM_PID"
echo ""
echo -e "${YELLOW}AWS IoT Topics:${NC}"
echo "  • Subscribe to: ${GREEN}drone/telemetry/drone_0${NC} (raw data)"
echo "  • Subscribe to: ${GREEN}drone/swarm/state${NC} (swarm coordination)"
echo ""
echo -e "${YELLOW}Logs:${NC}"
echo "  • RViz:      tail -f /tmp/rviz.log"
echo "  • Swarm:     tail -f /tmp/swarm.log"
echo ""
echo -e "${RED}Press Ctrl+C to stop all processes${NC}"
echo ""

# Wait for all background processes
wait


#!/bin/bash
# Quick script to run the ROS to DynamoDB bridge for real-time streaming

# Set AWS credentials (update with your actual credentials)
export AWS_DEFAULT_REGION="us-east-1"
export AWS_ACCESS_KEY_ID="your_access_key_here"
export AWS_SECRET_ACCESS_KEY="your_secret_key_here"
export AWS_SESSION_TOKEN="your_session_token_here"  # if using temporary credentials

echo "=========================================="
echo "ROS to DynamoDB Real-Time Bridge"
echo "=========================================="
echo ""

# Check if ROS is running
if ! rostopic list &> /dev/null; then
    echo "ERROR: ROS is not running!"
    echo "Please start your ROS system first (roscore + your drone nodes)"
    exit 1
fi

echo "✓ ROS is running"
echo ""

# Get topic and drone ID from command line or use defaults
TOPIC_NAME="${1:-/drone_0_pcl_render_node/cloud}"
DRONE_ID="${2:-drone_0}"

echo "Configuration:"
echo "  Topic: $TOPIC_NAME"
echo "  Drone ID: $DRONE_ID"
echo "  DynamoDB Table: drone_telemetry"
echo ""

# Check if topic exists
if rostopic list | grep -q "^${TOPIC_NAME}$"; then
    echo "✓ Topic $TOPIC_NAME is available"
    echo ""
    
    # Check topic rate
    echo "Checking topic rate..."
    RATE=$(timeout 3s rostopic hz $TOPIC_NAME 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$RATE" ]; then
        echo "✓ Topic is publishing at ~$RATE Hz"
    else
        echo "⚠ Warning: Topic exists but may not be publishing yet"
    fi
else
    echo "⚠ Warning: Topic $TOPIC_NAME not found"
    echo "Available topics:"
    rostopic list
    echo ""
    echo "Continue anyway? The bridge will wait for the topic to appear."
fi

echo ""
echo "Starting bridge..."
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Run the Python bridge directly
cd "$(dirname "$0")"
python3 ros_to_dynamodb_bridge.py \
    _aws_region:=us-east-1 \
    _table_name:=drone_telemetry \
    _drone_id:=$DRONE_ID \
    _topic_name:=$TOPIC_NAME \
    _topic_type:=PointCloud2 \
    _batch_size:=25 \
    _rate_limit:=10


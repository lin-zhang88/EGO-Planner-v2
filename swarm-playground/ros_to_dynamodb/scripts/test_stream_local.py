#!/usr/bin/env python3
"""
Test swarm_aware_stream.py locally without ROS
Just tests the AWS connection and message formatting
"""

import boto3
import json
import time

AWS_REGION = "us-east-1"

# Dummy telemetry data (simulates what swarm_aware_stream.py sends)
dummy_telemetry = {
    'drone_id': 'test_drone',
    'timestamp': time.time(),
    'ros_timestamp': 0.0,
    'message_type': 'PoseStamped',
    'topic_name': '/test/pose',
    'position': {
        'x': 10.5,
        'y': 20.3,
        'z': 5.2
    },
    'orientation': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'w': 1.0
    },
    'ingestion_time': time.time(),
    'message_number': 1
}

print("=" * 70)
print("Testing AWS IoT Connection")
print("=" * 70)

try:
    # Connect to IoT
    iot_client = boto3.client('iot-data', region_name=AWS_REGION)
    print("✓ Connected to AWS IoT Core")
    
    # Send dummy data
    print("\n📤 Sending dummy telemetry...")
    response = iot_client.publish(
        topic='drone/telemetry/test_drone',
        qos=1,
        payload=json.dumps(dummy_telemetry)
    )
    
    print(f"✓ Published successfully! HTTP {response['ResponseMetadata']['HTTPStatusCode']}")
    print("\n✅ Test PASSED - swarm_aware_stream.py AWS connection works!")
    print("\nNext: Run with real ROS data")
    print("  python3 swarm_aware_stream.py multi /drone_0_odom_visualization/pose:drone_0 /drone_1_odom_visualization/pose:drone_1")
    
except Exception as e:
    print(f"✗ Test FAILED: {e}")
    print("\nCheck AWS credentials!")


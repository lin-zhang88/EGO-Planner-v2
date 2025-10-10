#!/usr/bin/env python3
"""
Test Swarm Coordination - Simulate 2 drones and verify Lambda aggregates them
"""

import boto3
import json
import time

AWS_REGION = "us-east-1"

def test_swarm():
    print("=" * 70)
    print("Testing Swarm Coordination")
    print("=" * 70)
    
    try:
        iot_client = boto3.client('iot-data', region_name=AWS_REGION)
        print("✓ Connected to AWS IoT Core\n")
        
        # Send data from drone_0
        print("📤 Sending telemetry from drone_0...")
        drone_0_data = {
            'drone_id': 'drone_0',
            'timestamp': time.time(),
            'message_type': 'PoseStamped',
            'topic_name': '/drone_0_odom_visualization/pose',
            'position': {'x': 10.5, 'y': 20.3, 'z': 5.2},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1.0}
        }
        
        iot_client.publish(
            topic='drone/telemetry/drone_0',
            qos=1,
            payload=json.dumps(drone_0_data)
        )
        print("✓ drone_0 data sent")
        
        # Wait a moment
        time.sleep(1)
        
        # Send data from drone_1
        print("📤 Sending telemetry from drone_1...")
        drone_1_data = {
            'drone_id': 'drone_1',
            'timestamp': time.time(),
            'message_type': 'PoseStamped',
            'topic_name': '/drone_1_odom_visualization/pose',
            'position': {'x': 15.7, 'y': 25.8, 'z': 6.1},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1.0}
        }
        
        iot_client.publish(
            topic='drone/telemetry/drone_1',
            qos=1,
            payload=json.dumps(drone_1_data)
        )
        print("✓ drone_1 data sent")
        
        print("\n" + "=" * 70)
        print("✅ Test data sent!")
        print("=" * 70)
        print("\nNow check in AWS IoT Core:")
        print("1. Subscribe to: drone/swarm/state")
        print("2. You should see a message with:")
        print("   - drone_count: 1 or 2")
        print("   - drones: {drone_0: {...}, drone_1: {...}}")
        print("\nAlso check CloudWatch logs:")
        print("   Lambda → DroneSwarmCoordinator → Monitor → View logs")
        print("   Look for: 'Processing telemetry from drone_0' and 'drone_1'")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")

if __name__ == '__main__':
    test_swarm()


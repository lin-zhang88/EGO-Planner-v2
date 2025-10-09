#!/usr/bin/env python3
"""
Test AWS IoT Greengrass Connection
Sends dummy data to test the setup
"""

import boto3
import json
import time
from decimal import Decimal

# Configuration
AWS_REGION = "us-east-1"
TABLE_NAME = "drone_telemetry"
GREENGRASS_TOPIC = "drone/telemetry"

def test_greengrass_connection():
    """Test sending dummy data to Greengrass"""
    print("=" * 60)
    print("Testing AWS IoT Greengrass Connection")
    print("=" * 60)
    
    try:
        # Initialize clients
        iot_data = boto3.client('iot-data', region_name=AWS_REGION)
        dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)
        table = dynamodb.Table(TABLE_NAME)
        
        print("✓ Connected to AWS IoT Data")
        print("✓ Connected to DynamoDB")
        
        # Create dummy data
        dummy_data = {
            'drone_id': 'test_drone',
            'timestamp': Decimal(str(time.time())),
            'message_type': 'TestMessage',
            'topic_name': '/test/topic',
            'frame_id': 'test_frame',
            'point_count': 100,
            'sample_points': [
                {'x': Decimal('1.0'), 'y': Decimal('2.0'), 'z': Decimal('3.0')},
                {'x': Decimal('4.0'), 'y': Decimal('5.0'), 'z': Decimal('6.0')}
            ],
            'width': 10,
            'height': 10,
            'ingestion_time': Decimal(str(time.time())),
            'message_number': 1,
            'data_type': 'test',
            'is_dense': True,
            'field_count': 3
        }
        
        print("\n📤 Sending dummy data to Greengrass...")
        
        # Convert Decimal to float for JSON
        json_data = convert_decimals_to_floats(dummy_data)
        
        # Send to Greengrass
        response = iot_data.publish(
            topic=f"{GREENGRASS_TOPIC}/test_drone",
            payload=json.dumps(json_data)
        )
        
        print(f"✓ Sent to Greengrass: HTTP {response['ResponseMetadata']['HTTPStatusCode']}")
        
        # Also send directly to DynamoDB as backup
        print("\n📤 Sending to DynamoDB as backup...")
        table.put_item(Item=dummy_data)
        print("✓ Sent to DynamoDB")
        
        print("\n🎉 Test successful! Greengrass is working.")
        print("\nNext steps:")
        print("1. Set up Greengrass Core on your device")
        print("2. Configure Greengrass to forward to DynamoDB")
        print("3. Run: python3 simple_stream.py /drone_0_pcl_render_node/cloud drone_0")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        print("\nTroubleshooting:")
        print("1. Check AWS credentials are set")
        print("2. Verify IoT Core permissions")
        print("3. Ensure DynamoDB table exists")

def convert_decimals_to_floats(obj):
    """Convert Decimal objects to floats for JSON serialization"""
    if isinstance(obj, dict):
        return {k: convert_decimals_to_floats(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_decimals_to_floats(item) for item in obj]
    elif isinstance(obj, Decimal):
        return float(obj)
    else:
        return obj

if __name__ == '__main__':
    test_greengrass_connection()

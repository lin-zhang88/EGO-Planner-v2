#!/usr/bin/env python3
"""
Test Push to DynamoDB
Quick script to test pushing data to DynamoDB from console
"""

import boto3
import time
from decimal import Decimal
from datetime import datetime


def test_push_data():
    """Push test data to DynamoDB"""
    
    # Configuration
    region = 'us-east-1'
    table_name = 'drone_telemetry'
    drone_id = 'drone_7'
    
    print("=" * 60)
    print("Testing DynamoDB Push")
    print("=" * 60)
    print(f"Region: {region}")
    print(f"Table: {table_name}")
    print(f"Drone: {drone_id}")
    print()
    
    try:
        # Connect to DynamoDB
        print("Connecting to DynamoDB...")
        dynamodb = boto3.resource('dynamodb', region_name=region)
        table = dynamodb.Table(table_name)
        print("✓ Connected successfully")
        print()
        
        # Create test data
        timestamp = time.time()
        dt = datetime.fromtimestamp(timestamp)
        
        test_item = {
            'drone_id': drone_id,
            'timestamp': Decimal(str(timestamp)),
            'message_type': 'PointCloud2',
            'topic_name': '/drone_7_ego_planner_node/grid_map/occupancy_inflate',
            'frame_id': 'world',
            'point_count': 1500,
            'sample_points': [
                {'x': Decimal('1.5'), 'y': Decimal('2.3'), 'z': Decimal('0.8')},
                {'x': Decimal('1.6'), 'y': Decimal('2.4'), 'z': Decimal('0.9')},
                {'x': Decimal('1.7'), 'y': Decimal('2.5'), 'z': Decimal('1.0')}
            ],
            'width': 150,
            'height': 10,
            'is_dense': True,
            'ingestion_time': Decimal(str(time.time())),
            'test_data': True  # Mark as test
        }
        
        # Push to DynamoDB
        print(f"Pushing test data...")
        print(f"Timestamp: {dt.strftime('%Y-%m-%d %H:%M:%S.%f')}")
        
        table.put_item(Item=test_item)
        
        print("✓ Data pushed successfully!")
        print()
        
        # Verify by reading back
        print("Verifying data...")
        response = table.get_item(
            Key={
                'drone_id': drone_id,
                'timestamp': Decimal(str(timestamp))
            }
        )
        
        if 'Item' in response:
            print("✓ Data verified in DynamoDB!")
            print()
            print("Item details:")
            item = response['Item']
            print(f"  Drone ID: {item['drone_id']}")
            print(f"  Timestamp: {float(item['timestamp'])}")
            print(f"  Message Type: {item['message_type']}")
            print(f"  Point Count: {item['point_count']}")
            print(f"  Sample Points: {len(item['sample_points'])} points")
            print()
            print("✓ SUCCESS! Your DynamoDB setup is working correctly!")
        else:
            print("✗ Could not verify data")
        
        print()
        print("You can now:")
        print("  1. View data in AWS Console → DynamoDB → drone_telemetry")
        print("  2. Query data: python3 query_data.py")
        print("  3. Start the ROS bridge: roslaunch ros_to_dynamodb ros_to_dynamodb.launch")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        print()
        print("Troubleshooting:")
        print("  1. Check AWS credentials are set:")
        print("     aws sts get-caller-identity")
        print("  2. Verify table exists:")
        print("     aws dynamodb describe-table --table-name drone_telemetry")
        print("  3. Create table if needed:")
        print("     python3 setup_aws.py")
        return False
    
    return True


def push_multiple_samples(count=10):
    """Push multiple test samples"""
    
    region = 'us-east-1'
    table_name = 'drone_telemetry'
    drone_id = 'drone_7'
    
    print(f"Pushing {count} test samples...")
    
    try:
        dynamodb = boto3.resource('dynamodb', region_name=region)
        table = dynamodb.Table(table_name)
        
        with table.batch_writer() as batch:
            for i in range(count):
                timestamp = time.time()
                
                item = {
                    'drone_id': drone_id,
                    'timestamp': Decimal(str(timestamp)),
                    'message_type': 'PointCloud2',
                    'topic_name': '/drone_7_ego_planner_node/grid_map/occupancy_inflate',
                    'frame_id': 'world',
                    'point_count': 1500 + i * 10,
                    'sample_points': [
                        {'x': Decimal(str(1.5 + i*0.1)), 'y': Decimal('2.3'), 'z': Decimal('0.8')},
                        {'x': Decimal(str(1.6 + i*0.1)), 'y': Decimal('2.4'), 'z': Decimal('0.9')}
                    ],
                    'width': 150,
                    'height': 10,
                    'is_dense': True,
                    'ingestion_time': Decimal(str(time.time())),
                    'test_data': True,
                    'sample_number': i + 1
                }
                
                batch.put_item(Item=item)
                
                # Small delay between samples
                time.sleep(0.1)
                
                if (i + 1) % 5 == 0:
                    print(f"  Pushed {i + 1}/{count} samples...")
        
        print(f"✓ Successfully pushed {count} samples!")
        print()
        print("Query them with: python3 query_data.py drone_7 20")
        
    except Exception as e:
        print(f"✗ Error: {e}")


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--multiple':
        count = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        push_multiple_samples(count)
    else:
        test_push_data()


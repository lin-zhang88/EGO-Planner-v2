#!/usr/bin/env python3
"""
Query DynamoDB Data
Example script to query and display drone telemetry from DynamoDB
"""

import boto3
import sys
from decimal import Decimal
from datetime import datetime
from boto3.dynamodb.conditions import Key


def query_drone_data(table_name='drone_telemetry', drone_id='drone_7', region='us-east-1', limit=10):
    """Query the most recent data for a drone"""
    
    try:
        dynamodb = boto3.resource('dynamodb', region_name=region)
        table = dynamodb.Table(table_name)
        
        print(f"Querying DynamoDB table: {table_name}")
        print(f"Drone ID: {drone_id}")
        print(f"Limit: {limit} most recent items")
        print("-" * 80)
        
        response = table.query(
            KeyConditionExpression=Key('drone_id').eq(drone_id),
            ScanIndexForward=False,  # Sort descending (newest first)
            Limit=limit
        )
        
        items = response['Items']
        
        if not items:
            print(f"No data found for {drone_id}")
            return
        
        print(f"Found {len(items)} items:\n")
        
        for i, item in enumerate(items, 1):
            timestamp = float(item['timestamp'])
            dt = datetime.fromtimestamp(timestamp)
            
            print(f"[{i}] Timestamp: {dt.strftime('%Y-%m-%d %H:%M:%S.%f')}")
            print(f"    Message Type: {item.get('message_type', 'Unknown')}")
            print(f"    Topic: {item.get('topic_name', 'Unknown')}")
            
            if item.get('message_type') == 'PointCloud2':
                print(f"    Point Count: {item.get('point_count', 0)}")
                print(f"    Dimensions: {item.get('width', 0)} x {item.get('height', 0)}")
                if 'sample_points' in item and len(item['sample_points']) > 0:
                    sample = item['sample_points'][0]
                    print(f"    First Point: ({sample['x']:.3f}, {sample['y']:.3f}, {sample['z']:.3f})")
            
            elif item.get('message_type') == 'Odometry':
                pos = item.get('position', {})
                vel = item.get('linear_velocity', {})
                print(f"    Position: ({pos.get('x', 0):.3f}, {pos.get('y', 0):.3f}, {pos.get('z', 0):.3f})")
                print(f"    Velocity: ({vel.get('x', 0):.3f}, {vel.get('y', 0):.3f}, {vel.get('z', 0):.3f})")
            
            print()
        
        # Show stats
        print("-" * 80)
        print(f"Total items in query: {len(items)}")
        
        if 'LastEvaluatedKey' in response:
            print("More data available (use pagination to see all)")
        
    except Exception as e:
        print(f"Error querying data: {e}")
        print("\nMake sure:")
        print("  1. AWS credentials are configured")
        print("  2. The table exists")
        print("  3. You have read permissions")


def list_all_drones(table_name='drone_telemetry', region='us-east-1'):
    """List all unique drone IDs in the table"""
    
    try:
        dynamodb = boto3.resource('dynamodb', region_name=region)
        table = dynamodb.Table(table_name)
        
        print(f"Scanning table: {table_name}")
        print("-" * 80)
        
        response = table.scan(
            ProjectionExpression='drone_id',
            Select='SPECIFIC_ATTRIBUTES'
        )
        
        drone_ids = set()
        for item in response['Items']:
            drone_ids.add(item['drone_id'])
        
        # Handle pagination
        while 'LastEvaluatedKey' in response:
            response = table.scan(
                ProjectionExpression='drone_id',
                Select='SPECIFIC_ATTRIBUTES',
                ExclusiveStartKey=response['LastEvaluatedKey']
            )
            for item in response['Items']:
                drone_ids.add(item['drone_id'])
        
        print(f"Found {len(drone_ids)} unique drone(s):")
        for drone_id in sorted(drone_ids):
            print(f"  - {drone_id}")
        
    except Exception as e:
        print(f"Error scanning table: {e}")


def delete_drone_data(table_name='drone_telemetry', drone_id='drone_7', region='us-east-1'):
    """Delete all data for a specific drone (use with caution!)"""
    
    confirm = input(f"Are you sure you want to delete ALL data for {drone_id}? (yes/no): ")
    if confirm.lower() != 'yes':
        print("Cancelled.")
        return
    
    try:
        dynamodb = boto3.resource('dynamodb', region_name=region)
        table = dynamodb.Table(table_name)
        
        print(f"Deleting data for {drone_id}...")
        
        response = table.query(
            KeyConditionExpression=Key('drone_id').eq(drone_id)
        )
        
        with table.batch_writer() as batch:
            for item in response['Items']:
                batch.delete_item(
                    Key={
                        'drone_id': item['drone_id'],
                        'timestamp': item['timestamp']
                    }
                )
        
        print(f"Deleted {len(response['Items'])} items")
        
    except Exception as e:
        print(f"Error deleting data: {e}")


if __name__ == '__main__':
    print("=" * 80)
    print("DynamoDB Query Tool for Drone Telemetry")
    print("=" * 80)
    print()
    
    if len(sys.argv) > 1 and sys.argv[1] == '--list-drones':
        list_all_drones()
    elif len(sys.argv) > 1 and sys.argv[1] == '--delete':
        drone_id = sys.argv[2] if len(sys.argv) > 2 else 'drone_7'
        delete_drone_data(drone_id=drone_id)
    else:
        drone_id = sys.argv[1] if len(sys.argv) > 1 else 'drone_7'
        limit = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        query_drone_data(drone_id=drone_id, limit=limit)
        
        print("\nUsage examples:")
        print("  python3 query_data.py                  # Query drone_7 (latest 10)")
        print("  python3 query_data.py drone_0 20       # Query drone_0 (latest 20)")
        print("  python3 query_data.py --list-drones    # List all drones")
        print("  python3 query_data.py --delete drone_7 # Delete all drone_7 data")


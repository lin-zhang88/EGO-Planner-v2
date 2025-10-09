#!/usr/bin/env python3
"""
Simple Real-Time ROS to AWS IoT Greengrass Streamer
Uses Greengrass as mediator to DynamoDB for offline resilience

Usage:
    python3 simple_stream.py /drone_0_pcl_render_node/cloud drone_0
"""

import rospy
import boto3
import sys
import time
import json
import os
from decimal import Decimal
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Configuration
AWS_REGION = "us-east-1"
TABLE_NAME = "drone_telemetry"
GREENGRASS_GROUP_ID = "drone_telemetry_group"
GREENGRASS_TOPIC = "drone/telemetry"

class SimpleStreamer:
    def __init__(self, topic_name, drone_id):
        self.topic_name = topic_name
        self.drone_id = drone_id
        self.message_count = 0
        self.batch_count = 0
        self.offline_buffer = []
        
        # Connect to AWS IoT Greengrass
        print("=" * 60)
        print("Real-Time ROS → AWS IoT Greengrass → DynamoDB Streamer")
        print("=" * 60)
        print(f"Topic: {topic_name}")
        print(f"Drone: {drone_id}")
        print(f"Greengrass Group: {GREENGRASS_GROUP_ID}")
        print(f"Greengrass Topic: {GREENGRASS_TOPIC}")
        print()
        
        try:
            # Initialize Greengrass client
            self.greengrass = boto3.client('greengrass', region_name=AWS_REGION)
            self.iot_data = boto3.client('iot-data', region_name=AWS_REGION)
            
            # Also keep DynamoDB connection for direct fallback
            self.dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)
            self.table = self.dynamodb.Table(TABLE_NAME)
            
            print("✓ Connected to AWS IoT Greengrass")
            print("✓ Connected to DynamoDB (fallback)")
            
        except Exception as e:
            print(f"✗ Failed to connect to AWS: {e}")
            print("\nMake sure AWS credentials are set:")
            print("  export AWS_ACCESS_KEY_ID=your_key")
            print("  export AWS_SECRET_ACCESS_KEY=your_secret")
            print("  export AWS_SESSION_TOKEN=your_token")
            sys.exit(1)
        
        # Initialize ROS
        rospy.init_node('simple_dynamodb_streamer', anonymous=True)
        print("✓ ROS node initialized")
        
        # Subscribe to topic
        self.subscriber = rospy.Subscriber(
            topic_name,
            PointCloud2,
            self.callback,
            queue_size=10
        )
        print(f"✓ Subscribed to {topic_name}")
        print()
        print("Streaming data... (Press Ctrl+C to stop)")
        print("-" * 60)
    
    def callback(self, msg):
        """Process incoming messages"""
        try:
            self.message_count += 1
            
            # Fix timestamp issue - use current time if ROS timestamp is invalid
            ros_timestamp = msg.header.stamp.to_sec()
            if ros_timestamp == 0.0:
                timestamp = time.time()  # Use current time instead
                print(f"[{self.message_count}] Using current time (ROS timestamp was 0)")
            else:
                timestamp = ros_timestamp
            
            # Extract basic info
            point_count = msg.width * msg.height
            
            # Sample a few points (not all - too big!)
            sample_points = []
            for i, point in enumerate(pc2.read_points(msg, skip_nans=True)):
                if i >= 10:  # Just sample 10 points
                    break
                sample_points.append({
                    'x': Decimal(str(float(point[0]))),
                    'y': Decimal(str(float(point[1]))),
                    'z': Decimal(str(float(point[2])))
                })
            
            # Create item
            item = {
                'drone_id': self.drone_id,
                'timestamp': Decimal(str(timestamp)),
                'ros_timestamp': Decimal(str(ros_timestamp)),  # Keep original for debugging
                'message_type': 'PointCloud2',
                'topic_name': self.topic_name,
                'frame_id': msg.header.frame_id,
                'point_count': point_count,
                'sample_points': sample_points,
                'width': msg.width,
                'height': msg.height,
                'ingestion_time': Decimal(str(time.time())),
                'message_number': self.message_count,
                'data_type': msg.data_type if hasattr(msg, 'data_type') else 'unknown',
                'is_dense': msg.is_dense,
                'field_count': len(msg.fields) if hasattr(msg, 'fields') else 0
            }
            
            # Send to Greengrass (with offline fallback)
            self.send_to_greengrass(item)
            self.batch_count += 1
            
            # Print status every 10 messages
            if self.message_count % 10 == 0:
                print(f"[{self.message_count}] Sent to Greengrass | Points: {point_count} | Frame: {msg.header.frame_id}")
            
        except Exception as e:
            print(f"✗ Error: {e}")
    
    def send_to_greengrass(self, item):
        """Send data to Greengrass with offline fallback"""
        try:
            # Convert Decimal to float for JSON serialization
            json_item = self.convert_decimals_to_floats(item)
            
            # Try to send to Greengrass first
            try:
                response = self.iot_data.publish(
                    topic=f"{GREENGRASS_TOPIC}/{self.drone_id}",
                    payload=json.dumps(json_item)
                )
                print(f"✓ Sent to Greengrass: {response['ResponseMetadata']['HTTPStatusCode']}")
                
                # If we have buffered data, try to send it
                if self.offline_buffer:
                    self.flush_offline_buffer()
                    
            except Exception as greengrass_error:
                print(f"⚠ Greengrass offline, buffering data: {greengrass_error}")
                self.offline_buffer.append(json_item)
                
                # If buffer is getting too large, send directly to DynamoDB
                if len(self.offline_buffer) > 50:
                    print("⚠ Buffer full, sending directly to DynamoDB")
                    self.send_direct_to_dynamodb(item)
                    self.offline_buffer = []
                    
        except Exception as e:
            print(f"✗ Error sending to Greengrass: {e}")
            # Fallback to direct DynamoDB
            self.send_direct_to_dynamodb(item)
    
    def send_direct_to_dynamodb(self, item):
        """Direct fallback to DynamoDB"""
        try:
            self.table.put_item(Item=item)
            print("✓ Sent directly to DynamoDB (fallback)")
        except Exception as e:
            print(f"✗ DynamoDB fallback failed: {e}")
    
    def flush_offline_buffer(self):
        """Send buffered data when connection is restored"""
        if not self.offline_buffer:
            return
            
        print(f"📤 Flushing {len(self.offline_buffer)} buffered messages...")
        for buffered_item in self.offline_buffer:
            try:
                response = self.iot_data.publish(
                    topic=f"{GREENGRASS_TOPIC}/{self.drone_id}",
                    payload=json.dumps(buffered_item)
                )
            except Exception as e:
                print(f"✗ Failed to flush buffered item: {e}")
                break
        
        self.offline_buffer = []
        print("✓ Offline buffer flushed")
    
    def convert_decimals_to_floats(self, obj):
        """Convert Decimal objects to floats for JSON serialization"""
        if isinstance(obj, dict):
            return {k: self.convert_decimals_to_floats(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self.convert_decimals_to_floats(item) for item in obj]
        elif isinstance(obj, Decimal):
            return float(obj)
        else:
            return obj
    
    def run(self):
        """Keep running"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print()
            print("-" * 60)
            print(f"✓ Streamed {self.message_count} messages to Greengrass")
            if self.offline_buffer:
                print(f"⚠ {len(self.offline_buffer)} messages still in buffer")
                print("📤 Flushing remaining buffer...")
                self.flush_offline_buffer()
            print("✓ Done!")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 simple_stream.py <topic_name> [drone_id]")
        print()
        print("Examples:")
        print("  python3 simple_stream.py /drone_0_pcl_render_node/cloud drone_0")
        print("  python3 simple_stream.py /drone_7_ego_planner_node/grid_map/occupancy_inflate drone_7")
        sys.exit(1)
    
    topic = sys.argv[1]
    drone_id = sys.argv[2] if len(sys.argv) > 2 else "drone_0"
    
    streamer = SimpleStreamer(topic, drone_id)
    streamer.run()


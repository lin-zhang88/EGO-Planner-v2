#!/usr/bin/env python3
"""
Simple Real-Time ROS to DynamoDB Streamer
Easy standalone script - just run it!

Usage:
    python3 simple_stream.py /drone_0_pcl_render_node/cloud drone_0
"""

import rospy
import boto3
import sys
import time
from decimal import Decimal
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Configuration
AWS_REGION = "us-east-1"
TABLE_NAME = "drone_telemetry"

class SimpleStreamer:
    def __init__(self, topic_name, drone_id):
        self.topic_name = topic_name
        self.drone_id = drone_id
        self.message_count = 0
        self.batch_count = 0
        
        # Connect to DynamoDB
        print("=" * 60)
        print("Real-Time ROS → DynamoDB Streamer")
        print("=" * 60)
        print(f"Topic: {topic_name}")
        print(f"Drone: {drone_id}")
        print(f"Table: {TABLE_NAME}")
        print()
        
        try:
            self.dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)
            self.table = self.dynamodb.Table(TABLE_NAME)
            print("✓ Connected to DynamoDB")
        except Exception as e:
            print(f"✗ Failed to connect to DynamoDB: {e}")
            print("\nMake sure AWS credentials are set:")
            print("  export AWS_ACCESS_KEY_ID=your_key")
            print("  export AWS_SECRET_ACCESS_KEY=your_secret")
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
            
            # Send to DynamoDB
            self.table.put_item(Item=item)
            self.batch_count += 1
            
            # Print status every 10 messages
            if self.message_count % 10 == 0:
                print(f"[{self.message_count}] Sent to DynamoDB | Points: {point_count} | Frame: {msg.header.frame_id}")
            
        except Exception as e:
            print(f"✗ Error: {e}")
    
    def run(self):
        """Keep running"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print()
            print("-" * 60)
            print(f"✓ Streamed {self.message_count} messages to DynamoDB")
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


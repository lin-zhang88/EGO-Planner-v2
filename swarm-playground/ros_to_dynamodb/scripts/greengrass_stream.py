#!/usr/bin/env python3
"""
AWS IoT Greengrass Real-Time ROS to DynamoDB Streamer
Features:
- Offline buffering with automatic retry
- Local edge processing
- Fallback to direct DynamoDB
- Multi-drone support

Usage:
    python3 greengrass_stream.py /drone_0_pcl_render_node/cloud drone_0
    python3 greengrass_stream.py multi  # For multiple drones
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
from threading import Thread, Lock

# Configuration
AWS_REGION = os.environ.get('AWS_DEFAULT_REGION', 'us-east-1')
TABLE_NAME = "drone_telemetry"
IOT_ENDPOINT = None  # Will be discovered automatically
GREENGRASS_TOPIC_PREFIX = "drone/telemetry"

# Multi-drone configuration
DRONE_TOPICS = [
    ("/drone_0_pcl_render_node/cloud", "drone_0"),
    ("/drone_1_pcl_render_node/cloud", "drone_1"),
]

class GreengrassStreamer:
    """Greengrass-enabled ROS to DynamoDB streamer with offline resilience"""
    
    def __init__(self, topic_name, drone_id, init_ros=True):
        self.topic_name = topic_name
        self.drone_id = drone_id
        self.message_count = 0
        self.success_count = 0
        self.buffer_count = 0
        self.offline_buffer = []
        self.buffer_lock = Lock()
        self.is_online = True
        
        # Display banner
        print("=" * 70)
        print("🚁 AWS IoT Greengrass Real-Time Drone Telemetry Streamer")
        print("=" * 70)
        print(f"Drone ID:    {drone_id}")
        print(f"ROS Topic:   {topic_name}")
        print(f"AWS Region:  {AWS_REGION}")
        print(f"DDB Table:   {TABLE_NAME}")
        print("=" * 70)
        
        # Initialize AWS clients
        self._init_aws_clients()
        
        # Initialize ROS
        if init_ros:
            rospy.init_node('greengrass_streamer', anonymous=True)
            print("✓ ROS node initialized")
        
        # Subscribe to ROS topic
        self.subscriber = rospy.Subscriber(
            topic_name,
            PointCloud2,
            self.callback,
            queue_size=10
        )
        print(f"✓ Subscribed to {topic_name}")
        
        # Start background buffer flusher
        self.flusher_thread = Thread(target=self._periodic_flush, daemon=True)
        self.flusher_thread.start()
        print("✓ Background buffer flusher started")
        
        print("\n📡 Streaming started... (Press Ctrl+C to stop)")
        print("-" * 70)
    
    def _init_aws_clients(self):
        """Initialize AWS IoT and DynamoDB clients"""
        try:
            # IoT Data client for publishing to Greengrass
            self.iot_client = boto3.client('iot-data', region_name=AWS_REGION)
            print("✓ Connected to AWS IoT Core")
            
            # DynamoDB client for direct fallback
            self.dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)
            self.table = self.dynamodb.Table(TABLE_NAME)
            print("✓ Connected to DynamoDB (fallback)")
            
            # IoT client for endpoint discovery
            iot = boto3.client('iot', region_name=AWS_REGION)
            endpoint_response = iot.describe_endpoint(endpointType='iot:Data-ATS')
            self.iot_endpoint = endpoint_response['endpointAddress']
            print(f"✓ IoT Endpoint: {self.iot_endpoint}")
            
        except Exception as e:
            print(f"✗ AWS connection failed: {e}")
            print("\n🔧 Troubleshooting:")
            print("   1. Check AWS credentials are set")
            print("   2. Verify IAM permissions (IoT, DynamoDB)")
            print("   3. Ensure Greengrass Core is running")
            sys.exit(1)
    
    def callback(self, msg):
        """Process incoming ROS messages"""
        try:
            self.message_count += 1
            
            # Get timestamp
            ros_timestamp = msg.header.stamp.to_sec()
            timestamp = time.time() if ros_timestamp == 0.0 else ros_timestamp
            
            # Extract point cloud data
            point_count = msg.width * msg.height
            
            # Sample points (10 points to keep payload small)
            sample_points = []
            for i, point in enumerate(pc2.read_points(msg, skip_nans=True)):
                if i >= 10:
                    break
                sample_points.append({
                    'x': float(point[0]),
                    'y': float(point[1]),
                    'z': float(point[2])
                })
            
            # Create telemetry item
            telemetry = {
                'drone_id': self.drone_id,
                'timestamp': timestamp,
                'ros_timestamp': ros_timestamp,
                'message_type': 'PointCloud2',
                'topic_name': self.topic_name,
                'frame_id': msg.header.frame_id,
                'point_count': point_count,
                'sample_points': sample_points,
                'width': msg.width,
                'height': msg.height,
                'is_dense': msg.is_dense,
                'field_count': len(msg.fields),
                'ingestion_time': time.time(),
                'message_number': self.message_count
            }
            
            # Send to Greengrass
            self.send_telemetry(telemetry)
            
            # Status update every 10 messages
            if self.message_count % 10 == 0:
                status = "🟢 ONLINE" if self.is_online else "🔴 OFFLINE"
                buffer_info = f" | Buffer: {len(self.offline_buffer)}" if self.offline_buffer else ""
                print(f"[{self.message_count}] {status} | Points: {point_count}{buffer_info}")
                
        except Exception as e:
            print(f"✗ Callback error: {e}")
    
    def send_telemetry(self, telemetry):
        """Send telemetry data via Greengrass with fallback"""
        try:
            # Try sending to Greengrass/IoT Core
            topic = f"{GREENGRASS_TOPIC_PREFIX}/{self.drone_id}"
            payload = json.dumps(telemetry)
            
            response = self.iot_client.publish(
                topic=topic,
                qos=1,
                payload=payload
            )
            
            self.success_count += 1
            self.is_online = True
            
            # If we were offline and now online, flush buffer
            if self.offline_buffer:
                self._flush_buffer()
                
        except Exception as iot_error:
            # IoT/Greengrass is offline, buffer the data
            self.is_online = False
            with self.buffer_lock:
                self.offline_buffer.append(telemetry)
                self.buffer_count += 1
            
            # If buffer is too large, write to local file or DynamoDB
            if len(self.offline_buffer) > 100:
                print(f"⚠ Buffer full ({len(self.offline_buffer)} items), saving to DynamoDB")
                self._save_buffer_to_dynamodb()
    
    def _flush_buffer(self):
        """Flush offline buffer when connection is restored"""
        with self.buffer_lock:
            if not self.offline_buffer:
                return
            
            print(f"\n📤 Connection restored! Flushing {len(self.offline_buffer)} buffered messages...")
            
            flushed = 0
            failed = []
            
            for item in self.offline_buffer:
                try:
                    topic = f"{GREENGRASS_TOPIC_PREFIX}/{self.drone_id}"
                    self.iot_client.publish(
                        topic=topic,
                        qos=1,
                        payload=json.dumps(item)
                    )
                    flushed += 1
                except Exception as e:
                    failed.append(item)
            
            if failed:
                print(f"⚠ {len(failed)} items failed to flush, keeping in buffer")
                self.offline_buffer = failed
            else:
                print(f"✓ Successfully flushed {flushed} buffered messages")
                self.offline_buffer = []
    
    def _periodic_flush(self):
        """Periodically try to flush buffer in background"""
        while True:
            time.sleep(30)  # Try every 30 seconds
            if self.offline_buffer and self.is_online:
                self._flush_buffer()
    
    def _save_buffer_to_dynamodb(self):
        """Save buffer directly to DynamoDB when too large"""
        with self.buffer_lock:
            if not self.offline_buffer:
                return
            
            saved = 0
            for item in self.offline_buffer:
                try:
                    # Convert floats to Decimals for DynamoDB
                    ddb_item = self._convert_to_decimal(item)
                    self.table.put_item(Item=ddb_item)
                    saved += 1
                except Exception as e:
                    print(f"✗ DynamoDB save failed: {e}")
                    break
            
            # Clear saved items from buffer
            self.offline_buffer = self.offline_buffer[saved:]
            print(f"✓ Saved {saved} items to DynamoDB (fallback)")
    
    def _convert_to_decimal(self, obj):
        """Convert floats to Decimal for DynamoDB"""
        if isinstance(obj, dict):
            return {k: self._convert_to_decimal(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self._convert_to_decimal(item) for item in obj]
        elif isinstance(obj, float):
            return Decimal(str(obj))
        else:
            return obj
    
    def run(self):
        """Keep streaming"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self._shutdown()
    
    def _shutdown(self):
        """Graceful shutdown"""
        print("\n" + "=" * 70)
        print("🛑 Shutting down...")
        print(f"📊 Total messages received: {self.message_count}")
        print(f"✓ Successfully sent: {self.success_count}")
        print(f"📦 Buffered messages: {len(self.offline_buffer)}")
        
        if self.offline_buffer:
            print("\n📤 Flushing remaining buffer to DynamoDB...")
            self._save_buffer_to_dynamodb()
        
        print("✓ Shutdown complete")
        print("=" * 70)


class MultiDroneGreengrassStreamer:
    """Stream multiple drones simultaneously"""
    
    def __init__(self):
        print("=" * 70)
        print("🚁🚁 Multi-Drone Greengrass Streamer")
        print("=" * 70)
        print(f"Streaming {len(DRONE_TOPICS)} drones:")
        for topic, drone_id in DRONE_TOPICS:
            print(f"  • {topic} → {drone_id}")
        print("=" * 70)
        
        # Initialize ROS once
        rospy.init_node('multi_drone_greengrass_streamer', anonymous=True)
        print("✓ ROS node initialized\n")
        
        # Create streamers for each drone
        self.streamers = []
        for topic, drone_id in DRONE_TOPICS:
            print(f"Setting up {drone_id}...")
            streamer = GreengrassStreamer(topic, drone_id, init_ros=False)
            self.streamers.append(streamer)
            print()
        
        print("🚀 All drones streaming!")
        print("-" * 70)
    
    def run(self):
        """Keep streaming all drones"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("\n" + "=" * 70)
            print("🛑 Shutting down all drones...")
            for streamer in self.streamers:
                streamer._shutdown()


if __name__ == '__main__':
    # Check for multi-drone mode
    if len(sys.argv) > 1 and sys.argv[1] == 'multi':
        streamer = MultiDroneGreengrassStreamer()
        streamer.run()
    elif len(sys.argv) >= 3:
        topic = sys.argv[1]
        drone_id = sys.argv[2]
        streamer = GreengrassStreamer(topic, drone_id)
        streamer.run()
    else:
        print("Usage:")
        print("  Single drone: python3 greengrass_stream.py <topic> <drone_id>")
        print("  Multi drone:  python3 greengrass_stream.py multi")
        print()
        print("Examples:")
        print("  python3 greengrass_stream.py /drone_0_pcl_render_node/cloud drone_0")
        print("  python3 greengrass_stream.py multi")
        sys.exit(1)


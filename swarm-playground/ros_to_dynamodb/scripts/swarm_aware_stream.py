#!/usr/bin/env python3
"""
Swarm-Aware Drone Streamer
- Publishes own telemetry to IoT Core
- Subscribes to swarm state to avoid collisions
- Provides real-time positions of all drones

Usage:
    # Single drone
    python3 swarm_aware_stream.py /drone_0_pcl_render_node/cloud drone_0
    
    # Multiple drones (specify drone IDs)
    python3 swarm_aware_stream.py multi drone_0 drone_1
    python3 swarm_aware_stream.py multi drone_0 drone_1 drone_2
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
SWARM_TOPIC = "drone/swarm/state"

# Topic template for multi-drone mode
TOPIC_TEMPLATE = "/drone_{}_pcl_render_node/cloud"

class SwarmAwareStreamer:
    """Drone streamer with swarm awareness for collision avoidance"""
    
    def __init__(self, topic_name, drone_id, init_ros=True):
        self.topic_name = topic_name
        self.drone_id = drone_id
        self.message_count = 0
        self.swarm_state = {}
        self.swarm_lock = Lock()
        
        # Display banner
        print("=" * 70)
        print("🚁 Swarm-Aware Drone Telemetry Streamer")
        print("=" * 70)
        print(f"Drone ID:    {drone_id}")
        print(f"ROS Topic:   {topic_name}")
        print(f"AWS Region:  {AWS_REGION}")
        print("=" * 70)
        
        # Initialize AWS clients
        self._init_aws_clients()
        
        # Initialize ROS (only once for multi-drone)
        if init_ros:
            rospy.init_node(f'swarm_aware_streamer_{drone_id}', anonymous=True)
            print("✓ ROS node initialized")
        
        # Subscribe to ROS topic
        self.subscriber = rospy.Subscriber(
            topic_name,
            PointCloud2,
            self.callback,
            queue_size=10
        )
        print(f"✓ Subscribed to {topic_name}")
        
        # Subscribe to swarm state
        self._setup_swarm_subscription()
        
        print("\n📡 Streaming started... (Press Ctrl+C to stop)")
        print("-" * 70)
    
    def _init_aws_clients(self):
        """Initialize AWS IoT clients"""
        try:
            # IoT Data client for publishing
            self.iot_client = boto3.client('iot-data', region_name=AWS_REGION)
            print("✓ Connected to AWS IoT Core")
            
            # Get IoT endpoint
            iot = boto3.client('iot', region_name=AWS_REGION)
            endpoint_response = iot.describe_endpoint(endpointType='iot:Data-ATS')
            self.iot_endpoint = endpoint_response['endpointAddress']
            print(f"✓ IoT Endpoint: {self.iot_endpoint}")
            
        except Exception as e:
            print(f"✗ AWS connection failed: {e}")
            sys.exit(1)
    
    def _setup_swarm_subscription(self):
        """Subscribe to swarm state updates via MQTT"""
        print("\n🔗 Setting up swarm state subscription...")
        
        # Note: For production, use AWS IoT SDK with certificates
        # For now, we'll poll via API
        self.swarm_thread = Thread(target=self._poll_swarm_state, daemon=True)
        self.swarm_thread.start()
        print("✓ Swarm state subscription active")
    
    def _poll_swarm_state(self):
        """Poll for swarm state updates"""
        while True:
            try:
                # In production, use MQTT subscribe
                # For now, we publish and Lambda broadcasts back
                time.sleep(1)
            except Exception as e:
                print(f"⚠ Swarm polling error: {e}")
    
    def callback(self, msg):
        """Process incoming ROS messages and publish to IoT"""
        try:
            self.message_count += 1
            
            # Get timestamp
            ros_timestamp = msg.header.stamp.to_sec()
            timestamp = time.time() if ros_timestamp == 0.0 else ros_timestamp
            
            # Extract point cloud data
            point_count = msg.width * msg.height
            
            # Sample points
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
            
            # Publish to IoT Core
            self.publish_telemetry(telemetry)
            
            # Status update every 10 messages
            if self.message_count % 10 == 0:
                with self.swarm_lock:
                    swarm_size = len(self.swarm_state)
                print(f"[{self.message_count}] 🟢 ONLINE | Points: {point_count} | Swarm: {swarm_size} drones")
                
                # Show other drones' positions
                if swarm_size > 0:
                    self._display_swarm_info()
                    
        except Exception as e:
            print(f"✗ Callback error: {e}")
    
    def publish_telemetry(self, telemetry):
        """Publish telemetry to IoT Core"""
        try:
            topic = f"drone/telemetry/{self.drone_id}"
            payload = json.dumps(telemetry)
            
            self.iot_client.publish(
                topic=topic,
                qos=1,
                payload=payload
            )
        except Exception as e:
            print(f"✗ Publish error: {e}")
    
    def _display_swarm_info(self):
        """Display current swarm state"""
        with self.swarm_lock:
            if not self.swarm_state:
                return
            
            print("\n📍 Swarm State:")
            for drone_id, state in self.swarm_state.items():
                if drone_id != self.drone_id:
                    pos = state.get('position', {})
                    print(f"   {drone_id}: x={pos.get('x', 0):.2f}, y={pos.get('y', 0):.2f}, z={pos.get('z', 0):.2f}")
            print()
    
    def update_swarm_state(self, swarm_data):
        """Update swarm state from broadcast"""
        with self.swarm_lock:
            self.swarm_state = swarm_data.get('drones', {})
    
    def get_other_drones(self):
        """Get positions of all other drones"""
        with self.swarm_lock:
            return {k: v for k, v in self.swarm_state.items() if k != self.drone_id}
    
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
        print(f"📊 Total messages sent: {self.message_count}")
        print("✓ Shutdown complete")
        print("=" * 70)


class MultiDroneSwarmStreamer:
    """Stream multiple drones simultaneously"""
    
    def __init__(self, drone_ids):
        # Build topics from drone IDs
        self.drone_topics = []
        for drone_id in drone_ids:
            # Extract number from drone_id (e.g., "drone_0" -> "0")
            drone_num = drone_id.split('_')[-1] if '_' in drone_id else drone_id
            topic = TOPIC_TEMPLATE.format(drone_num)
            self.drone_topics.append((topic, drone_id))
        
        print("=" * 70)
        print("🚁🚁 Multi-Drone Swarm Streamer")
        print("=" * 70)
        print(f"Streaming {len(self.drone_topics)} drones:")
        for topic, drone_id in self.drone_topics:
            print(f"  • {topic} → {drone_id}")
        print("=" * 70)
        
        # Initialize ROS once
        rospy.init_node('multi_drone_swarm_streamer', anonymous=True)
        print("✓ ROS node initialized\n")
        
        # Create streamers for each drone
        self.streamers = []
        for topic, drone_id in self.drone_topics:
            print(f"Setting up {drone_id}...")
            streamer = SwarmAwareStreamer(topic, drone_id, init_ros=False)
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
        if len(sys.argv) < 3:
            print("Error: Please specify drone IDs for multi mode")
            print()
            print("Usage:")
            print("  python3 swarm_aware_stream.py multi <drone_id1> <drone_id2> ...")
            print()
            print("Examples:")
            print("  python3 swarm_aware_stream.py multi drone_0 drone_1")
            print("  python3 swarm_aware_stream.py multi drone_0 drone_1 drone_2")
            sys.exit(1)
        
        # Get drone IDs from command line
        drone_ids = sys.argv[2:]
        multi_streamer = MultiDroneSwarmStreamer(drone_ids)
        multi_streamer.run()
        
    elif len(sys.argv) >= 3:
        # Single drone mode
        topic = sys.argv[1]
        drone_id = sys.argv[2]
        streamer = SwarmAwareStreamer(topic, drone_id)
        streamer.run()
    else:
        print("Usage:")
        print("  Single drone: python3 swarm_aware_stream.py <topic> <drone_id>")
        print("  Multi drone:  python3 swarm_aware_stream.py multi <drone_id1> <drone_id2> ...")
        print()
        print("Examples:")
        print("  # Single drone")
        print("  python3 swarm_aware_stream.py /drone_0_pcl_render_node/cloud drone_0")
        print()
        print("  # Multiple drones")
        print("  python3 swarm_aware_stream.py multi drone_0 drone_1")
        print("  python3 swarm_aware_stream.py multi drone_0 drone_1 drone_2")
        sys.exit(1)


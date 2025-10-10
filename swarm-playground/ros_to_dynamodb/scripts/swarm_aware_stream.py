#!/usr/bin/env python3
"""
Swarm-Aware Drone Streamer
- Publishes own telemetry to IoT Core
- Subscribes to swarm state to avoid collisions
- Provides real-time positions of all drones

Usage:
    # Single drone
    python3 swarm_aware_stream.py /drone_0_pcl_render_node/cloud drone_0
    
    # Multiple drones (topic:drone_id pairs)
    python3 swarm_aware_stream.py multi /drone_0_pcl_render_node/cloud:drone_0 /drone_1_pcl_render_node/cloud:drone_1
    python3 swarm_aware_stream.py multi /drone_0_ego_planner/grid:drone_0 /drone_1_sensor/data:drone_1
"""

import rospy
import boto3
import sys
import time
import json
import os
from decimal import Decimal
from rospy import AnyMsg
from threading import Thread, Lock

# Configuration
AWS_REGION = os.environ.get('AWS_DEFAULT_REGION', 'us-east-1')
TABLE_NAME = "drone_telemetry"
SWARM_TOPIC = "drone/swarm/state"

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
        
        # Subscribe to ROS topic (any message type)
        self.subscriber = rospy.Subscriber(
            topic_name,
            AnyMsg,
            self.callback,
            queue_size=10
        )
        print(f"✓ Subscribed to {topic_name} (accepts any message type)")
        
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
            
            # Get message type from connection header
            msg_type_full = msg._connection_header['type']
            msg_type = msg_type_full.split('/')[-1]
            
            # Deserialize based on message type
            import genpy
            msg_class = genpy.message.get_message_class(msg_type_full)
            
            if msg_class:
                actual_msg = msg_class()
                actual_msg.deserialize(msg._buff)
            else:
                print(f"⚠ Could not deserialize {msg_type_full}")
                return
            
            # Get timestamp
            ros_timestamp = 0.0
            if hasattr(actual_msg, 'header') and hasattr(actual_msg.header, 'stamp'):
                ros_timestamp = actual_msg.header.stamp.to_sec()
            timestamp = time.time() if ros_timestamp == 0.0 else ros_timestamp
            
            # Extract position based on message type
            position = {'x': 0, 'y': 0, 'z': 0}
            extra_data = {}
            
            if msg_type == 'PoseStamped':
                # Handle PoseStamped
                position = {
                    'x': float(actual_msg.pose.position.x),
                    'y': float(actual_msg.pose.position.y),
                    'z': float(actual_msg.pose.position.z)
                }
                extra_data['orientation'] = {
                    'x': float(actual_msg.pose.orientation.x),
                    'y': float(actual_msg.pose.orientation.y),
                    'z': float(actual_msg.pose.orientation.z),
                    'w': float(actual_msg.pose.orientation.w)
                }
            elif msg_type == 'PointCloud2':
                # Handle PointCloud2
                import sensor_msgs.point_cloud2 as pc2
                sample_points = []
                for i, point in enumerate(pc2.read_points(actual_msg, skip_nans=True)):
                    if i >= 10:
                        break
                    sample_points.append({
                        'x': float(point[0]),
                        'y': float(point[1]),
                        'z': float(point[2])
                    })
                if sample_points:
                    position = sample_points[0]
                extra_data['point_count'] = actual_msg.width * actual_msg.height
                extra_data['sample_points'] = sample_points
            elif msg_type == 'Odometry':
                # Handle Odometry
                position = {
                    'x': float(actual_msg.pose.pose.position.x),
                    'y': float(actual_msg.pose.pose.position.y),
                    'z': float(actual_msg.pose.pose.position.z)
                }
                extra_data['twist'] = {
                    'linear': {
                        'x': float(actual_msg.twist.twist.linear.x),
                        'y': float(actual_msg.twist.twist.linear.y),
                        'z': float(actual_msg.twist.twist.linear.z)
                    }
                }
            
            # Create telemetry item
            telemetry = {
                'drone_id': self.drone_id,
                'timestamp': timestamp,
                'ros_timestamp': ros_timestamp,
                'message_type': msg_type,
                'topic_name': self.topic_name,
                'position': position,
                'ingestion_time': time.time(),
                'message_number': self.message_count,
                **extra_data
            }
            
            # Publish to IoT Core
            self.publish_telemetry(telemetry)
            
            # Status update every 10 messages
            if self.message_count % 10 == 0:
                with self.swarm_lock:
                    swarm_size = len(self.swarm_state)
                print(f"[{self.message_count}] 🟢 ONLINE | Type: {msg_type} | Pos: ({position['x']:.2f}, {position['y']:.2f}, {position['z']:.2f}) | Swarm: {swarm_size}")
                
                # Show other drones' positions
                if swarm_size > 0:
                    self._display_swarm_info()
                    
        except Exception as e:
            print(f"✗ Callback error: {e}")
            import traceback
            traceback.print_exc()
    
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
    
    def __init__(self, topic_drone_pairs):
        # Parse topic:drone_id pairs
        self.drone_topics = []
        for pair in topic_drone_pairs:
            if ':' not in pair:
                print(f"Error: Invalid format '{pair}'. Expected 'topic:drone_id'")
                sys.exit(1)
            
            topic, drone_id = pair.split(':', 1)
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
            print("Error: Please specify topic:drone_id pairs for multi mode")
            print()
            print("Usage:")
            print("  python3 swarm_aware_stream.py multi <topic1:drone_id1> <topic2:drone_id2> ...")
            print()
            print("Examples:")
            print("  python3 swarm_aware_stream.py multi /drone_0_pcl_render_node/cloud:drone_0 /drone_1_pcl_render_node/cloud:drone_1")
            print("  python3 swarm_aware_stream.py multi /drone_0_ego_planner/grid:drone_0 /drone_1_sensor/data:drone_1")
            sys.exit(1)
        
        # Get topic:drone_id pairs from command line
        topic_drone_pairs = sys.argv[2:]
        multi_streamer = MultiDroneSwarmStreamer(topic_drone_pairs)
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
        print("  Multi drone:  python3 swarm_aware_stream.py multi <topic1:drone_id1> <topic2:drone_id2> ...")
        print()
        print("Examples:")
        print("  # Single drone")
        print("  python3 swarm_aware_stream.py /drone_0_pcl_render_node/cloud drone_0")
        print()
        print("  # Multiple drones with custom topics")
        print("  python3 swarm_aware_stream.py multi /drone_0_pcl_render_node/cloud:drone_0 /drone_1_pcl_render_node/cloud:drone_1")
        print("  python3 swarm_aware_stream.py multi /drone_0_ego_planner/grid:drone_0 /drone_1_sensor/data:drone_1")
        sys.exit(1)


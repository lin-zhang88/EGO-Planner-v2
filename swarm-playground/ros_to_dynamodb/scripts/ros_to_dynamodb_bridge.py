#!/usr/bin/env python3
"""
ROS to DynamoDB Bridge
Subscribes to ROS topics and streams data to AWS DynamoDB
"""

import rospy
import boto3
import json
import time
from decimal import Decimal
from datetime import datetime
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2


class ROSToDynamoDBBridge:
    def __init__(self):
        rospy.init_node('ros_to_dynamodb_bridge', anonymous=True)
        
        # Load parameters
        self.aws_region = rospy.get_param('~aws_region', 'us-east-1')
        self.table_name = rospy.get_param('~table_name', 'drone_telemetry')
        self.drone_id = rospy.get_param('~drone_id', 'drone_7')
        self.topic_name = rospy.get_param('~topic_name', '/drone_7_ego_planner_node/grid_map/occupancy_inflate')
        self.topic_type = rospy.get_param('~topic_type', 'PointCloud2')
        self.batch_size = rospy.get_param('~batch_size', 25)  # DynamoDB batch limit
        self.rate_limit = rospy.get_param('~rate_limit', 10)  # Hz
        
        # AWS Credentials (should be set via environment variables or AWS credentials file)
        # export AWS_ACCESS_KEY_ID=your_access_key
        # export AWS_SECRET_ACCESS_KEY=your_secret_key
        # Or use ~/.aws/credentials
        
        # Initialize DynamoDB client
        try:
            self.dynamodb = boto3.resource('dynamodb', region_name=self.aws_region)
            self.table = self.dynamodb.Table(self.table_name)
            rospy.loginfo(f"Connected to DynamoDB table: {self.table_name}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to DynamoDB: {e}")
            rospy.logerr("Make sure AWS credentials are configured properly")
            raise
        
        # Initialize rate limiter
        self.rate = rospy.Rate(self.rate_limit)
        self.last_publish_time = time.time()
        
        # Batch buffer
        self.batch_buffer = []
        
        # Subscribe to the appropriate topic
        self.setup_subscriber()
        
        rospy.loginfo(f"ROS to DynamoDB Bridge initialized")
        rospy.loginfo(f"Listening to: {self.topic_name}")
        rospy.loginfo(f"Drone ID: {self.drone_id}")
        rospy.loginfo(f"Writing to DynamoDB table: {self.table_name}")
        
    def setup_subscriber(self):
        """Setup ROS subscriber based on topic type"""
        if self.topic_type == 'PointCloud2':
            self.subscriber = rospy.Subscriber(
                self.topic_name, 
                PointCloud2, 
                self.pointcloud_callback
            )
        elif self.topic_type == 'Odometry':
            self.subscriber = rospy.Subscriber(
                self.topic_name,
                Odometry,
                self.odometry_callback
            )
        elif self.topic_type == 'PoseStamped':
            self.subscriber = rospy.Subscriber(
                self.topic_name,
                PoseStamped,
                self.pose_callback
            )
        else:
            rospy.logerr(f"Unsupported topic type: {self.topic_type}")
            raise ValueError(f"Unsupported topic type: {self.topic_type}")
    
    def pointcloud_callback(self, msg):
        """Handle PointCloud2 messages"""
        try:
            # Extract basic information
            timestamp = msg.header.stamp.to_sec()
            frame_id = msg.header.frame_id
            
            # Count points in the cloud
            point_count = msg.width * msg.height
            
            # Optionally sample some points (don't send all points - too large)
            # Extract just a summary or sample
            sample_points = []
            max_samples = 100  # Only sample first 100 points
            
            for i, point in enumerate(pc2.read_points(msg, skip_nans=True)):
                if i >= max_samples:
                    break
                # Each point is (x, y, z, ...) depending on fields
                sample_points.append({
                    'x': Decimal(str(float(point[0]))),
                    'y': Decimal(str(float(point[1]))),
                    'z': Decimal(str(float(point[2])))
                })
            
            # Create DynamoDB item
            item = {
                'drone_id': self.drone_id,
                'timestamp': Decimal(str(timestamp)),
                'message_type': 'PointCloud2',
                'topic_name': self.topic_name,
                'frame_id': frame_id,
                'point_count': point_count,
                'sample_points': sample_points,
                'width': msg.width,
                'height': msg.height,
                'is_dense': msg.is_dense,
                'ingestion_time': Decimal(str(time.time()))
            }
            
            self.send_to_dynamodb(item)
            
        except Exception as e:
            rospy.logerr(f"Error processing PointCloud2 message: {e}")
    
    def odometry_callback(self, msg):
        """Handle Odometry messages"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            item = {
                'drone_id': self.drone_id,
                'timestamp': Decimal(str(timestamp)),
                'message_type': 'Odometry',
                'topic_name': self.topic_name,
                'position': {
                    'x': Decimal(str(msg.pose.pose.position.x)),
                    'y': Decimal(str(msg.pose.pose.position.y)),
                    'z': Decimal(str(msg.pose.pose.position.z))
                },
                'orientation': {
                    'x': Decimal(str(msg.pose.pose.orientation.x)),
                    'y': Decimal(str(msg.pose.pose.orientation.y)),
                    'z': Decimal(str(msg.pose.pose.orientation.z)),
                    'w': Decimal(str(msg.pose.pose.orientation.w))
                },
                'linear_velocity': {
                    'x': Decimal(str(msg.twist.twist.linear.x)),
                    'y': Decimal(str(msg.twist.twist.linear.y)),
                    'z': Decimal(str(msg.twist.twist.linear.z))
                },
                'angular_velocity': {
                    'x': Decimal(str(msg.twist.twist.angular.x)),
                    'y': Decimal(str(msg.twist.twist.angular.y)),
                    'z': Decimal(str(msg.twist.twist.angular.z))
                },
                'ingestion_time': Decimal(str(time.time()))
            }
            
            self.send_to_dynamodb(item)
            
        except Exception as e:
            rospy.logerr(f"Error processing Odometry message: {e}")
    
    def pose_callback(self, msg):
        """Handle PoseStamped messages"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            item = {
                'drone_id': self.drone_id,
                'timestamp': Decimal(str(timestamp)),
                'message_type': 'PoseStamped',
                'topic_name': self.topic_name,
                'position': {
                    'x': Decimal(str(msg.pose.position.x)),
                    'y': Decimal(str(msg.pose.position.y)),
                    'z': Decimal(str(msg.pose.position.z))
                },
                'orientation': {
                    'x': Decimal(str(msg.pose.orientation.x)),
                    'y': Decimal(str(msg.pose.orientation.y)),
                    'z': Decimal(str(msg.pose.orientation.z)),
                    'w': Decimal(str(msg.pose.orientation.w))
                },
                'ingestion_time': Decimal(str(time.time()))
            }
            
            self.send_to_dynamodb(item)
            
        except Exception as e:
            rospy.logerr(f"Error processing PoseStamped message: {e}")
    
    def send_to_dynamodb(self, item):
        """Send item to DynamoDB (with batching)"""
        # Add to batch buffer
        self.batch_buffer.append(item)
        
        # Send batch if buffer is full or rate limit reached
        if len(self.batch_buffer) >= self.batch_size:
            self.flush_batch()
    
    def flush_batch(self):
        """Flush batch buffer to DynamoDB"""
        if not self.batch_buffer:
            return
        
        try:
            # Use batch writer for efficiency
            with self.table.batch_writer() as batch:
                for item in self.batch_buffer:
                    batch.put_item(Item=item)
            
            rospy.loginfo(f"Successfully wrote {len(self.batch_buffer)} items to DynamoDB")
            self.batch_buffer = []
            
        except Exception as e:
            rospy.logerr(f"Error writing batch to DynamoDB: {e}")
            # Keep items in buffer for retry
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("ROS to DynamoDB Bridge is running...")
        
        # Periodic flush timer
        flush_rate = rospy.Rate(1)  # Flush every second if needed
        
        while not rospy.is_shutdown():
            # Periodic flush even if batch not full
            if self.batch_buffer and (time.time() - self.last_publish_time) > 5.0:
                self.flush_batch()
                self.last_publish_time = time.time()
            
            flush_rate.sleep()
    
    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down ROS to DynamoDB Bridge...")
        # Flush remaining items
        self.flush_batch()
        rospy.loginfo("Shutdown complete")


if __name__ == '__main__':
    try:
        bridge = ROSToDynamoDBBridge()
        rospy.on_shutdown(bridge.shutdown)
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")


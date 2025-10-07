# ROS to DynamoDB Bridge

This package bridges ROS topics to AWS DynamoDB, allowing you to stream drone telemetry and sensor data to the cloud for storage, analysis, and monitoring.

## Features

- Subscribe to multiple ROS message types (PointCloud2, Odometry, PoseStamped)
- Batch writes to DynamoDB for efficiency
- Configurable rate limiting
- Automatic retry on failures
- Support for multiple drones

## Prerequisites

### 1. AWS Account Setup

1. Create an AWS account at https://aws.amazon.com
2. Create a DynamoDB table:
   - Go to AWS Console → DynamoDB → Create Table
   - Table name: `drone_telemetry` (or your preferred name)
   - Partition key: `drone_id` (String)
   - Sort key: `timestamp` (Number)
   - Use default settings for other options
   - Click "Create table"

### 2. AWS Credentials

Create an IAM user with DynamoDB permissions:

1. Go to AWS Console → IAM → Users → Add User
2. Enable "Programmatic access"
3. Attach policy: `AmazonDynamoDBFullAccess` (or create a custom policy)
4. Save the Access Key ID and Secret Access Key

Configure credentials on your local machine:

```bash
# Option 1: Using AWS CLI
pip install awscli
aws configure
# Enter your Access Key ID, Secret Access Key, region (e.g., us-east-1)

# Option 2: Using environment variables
export AWS_ACCESS_KEY_ID=your_access_key_here
export AWS_SECRET_ACCESS_KEY=your_secret_key_here
export AWS_DEFAULT_REGION=us-east-1
```

### 3. Install Python Dependencies

```bash
# Install boto3 (AWS SDK for Python)
pip install boto3

# Or if using pip3
pip3 install boto3

# For system-wide installation
sudo apt-get install python3-boto3  # Ubuntu/Debian
```

## Installation

1. Copy this package to your catkin workspace:

```bash
cd ~/your_workspace/src
# If you haven't already, this package should be in:
# EGO-Planner-v2/swarm-playground/ros_to_dynamodb/
```

2. Build the package:

```bash
cd ~/your_workspace
catkin_make
# or
catkin build ros_to_dynamodb

source devel/setup.bash
```

3. Make the Python script executable:

```bash
chmod +x src/ros_to_dynamodb/scripts/ros_to_dynamodb_bridge.py
```

## Configuration

Edit `config/bridge_config.yaml` to configure the bridge:

```yaml
# AWS Configuration
aws_region: "us-east-1"  # Your AWS region
table_name: "drone_telemetry"  # Your DynamoDB table name

# Drone Configuration
drone_id: "drone_7"  # Which drone to track

# Topic Configuration
topic_name: "/drone_7_ego_planner_node/grid_map/occupancy_inflate"
topic_type: "PointCloud2"  # PointCloud2, Odometry, or PoseStamped

# Performance settings
batch_size: 25  # Messages per batch
rate_limit: 10  # Max messages per second
```

## Usage

### Quick Start

1. Start your ROS system (drones, planners, etc.)

2. In a new terminal, launch the bridge:

```bash
source devel/setup.bash
roslaunch ros_to_dynamodb ros_to_dynamodb.launch
```

3. Verify data is being sent:
   - Check the terminal output for "Successfully wrote X items to DynamoDB"
   - Go to AWS Console → DynamoDB → Tables → drone_telemetry → Explore Items
   - You should see data appearing in real-time

### Advanced Usage

#### Run for a Different Drone

```bash
roslaunch ros_to_dynamodb ros_to_dynamodb.launch \
  drone_id:=drone_0 \
  topic_name:=/drone_0_ego_planner_node/grid_map/occupancy_inflate
```

#### Monitor Multiple Drones Simultaneously

Launch multiple instances:

```bash
# Terminal 1: Drone 7
roslaunch ros_to_dynamodb ros_to_dynamodb.launch

# Terminal 2: Drone 0
roslaunch ros_to_dynamodb ros_to_dynamodb.launch \
  drone_id:=drone_0 \
  topic_name:=/drone_0_ego_planner_node/grid_map/occupancy_inflate

# Terminal 3: Drone 1
roslaunch ros_to_dynamodb ros_to_dynamodb.launch \
  drone_id:=drone_1 \
  topic_name:=/drone_1_ego_planner_node/grid_map/occupancy_inflate
```

#### Track Different Topics

For odometry data:
```bash
roslaunch ros_to_dynamodb ros_to_dynamodb.launch \
  topic_name:=/drone_7_odom_visualization/cmd \
  topic_type:=Odometry
```

## Available Topics

Based on your system, you can subscribe to:

- Grid maps: `/drone_7_ego_planner_node/grid_map/occupancy_inflate` (PointCloud2)
- Planning lists: `/drone_7_ego_planner_node/init_list`
- Optimal trajectories: `/drone_7_ego_planner_node/optimal_list`
- Visualization: `/drone_7_odom_visualization/cmd`
- Covariance: `/drone_7_odom_visualization/covariance`

## Querying Data from DynamoDB

### Using AWS Console

1. Go to DynamoDB → Tables → drone_telemetry → Explore Items
2. Use filters to query specific drones or time ranges

### Using Python (boto3)

```python
import boto3
from decimal import Decimal

dynamodb = boto3.resource('dynamodb', region_name='us-east-1')
table = dynamodb.Table('drone_telemetry')

# Query all data for drone_7
response = table.query(
    KeyConditionExpression='drone_id = :drone_id',
    ExpressionAttributeValues={
        ':drone_id': 'drone_7'
    }
)

for item in response['Items']:
    print(f"Timestamp: {item['timestamp']}, Points: {item['point_count']}")
```

### Using AWS CLI

```bash
# Get the latest 10 items for drone_7
aws dynamodb query \
    --table-name drone_telemetry \
    --key-condition-expression "drone_id = :drone_id" \
    --expression-attribute-values '{":drone_id":{"S":"drone_7"}}' \
    --limit 10 \
    --scan-index-forward false
```

## Data Structure

The bridge stores data with the following structure:

### PointCloud2 Messages
```json
{
  "drone_id": "drone_7",
  "timestamp": 1234567890.123,
  "message_type": "PointCloud2",
  "topic_name": "/drone_7_ego_planner_node/grid_map/occupancy_inflate",
  "frame_id": "world",
  "point_count": 15000,
  "sample_points": [
    {"x": 1.5, "y": 2.3, "z": 0.8},
    ...
  ],
  "width": 150,
  "height": 100,
  "is_dense": true,
  "ingestion_time": 1234567891.456
}
```

### Odometry Messages
```json
{
  "drone_id": "drone_7",
  "timestamp": 1234567890.123,
  "message_type": "Odometry",
  "position": {"x": 1.5, "y": 2.3, "z": 0.8},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
  "linear_velocity": {"x": 0.5, "y": 0.0, "z": 0.0},
  "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.1},
  "ingestion_time": 1234567891.456
}
```

## Troubleshooting

### "Unable to locate credentials"

Make sure AWS credentials are configured:
```bash
aws configure
# or
export AWS_ACCESS_KEY_ID=your_key
export AWS_SECRET_ACCESS_KEY=your_secret
```

### "ResourceNotFoundException: Table not found"

Create the DynamoDB table in AWS Console with:
- Partition key: `drone_id` (String)
- Sort key: `timestamp` (Number)

### No data appearing in DynamoDB

1. Check if the ROS topic is publishing:
   ```bash
   rostopic echo /drone_7_ego_planner_node/grid_map/occupancy_inflate
   ```

2. Check the bridge node is running:
   ```bash
   rosnode list | grep dynamodb
   ```

3. Check the logs:
   ```bash
   rosnode log ros_to_dynamodb_bridge
   ```

### Rate limiting issues

If you're getting throttled by DynamoDB:
- Reduce `rate_limit` in config
- Increase provisioned capacity in DynamoDB
- Consider using DynamoDB On-Demand mode

## Cost Considerations

- DynamoDB charges per read/write request
- On-Demand mode: ~$1.25 per million write requests
- Provisioned mode: Fixed hourly cost
- Consider the data volume and frequency when choosing a mode

## License

MIT License

## Support

For issues or questions, please check the logs and AWS documentation:
- ROS logs: `rosnode log ros_to_dynamodb_bridge`
- AWS DynamoDB docs: https://docs.aws.amazon.com/dynamodb/


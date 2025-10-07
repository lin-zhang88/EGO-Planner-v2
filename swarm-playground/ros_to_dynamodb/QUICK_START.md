# Quick Start Guide - ROS to DynamoDB

Get your drone data into AWS DynamoDB in 5 minutes!

## Step 1: Install Dependencies

```bash
# Install boto3 (AWS SDK)
pip3 install boto3

# Or system-wide
sudo apt-get install python3-boto3
```

## Step 2: Configure AWS Credentials

### Option A: Using AWS CLI (Recommended)
```bash
pip3 install awscli
aws configure
# Enter:
#   AWS Access Key ID: [Your access key]
#   AWS Secret Access Key: [Your secret key]
#   Default region: us-east-1
#   Default output format: json
```

### Option B: Using Environment Variables
```bash
export AWS_ACCESS_KEY_ID=your_access_key_here
export AWS_SECRET_ACCESS_KEY=your_secret_key_here
export AWS_DEFAULT_REGION=us-east-1

# Add to ~/.bashrc to make permanent
echo 'export AWS_ACCESS_KEY_ID=your_key' >> ~/.bashrc
echo 'export AWS_SECRET_ACCESS_KEY=your_secret' >> ~/.bashrc
```

**Don't have AWS credentials?**
1. Go to https://aws.amazon.com and create an account
2. Go to IAM → Users → Add User
3. Enable "Programmatic access"
4. Attach "AmazonDynamoDBFullAccess" policy
5. Save the credentials

## Step 3: Create DynamoDB Table

### Option A: Using the setup script (Easy!)
```bash
cd /Users/owusunp/EGO-Planner-v2/swarm-playground/ros_to_dynamodb/scripts
python3 setup_aws.py
# Follow the prompts
```

### Option B: Manual setup via AWS Console
1. Go to AWS Console → DynamoDB → Create Table
2. Table name: `drone_telemetry`
3. Partition key: `drone_id` (String)
4. Sort key: `timestamp` (Number)
5. Billing mode: On-Demand
6. Click "Create table"

## Step 4: Build the ROS Package

```bash
# Navigate to your workspace (adjust path if needed)
cd /Users/owusunp/EGO-Planner-v2/swarm-playground/formation_ws
# Or: cd /Users/owusunp/EGO-Planner-v2/swarm-playground/main_ws

# Copy the package to the workspace
cp -r ../ros_to_dynamodb src/

# Build
catkin_make
# or: catkin build ros_to_dynamodb

# Source the workspace
source devel/setup.bash

# Make script executable
chmod +x src/ros_to_dynamodb/scripts/*.py
```

## Step 5: Run It!

### Start your drones first
```bash
# In one terminal, start your drone simulation/system
# (whatever you normally do to get your drones running)
roslaunch ...  # your normal launch command
```

### Start the bridge
```bash
# In a new terminal
source devel/setup.bash
roslaunch ros_to_dynamodb ros_to_dynamodb.launch
```

You should see:
```
[INFO] Connected to DynamoDB table: drone_telemetry
[INFO] ROS to DynamoDB Bridge initialized
[INFO] Listening to: /drone_7_ego_planner_node/grid_map/occupancy_inflate
[INFO] Successfully wrote 25 items to DynamoDB
```

## Step 6: Verify Data

### Check in terminal
```bash
cd /Users/owusunp/EGO-Planner-v2/swarm-playground/ros_to_dynamodb/scripts
python3 query_data.py
```

### Check in AWS Console
1. Go to AWS Console → DynamoDB → Tables
2. Click on `drone_telemetry`
3. Click "Explore table items"
4. You should see your data!

## Troubleshooting

### "Unable to locate credentials"
```bash
# Verify credentials are set
aws sts get-caller-identity

# If error, reconfigure
aws configure
```

### "Table drone_telemetry not found"
```bash
# Run setup script
python3 scripts/setup_aws.py
```

### "No module named 'boto3'"
```bash
# Install boto3
pip3 install boto3
```

### Bridge not receiving messages
```bash
# Check if topic is publishing
rostopic list | grep drone_7
rostopic hz /drone_7_ego_planner_node/grid_map/occupancy_inflate

# Check if bridge node is running
rosnode list | grep dynamodb
```

## Customize for Your Setup

Edit `config/bridge_config.yaml`:

```yaml
# Track a different drone
drone_id: "drone_0"
topic_name: "/drone_0_ego_planner_node/grid_map/occupancy_inflate"

# Or track odometry
topic_name: "/drone_7_odom_visualization/cmd"
topic_type: "Odometry"

# Adjust performance
rate_limit: 5  # Lower if you want less data
batch_size: 10  # Smaller batches = more frequent writes
```

## Next Steps

- **Multiple drones**: Launch multiple bridge instances with different configs
- **Data analysis**: Use the query script to analyze your data
- **Visualization**: Connect to AWS QuickSight or export to S3 for analysis
- **Monitoring**: Set up CloudWatch alarms on your DynamoDB table

## Cost Estimate

DynamoDB On-Demand pricing (as of 2024):
- Write: $1.25 per million requests
- Read: $0.25 per million requests
- Storage: $0.25 per GB-month

Example: At 10 Hz for 1 hour = 36,000 writes ≈ $0.045

## Support

If you run into issues:
1. Check the bridge logs: `rosnode log ros_to_dynamodb_bridge`
2. Test AWS connection: `python3 scripts/setup_aws.py`
3. Verify ROS topics: `rostopic list` and `rostopic echo <topic>`

Happy data streaming! 🚁☁️


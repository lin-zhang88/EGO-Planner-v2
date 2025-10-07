# Ubuntu Setup Guide - Real-Time ROS to DynamoDB

Follow these steps on your **Ubuntu machine** to stream drone data to DynamoDB in real-time.

---

## STEP 1: Copy Files to Ubuntu

Copy the entire `ros_to_dynamodb` folder to your Ubuntu machine:

```bash
# On Ubuntu, navigate to your workspace
cd ~/your_catkin_workspace/src
# (e.g., cd ~/EGO-Planner-v2/swarm-playground/formation_ws/src)

# If you're copying from Mac via scp:
# scp -r /Users/owusunp/EGO-Planner-v2/swarm-playground/ros_to_dynamodb ubuntu@your-ip:~/path/to/workspace/src/

# Or just copy the folder manually using USB, shared folder, etc.
```

---

## STEP 2: Install Python Dependencies on Ubuntu

```bash
# Install boto3 (AWS SDK)
pip3 install boto3

# Or system-wide
sudo apt-get update
sudo apt-get install -y python3-boto3
```

---

## STEP 3: Set AWS Credentials on Ubuntu

**Option A: Using environment variables (recommended for temporary credentials)**

```bash
# Run this in your terminal (replace with your actual credentials)
export AWS_DEFAULT_REGION="us-east-1"
export AWS_ACCESS_KEY_ID="your_access_key_here"
export AWS_SECRET_ACCESS_KEY="your_secret_key_here"
export AWS_SESSION_TOKEN="your_session_token_here"  # if using temporary credentials

# Verify credentials work
pip3 install awscli  # if not already installed
aws sts get-caller-identity
```

**Option B: Using AWS CLI configure (for permanent credentials)**

```bash
pip3 install awscli
aws configure
# Enter your Access Key ID, Secret Key, region (us-east-1), format (json)
```

---

## STEP 4: Make Scripts Executable

```bash
cd ~/your_workspace/src/ros_to_dynamodb/scripts
chmod +x *.py
chmod +x *.sh
```

---

## STEP 5: Build the ROS Package (if using catkin)

```bash
cd ~/your_workspace
catkin_make
# or: catkin build ros_to_dynamodb

source devel/setup.bash
```

---

## STEP 6: Start Your ROS System

```bash
# In one terminal, start your drone simulation/system
# For example:
source devel/setup.bash
roslaunch your_package your_launch_file.launch

# Make sure your topics are publishing
rostopic list | grep drone_0
```

---

## STEP 7: Stream Real-Time Data to DynamoDB

**Option A: Using the standalone Python script (EASIEST)**

```bash
# In a new terminal
cd ~/your_workspace/src/ros_to_dynamodb/scripts

# Make sure AWS credentials are set
export AWS_DEFAULT_REGION="us-east-1"
export AWS_ACCESS_KEY_ID="your_key"
export AWS_SECRET_ACCESS_KEY="your_secret"
# Add AWS_SESSION_TOKEN if using temporary credentials

# Run the bridge
python3 ros_to_dynamodb_bridge.py \
    _aws_region:=us-east-1 \
    _table_name:=drone_telemetry \
    _drone_id:=drone_0 \
    _topic_name:=/drone_0_pcl_render_node/cloud \
    _topic_type:=PointCloud2 \
    _batch_size:=25 \
    _rate_limit:=10
```

**Option B: Using roslaunch**

```bash
# In a new terminal
source devel/setup.bash
roslaunch ros_to_dynamodb drone_0_cloud.launch
```

**Option C: Using the convenience script**

```bash
cd ~/your_workspace/src/ros_to_dynamodb/scripts
./run_realtime_bridge.sh /drone_0_pcl_render_node/cloud drone_0
```

---

## STEP 8: Verify Data is Being Sent

You should see output like:
```
[INFO] ROS to DynamoDB Bridge initialized
[INFO] Listening to: /drone_0_pcl_render_node/cloud
[INFO] Drone ID: drone_0
[INFO] Successfully wrote 25 items to DynamoDB
[INFO] Successfully wrote 25 items to DynamoDB
...
```

---

## STEP 9: Query Your Data

**On Ubuntu or Mac:**

```bash
cd ~/your_workspace/src/ros_to_dynamodb/scripts
python3 query_data.py drone_0 20
```

**Or check AWS Console:**
1. Go to https://console.aws.amazon.com
2. Navigate to DynamoDB → Tables → drone_telemetry
3. Click "Explore table items"
4. You should see real-time data appearing!

---

## Troubleshooting

### "Unable to locate credentials"
```bash
# Set credentials again
export AWS_ACCESS_KEY_ID="your_key"
export AWS_SECRET_ACCESS_KEY="your_secret"

# Verify
aws sts get-caller-identity
```

### "No module named 'boto3'"
```bash
pip3 install boto3
```

### "Topic not found"
```bash
# Check available topics
rostopic list

# Check if topic is publishing
rostopic echo /drone_0_pcl_render_node/cloud
```

### Bridge not receiving messages
```bash
# Check topic rate
rostopic hz /drone_0_pcl_render_node/cloud

# Check if bridge node is running
rosnode list | grep dynamodb
```

---

## Summary of Commands (Quick Copy-Paste for Ubuntu)

```bash
# 1. Install dependencies
pip3 install boto3

# 2. Set AWS credentials (in every new terminal or add to ~/.bashrc)
export AWS_DEFAULT_REGION="us-east-1"
export AWS_ACCESS_KEY_ID="your_key"
export AWS_SECRET_ACCESS_KEY="your_secret"
export AWS_SESSION_TOKEN="your_token"  # if using temporary credentials

# 3. Navigate to scripts
cd ~/your_workspace/src/ros_to_dynamodb/scripts

# 4. Make executable
chmod +x *.py

# 5. Run the bridge (make sure ROS is running first!)
python3 ros_to_dynamodb_bridge.py \
    _aws_region:=us-east-1 \
    _table_name:=drone_telemetry \
    _drone_id:=drone_0 \
    _topic_name:=/drone_0_pcl_render_node/cloud \
    _topic_type:=PointCloud2

# 6. In another terminal, query data
python3 query_data.py drone_0
```

---

## Files Created

All these files are in the `ros_to_dynamodb` folder:

```
ros_to_dynamodb/
├── CMakeLists.txt
├── package.xml
├── README.md
├── QUICK_START.md
├── UBUNTU_SETUP_GUIDE.md (this file)
├── config/
│   ├── bridge_config.yaml
│   └── drone_0_cloud.yaml
├── launch/
│   ├── ros_to_dynamodb.launch
│   └── drone_0_cloud.launch
└── scripts/
    ├── ros_to_dynamodb_bridge.py (MAIN BRIDGE SCRIPT)
    ├── setup_aws.py
    ├── test_push.py
    ├── query_data.py
    └── run_realtime_bridge.sh
```

Good luck! 🚁☁️


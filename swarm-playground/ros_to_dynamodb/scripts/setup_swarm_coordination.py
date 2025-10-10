#!/usr/bin/env python3
"""
Setup AWS IoT + Lambda for Real-time Swarm Coordination
Each drone publishes its position, Lambda aggregates and broadcasts to all drones
"""

import boto3
import json
import zipfile
import io
import time

AWS_REGION = "us-east-1"
LAMBDA_FUNCTION_NAME = "DroneSwarmCoordinator"
IOT_RULE_NAME = "DroneSwarmCoordinationRule"
TABLE_NAME = "drone_telemetry"

# Lambda function code
LAMBDA_CODE = '''
import json
import boto3
import time
from decimal import Decimal

dynamodb = boto3.resource('dynamodb')
iot_client = boto3.client('iot-data')
table = dynamodb.Table('drone_telemetry')

def lambda_handler(event, context):
    """
    Process incoming drone telemetry:
    1. Store in DynamoDB
    2. Query all recent drone positions
    3. Broadcast swarm state to all drones
    """
    try:
        # Parse incoming telemetry
        telemetry = event
        drone_id = telemetry.get('drone_id', 'unknown')
        
        print(f"Processing telemetry from {drone_id}")
        
        # Step 1: Store in DynamoDB
        ddb_item = convert_to_decimal(telemetry)
        table.put_item(Item=ddb_item)
        
        # Step 2: Query recent positions of all drones (last 5 seconds)
        current_time = time.time()
        cutoff_time = current_time - 5  # Last 5 seconds
        
        # Scan DynamoDB for all recent drones
        swarm_state = {}
        
        try:
            # Get all drone IDs by scanning recent data
            response = table.scan(
                FilterExpression='#ts > :cutoff',
                ExpressionAttributeNames={'#ts': 'timestamp'},
                ExpressionAttributeValues={':cutoff': Decimal(str(cutoff_time))},
                Limit=100
            )
            
            # Group by drone_id and keep only the latest from each
            drone_latest = {}
            for item in response.get('Items', []):
                d_id = item.get('drone_id')
                d_ts = float(item.get('timestamp', 0))
                
                if d_id not in drone_latest or d_ts > drone_latest[d_id]['timestamp']:
                    # Extract position from message_data if available
                    message_data = item.get('message_data', {})
                    position = {'x': 0, 'y': 0, 'z': 0}
                    
                    # Try to get position from pose message
                    if 'pose' in message_data:
                        if 'position' in message_data['pose']:
                            pos = message_data['pose']['position']
                            position = {
                                'x': float(pos.get('x', 0)),
                                'y': float(pos.get('y', 0)),
                                'z': float(pos.get('z', 0))
                            }
                        elif 'pose' in message_data['pose'] and 'position' in message_data['pose']['pose']:
                            pos = message_data['pose']['pose']['position']
                            position = {
                                'x': float(pos.get('x', 0)),
                                'y': float(pos.get('y', 0)),
                                'z': float(pos.get('z', 0))
                            }
                    
                    # Fallback to sample_points if available
                    sample_points = item.get('sample_points', [])
                    if not any(position.values()) and sample_points:
                        position = {
                            'x': float(sample_points[0].get('x', 0)),
                            'y': float(sample_points[0].get('y', 0)),
                            'z': float(sample_points[0].get('z', 0))
                        }
                    
                    drone_latest[d_id] = {
                        'timestamp': d_ts,
                        'position': position,
                        'message_type': str(item.get('message_type', 'unknown')),
                        'topic_name': str(item.get('topic_name', 'unknown'))
                    }
            
            swarm_state = drone_latest
            print(f"Found {len(swarm_state)} drones in swarm")
            
        except Exception as query_error:
            print(f"Error querying drones: {query_error}")
            # Fallback to just current drone
            swarm_state[drone_id] = {
                'timestamp': telemetry.get('timestamp'),
                'position': {'x': 0, 'y': 0, 'z': 0},
                'message_type': telemetry.get('message_type', 'unknown'),
                'topic_name': telemetry.get('topic_name', 'unknown')
            }
        
        # Step 3: Broadcast swarm state to all drones
        swarm_message = {
            'message_type': 'swarm_state',
            'timestamp': current_time,
            'drone_count': len(swarm_state),
            'drones': swarm_state
        }
        
        # Publish to swarm coordination topic
        iot_client.publish(
            topic='drone/swarm/state',
            qos=1,
            payload=json.dumps(swarm_message, default=float)
        )
        
        print(f"Broadcasted swarm state with {len(swarm_state)} drones")
        
        return {
            'statusCode': 200,
            'body': json.dumps({
                'message': 'Swarm coordination successful',
                'drone_id': drone_id,
                'swarm_size': len(swarm_state)
            })
        }
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps({'error': str(e)})
        }

def convert_to_decimal(obj):
    """Convert floats to Decimal for DynamoDB"""
    if isinstance(obj, dict):
        return {k: convert_to_decimal(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_decimal(item) for item in obj]
    elif isinstance(obj, float):
        return Decimal(str(obj))
    else:
        return obj
'''

def create_lambda_package():
    """Create Lambda deployment package"""
    print("📦 Creating Lambda deployment package...")
    
    zip_buffer = io.BytesIO()
    with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
        zip_file.writestr('lambda_function.py', LAMBDA_CODE)
    
    zip_buffer.seek(0)
    return zip_buffer.read()

def setup_swarm_coordination():
    """Setup complete swarm coordination system"""
    print("=" * 70)
    print("🚁 Setting up AWS IoT + Lambda Swarm Coordination")
    print("=" * 70)
    
    try:
        # Initialize AWS clients
        lambda_client = boto3.client('lambda', region_name=AWS_REGION)
        iot = boto3.client('iot', region_name=AWS_REGION)
        iam = boto3.client('iam', region_name=AWS_REGION)
        sts = boto3.client('sts', region_name=AWS_REGION)
        
        account_id = sts.get_caller_identity()['Account']
        print(f"Account ID: {account_id}\n")
        
        # Step 1: Create IAM role for Lambda
        lambda_role_name = "DroneSwarmLambdaRole"
        lambda_trust_policy = {
            "Version": "2012-10-17",
            "Statement": [
                {
                    "Effect": "Allow",
                    "Principal": {"Service": "lambda.amazonaws.com"},
                    "Action": "sts:AssumeRole"
                }
            ]
        }
        
        print(f"📝 Creating Lambda IAM role: {lambda_role_name}")
        try:
            lambda_role = iam.create_role(
                RoleName=lambda_role_name,
                AssumeRolePolicyDocument=json.dumps(lambda_trust_policy),
                Description="Role for Drone Swarm Coordinator Lambda"
            )
            lambda_role_arn = lambda_role['Role']['Arn']
            print(f"✓ Created role: {lambda_role_arn}")
            time.sleep(10)  # Wait for role to propagate
        except iam.exceptions.EntityAlreadyExistsException:
            lambda_role_arn = f"arn:aws:iam::{account_id}:role/{lambda_role_name}"
            print(f"✓ Role already exists: {lambda_role_arn}")
        
        # Attach policies to Lambda role
        print("\n📝 Attaching policies to Lambda role...")
        policies = [
            "arn:aws:iam::aws:policy/service-role/AWSLambdaBasicExecutionRole",
            "arn:aws:iam::aws:policy/AmazonDynamoDBFullAccess",
            "arn:aws:iam::aws:policy/AWSIoTFullAccess"
        ]
        
        for policy_arn in policies:
            try:
                iam.attach_role_policy(RoleName=lambda_role_name, PolicyArn=policy_arn)
                print(f"✓ Attached: {policy_arn.split('/')[-1]}")
            except Exception as e:
                print(f"⚠ Policy: {e}")
        
        # Step 2: Create Lambda function
        print(f"\n📝 Creating Lambda function: {LAMBDA_FUNCTION_NAME}")
        
        lambda_package = create_lambda_package()
        
        try:
            # Delete if exists
            try:
                lambda_client.delete_function(FunctionName=LAMBDA_FUNCTION_NAME)
                print("✓ Deleted existing function")
                time.sleep(2)
            except:
                pass
            
            lambda_response = lambda_client.create_function(
                FunctionName=LAMBDA_FUNCTION_NAME,
                Runtime='python3.9',
                Role=lambda_role_arn,
                Handler='lambda_function.lambda_handler',
                Code={'ZipFile': lambda_package},
                Description='Drone Swarm Coordinator - aggregates and broadcasts drone positions',
                Timeout=30,
                MemorySize=256,
                Environment={
                    'Variables': {
                        'TABLE_NAME': TABLE_NAME
                    }
                }
            )
            lambda_arn = lambda_response['FunctionArn']
            print(f"✓ Created Lambda: {lambda_arn}")
        except Exception as e:
            print(f"✗ Lambda creation failed: {e}")
            return
        
        # Step 3: Create IoT role for invoking Lambda
        iot_role_name = "IoTLambdaInvokeRole"
        iot_trust_policy = {
            "Version": "2012-10-17",
            "Statement": [
                {
                    "Effect": "Allow",
                    "Principal": {"Service": "iot.amazonaws.com"},
                    "Action": "sts:AssumeRole"
                }
            ]
        }
        
        print(f"\n📝 Creating IoT IAM role: {iot_role_name}")
        try:
            iot_role = iam.create_role(
                RoleName=iot_role_name,
                AssumeRolePolicyDocument=json.dumps(iot_trust_policy),
                Description="Role for IoT to invoke Lambda"
            )
            iot_role_arn = iot_role['Role']['Arn']
            print(f"✓ Created role: {iot_role_arn}")
            time.sleep(10)
        except iam.exceptions.EntityAlreadyExistsException:
            iot_role_arn = f"arn:aws:iam::{account_id}:role/{iot_role_name}"
            print(f"✓ Role already exists: {iot_role_arn}")
        
        # Attach Lambda invoke policy
        lambda_policy = {
            "Version": "2012-10-17",
            "Statement": [
                {
                    "Effect": "Allow",
                    "Action": "lambda:InvokeFunction",
                    "Resource": lambda_arn
                }
            ]
        }
        
        try:
            iam.put_role_policy(
                RoleName=iot_role_name,
                PolicyName="LambdaInvokePolicy",
                PolicyDocument=json.dumps(lambda_policy)
            )
            print("✓ Attached Lambda invoke policy")
        except Exception as e:
            print(f"⚠ Policy error: {e}")
        
        # Step 4: Create IoT rule to trigger Lambda
        print(f"\n📝 Creating IoT rule: {IOT_RULE_NAME}")
        
        rule_payload = {
            "sql": "SELECT * FROM 'drone/telemetry/#'",
            "description": "Trigger swarm coordinator Lambda for each drone message",
            "actions": [
                {
                    "lambda": {
                        "functionArn": lambda_arn
                    }
                }
            ],
            "ruleDisabled": False
        }
        
        try:
            # Delete if exists
            try:
                iot.delete_topic_rule(ruleName=IOT_RULE_NAME)
                print("✓ Deleted existing rule")
            except:
                pass
            
            iot.create_topic_rule(
                ruleName=IOT_RULE_NAME,
                topicRulePayload=rule_payload
            )
            print(f"✓ Created IoT rule: {IOT_RULE_NAME}")
        except Exception as e:
            print(f"✗ Rule creation failed: {e}")
            print("Waiting 10 seconds and retrying...")
            time.sleep(10)
            try:
                iot.create_topic_rule(
                    ruleName=IOT_RULE_NAME,
                    topicRulePayload=rule_payload
                )
                print(f"✓ Created IoT rule: {IOT_RULE_NAME}")
            except Exception as retry_error:
                print(f"✗ Retry failed: {retry_error}")
                return
        
        # Step 5: Add Lambda permission for IoT
        print("\n📝 Adding Lambda permission for IoT...")
        try:
            lambda_client.add_permission(
                FunctionName=LAMBDA_FUNCTION_NAME,
                StatementId='IoTInvokePermission',
                Action='lambda:InvokeFunction',
                Principal='iot.amazonaws.com',
                SourceArn=f"arn:aws:iot:{AWS_REGION}:{account_id}:rule/{IOT_RULE_NAME}"
            )
            print("✓ Lambda permission added")
        except lambda_client.exceptions.ResourceConflictException:
            print("✓ Permission already exists")
        except Exception as e:
            print(f"⚠ Permission error: {e}")
        
        print("\n" + "=" * 70)
        print("🎉 Swarm Coordination Setup Complete!")
        print("=" * 70)
        print("\n📊 Architecture:")
        print("  1. Drones publish → drone/telemetry/{drone_id}")
        print("  2. IoT Rule triggers → Lambda function")
        print("  3. Lambda stores → DynamoDB")
        print("  4. Lambda broadcasts → drone/swarm/state")
        print("  5. Drones subscribe → drone/swarm/state (get all positions)")
        print("\n🚀 Next Steps:")
        print("  1. Run: python3 swarm_aware_stream.py multi <topic1:drone_id1> <topic2:drone_id2>")
        print("     Example: python3 swarm_aware_stream.py multi /drone_0_odom_visualization/pose:drone_0 /drone_1_odom_visualization/pose:drone_1")
        print("  2. Subscribe to: drone/swarm/state (in IoT Core)")
        print("  3. Watch real-time swarm coordination with ALL drones!")
        
    except Exception as e:
        print(f"\n✗ Setup failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    setup_swarm_coordination()


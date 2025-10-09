#!/usr/bin/env python3
"""
Setup AWS IoT Greengrass for Drone Telemetry
Creates Greengrass group, device, and Lambda function
"""

import boto3
import json
import time
import os

# Configuration
AWS_REGION = "us-east-1"
GREENGRASS_GROUP_NAME = "DroneTelemetryGroup"
LAMBDA_FUNCTION_NAME = "DroneTelemetryProcessor"
TABLE_NAME = "drone_telemetry"

def setup_greengrass():
    """Setup Greengrass group and components"""
    print("=" * 60)
    print("Setting up AWS IoT Greengrass for Drone Telemetry")
    print("=" * 60)
    
    try:
        # Initialize clients
        greengrass = boto3.client('greengrass', region_name=AWS_REGION)
        iot = boto3.client('iot', region_name=AWS_REGION)
        lambda_client = boto3.client('lambda', region_name=AWS_REGION)
        iam = boto3.client('iam', region_name=AWS_REGION)
        
        print("✓ Connected to AWS services")
        
        # Step 1: Create Greengrass Group
        print("\n📦 Creating Greengrass Group...")
        try:
            group_response = greengrass.create_group(
                Name=GREENGRASS_GROUP_NAME,
                InitialVersion={
                    'CoreDefinitionVersionArn': 'arn:aws:greengrass:us-east-1:111503169675:greengrass/definition/cores/1',
                    'DeviceDefinitionVersionArn': 'arn:aws:greengrass:us-east-1:111503169675:greengrass/definition/devices/1',
                    'FunctionDefinitionVersionArn': 'arn:aws:greengrass:us-east-1:111503169675:greengrass/definition/functions/1'
                }
            )
            group_id = group_response['Id']
            print(f"✓ Created Greengrass Group: {group_id}")
        except Exception as e:
            print(f"⚠ Group might already exist: {e}")
            # Try to get existing group
            groups = greengrass.list_groups()
            group_id = None
            for group in groups['Groups']:
                if group['Name'] == GREENGRASS_GROUP_NAME:
                    group_id = group['Id']
                    break
            
            if not group_id:
                print("✗ Could not find or create group")
                return
            print(f"✓ Using existing group: {group_id}")
        
        # Step 2: Create IAM Role for Lambda
        print("\n🔐 Creating IAM Role for Lambda...")
        role_name = "DroneTelemetryGreengrassRole"
        trust_policy = {
            "Version": "2012-10-17",
            "Statement": [
                {
                    "Effect": "Allow",
                    "Principal": {
                        "Service": "lambda.amazonaws.com"
                    },
                    "Action": "sts:AssumeRole"
                }
            ]
        }
        
        try:
            role = iam.create_role(
                RoleName=role_name,
                AssumeRolePolicyDocument=json.dumps(trust_policy),
                Description="Role for Greengrass Lambda function"
            )
            print(f"✓ Created IAM Role: {role_name}")
        except Exception as e:
            print(f"⚠ Role might already exist: {e}")
        
        # Attach policies
        policies = [
            "arn:aws:iam::aws:policy/service-role/AWSLambdaBasicExecutionRole",
            "arn:aws:iam::aws:policy/AmazonDynamoDBFullAccess"
        ]
        
        for policy_arn in policies:
            try:
                iam.attach_role_policy(RoleName=role_name, PolicyArn=policy_arn)
                print(f"✓ Attached policy: {policy_arn}")
            except Exception as e:
                print(f"⚠ Policy attachment: {e}")
        
        # Step 3: Create Lambda function
        print("\n⚡ Creating Lambda function...")
        lambda_code = '''
import json
import boto3
from decimal import Decimal

def lambda_handler(event, context):
    """Process drone telemetry and send to DynamoDB"""
    dynamodb = boto3.resource('dynamodb')
    table = dynamodb.Table('drone_telemetry')
    
    try:
        # Convert floats back to Decimal for DynamoDB
        item = convert_floats_to_decimals(event)
        
        # Send to DynamoDB
        table.put_item(Item=item)
        
        return {
            'statusCode': 200,
            'body': json.dumps('Success')
        }
    except Exception as e:
        print(f"Error: {e}")
        return {
            'statusCode': 500,
            'body': json.dumps(str(e))
        }

def convert_floats_to_decimals(obj):
    """Convert float values back to Decimal for DynamoDB"""
    if isinstance(obj, dict):
        return {k: convert_floats_to_decimals(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_floats_to_decimals(item) for item in obj]
    elif isinstance(obj, float):
        return Decimal(str(obj))
    else:
        return obj
'''
        
        try:
            lambda_response = lambda_client.create_function(
                FunctionName=LAMBDA_FUNCTION_NAME,
                Runtime='python3.9',
                Role=f'arn:aws:iam::111503169675:role/{role_name}',
                Handler='index.lambda_handler',
                Code={'ZipFile': lambda_code.encode()},
                Description='Process drone telemetry data'
            )
            print(f"✓ Created Lambda function: {LAMBDA_FUNCTION_NAME}")
        except Exception as e:
            print(f"⚠ Lambda function might already exist: {e}")
        
        # Step 4: Create IoT Thing
        print("\n🔗 Creating IoT Thing...")
        thing_name = "DroneTelemetryThing"
        try:
            iot.create_thing(thingName=thing_name)
            print(f"✓ Created IoT Thing: {thing_name}")
        except Exception as e:
            print(f"⚠ Thing might already exist: {e}")
        
        print("\n🎉 Greengrass setup complete!")
        print("\nNext steps:")
        print("1. Install Greengrass Core on your device")
        print("2. Download and install the Greengrass Core software")
        print("3. Configure the device to connect to your Greengrass group")
        print("4. Test with: python3 test_greengrass.py")
        
    except Exception as e:
        print(f"✗ Setup failed: {e}")

if __name__ == '__main__':
    setup_greengrass()

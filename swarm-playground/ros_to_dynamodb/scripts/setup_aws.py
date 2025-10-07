#!/usr/bin/env python3
"""
AWS DynamoDB Setup Script
Creates the DynamoDB table needed for the ROS bridge
"""

import boto3
import sys
from botocore.exceptions import ClientError


def create_table(table_name='drone_telemetry', region='us-east-1'):
    """Create DynamoDB table with appropriate schema"""
    
    try:
        dynamodb = boto3.resource('dynamodb', region_name=region)
        
        print(f"Creating DynamoDB table: {table_name}")
        print(f"Region: {region}")
        
        table = dynamodb.create_table(
            TableName=table_name,
            KeySchema=[
                {
                    'AttributeName': 'drone_id',
                    'KeyType': 'HASH'  # Partition key
                },
                {
                    'AttributeName': 'timestamp',
                    'KeyType': 'RANGE'  # Sort key
                }
            ],
            AttributeDefinitions=[
                {
                    'AttributeName': 'drone_id',
                    'AttributeType': 'S'  # String
                },
                {
                    'AttributeName': 'timestamp',
                    'AttributeType': 'N'  # Number
                }
            ],
            BillingMode='PAY_PER_REQUEST',  # On-demand billing
            Tags=[
                {
                    'Key': 'Project',
                    'Value': 'EGO-Planner-Drones'
                },
                {
                    'Key': 'Purpose',
                    'Value': 'Telemetry'
                }
            ]
        )
        
        print("Waiting for table to be created...")
        table.meta.client.get_waiter('table_exists').wait(TableName=table_name)
        
        print(f"✓ Table '{table_name}' created successfully!")
        print(f"\nTable details:")
        print(f"  - Partition key: drone_id (String)")
        print(f"  - Sort key: timestamp (Number)")
        print(f"  - Billing mode: On-Demand")
        print(f"  - Region: {region}")
        print(f"\nYou can now start the ROS bridge!")
        
        return True
        
    except ClientError as e:
        if e.response['Error']['Code'] == 'ResourceInUseException':
            print(f"✓ Table '{table_name}' already exists!")
            return True
        else:
            print(f"✗ Error creating table: {e}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        print("\nMake sure your AWS credentials are configured:")
        print("  aws configure")
        print("  or set AWS_ACCESS_KEY_ID and AWS_SECRET_ACCESS_KEY")
        return False


def check_credentials():
    """Verify AWS credentials are configured"""
    try:
        sts = boto3.client('sts')
        identity = sts.get_caller_identity()
        print("✓ AWS credentials are configured")
        print(f"  Account: {identity['Account']}")
        print(f"  User ARN: {identity['Arn']}")
        return True
    except Exception as e:
        print("✗ AWS credentials not found or invalid")
        print(f"  Error: {e}")
        print("\nPlease configure AWS credentials:")
        print("  aws configure")
        print("  or set environment variables:")
        print("    export AWS_ACCESS_KEY_ID=your_key")
        print("    export AWS_SECRET_ACCESS_KEY=your_secret")
        return False


if __name__ == '__main__':
    print("=" * 60)
    print("AWS DynamoDB Setup for ROS Bridge")
    print("=" * 60)
    print()
    
    # Check credentials
    if not check_credentials():
        sys.exit(1)
    
    print()
    
    # Get parameters
    table_name = input("Enter table name [drone_telemetry]: ").strip() or 'drone_telemetry'
    region = input("Enter AWS region [us-east-1]: ").strip() or 'us-east-1'
    
    print()
    
    # Create table
    if create_table(table_name, region):
        sys.exit(0)
    else:
        sys.exit(1)


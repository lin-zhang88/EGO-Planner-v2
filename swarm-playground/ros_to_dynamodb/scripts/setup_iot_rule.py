#!/usr/bin/env python3
"""
Setup IoT Rule to forward Greengrass messages to DynamoDB
"""

import boto3
import json

AWS_REGION = "us-east-1"
RULE_NAME = "DroneTelemetryToDynamoDB"
TABLE_NAME = "drone_telemetry"

def setup_iot_rule():
    """Create IoT rule to forward messages to DynamoDB"""
    print("=" * 70)
    print("Setting up IoT Core Rule → DynamoDB")
    print("=" * 70)
    
    try:
        iot = boto3.client('iot', region_name=AWS_REGION)
        iam = boto3.client('iam', region_name=AWS_REGION)
        sts = boto3.client('sts', region_name=AWS_REGION)
        
        # Get account ID
        account_id = sts.get_caller_identity()['Account']
        print(f"Account ID: {account_id}")
        
        # Step 1: Create IAM role for IoT
        role_name = "IoTDynamoDBRole"
        trust_policy = {
            "Version": "2012-10-17",
            "Statement": [
                {
                    "Effect": "Allow",
                    "Principal": {
                        "Service": "iot.amazonaws.com"
                    },
                    "Action": "sts:AssumeRole"
                }
            ]
        }
        
        print(f"\n📝 Creating IAM role: {role_name}")
        try:
            role_response = iam.create_role(
                RoleName=role_name,
                AssumeRolePolicyDocument=json.dumps(trust_policy),
                Description="Role for IoT to write to DynamoDB"
            )
            role_arn = role_response['Role']['Arn']
            print(f"✓ Created role: {role_arn}")
        except iam.exceptions.EntityAlreadyExistsException:
            role_arn = f"arn:aws:iam::{account_id}:role/{role_name}"
            print(f"✓ Role already exists: {role_arn}")
        
        # Step 2: Attach DynamoDB policy to role
        policy_document = {
            "Version": "2012-10-17",
            "Statement": [
                {
                    "Effect": "Allow",
                    "Action": [
                        "dynamodb:PutItem",
                        "dynamodb:UpdateItem"
                    ],
                    "Resource": f"arn:aws:dynamodb:{AWS_REGION}:{account_id}:table/{TABLE_NAME}"
                }
            ]
        }
        
        policy_name = "IoTDynamoDBPolicy"
        print(f"\n📝 Creating/updating policy: {policy_name}")
        try:
            iam.put_role_policy(
                RoleName=role_name,
                PolicyName=policy_name,
                PolicyDocument=json.dumps(policy_document)
            )
            print(f"✓ Policy attached")
        except Exception as e:
            print(f"⚠ Policy error: {e}")
        
        # Step 3: Create IoT rule
        print(f"\n📝 Creating IoT rule: {RULE_NAME}")
        
        rule_payload = {
            "sql": "SELECT * FROM 'drone/telemetry/#'",
            "description": "Forward drone telemetry to DynamoDB",
            "actions": [
                {
                    "dynamoDBv2": {
                        "roleArn": role_arn,
                        "putItem": {
                            "tableName": TABLE_NAME
                        }
                    }
                }
            ],
            "ruleDisabled": False
        }
        
        try:
            # Delete existing rule if it exists
            try:
                iot.delete_topic_rule(ruleName=RULE_NAME)
                print(f"✓ Deleted existing rule")
            except:
                pass
            
            # Create new rule
            iot.create_topic_rule(
                ruleName=RULE_NAME,
                topicRulePayload=rule_payload
            )
            print(f"✓ Created IoT rule: {RULE_NAME}")
        except Exception as e:
            print(f"✗ Rule creation failed: {e}")
            return
        
        print("\n" + "=" * 70)
        print("🎉 Setup complete!")
        print("=" * 70)
        print(f"IoT Rule: {RULE_NAME}")
        print(f"Topic: drone/telemetry/#")
        print(f"Destination: DynamoDB table '{TABLE_NAME}'")
        print("\nNow run your streamer and data will flow to DynamoDB!")
        
    except Exception as e:
        print(f"\n✗ Setup failed: {e}")
        print("\nTroubleshooting:")
        print("1. Check AWS credentials")
        print("2. Verify IAM permissions")
        print("3. Ensure DynamoDB table exists")

if __name__ == '__main__':
    setup_iot_rule()


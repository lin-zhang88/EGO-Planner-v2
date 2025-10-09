#!/usr/bin/env python3
"""
Verify AWS Swarm Coordination Setup
Checks all components: Lambda, IoT Rules, IAM Roles, DynamoDB
"""

import boto3
import json
from datetime import datetime

AWS_REGION = "us-east-1"
LAMBDA_FUNCTION_NAME = "DroneSwarmCoordinator"
IOT_RULE_NAME = "DroneSwarmCoordinationRule"
TABLE_NAME = "drone_telemetry"

def verify_setup():
    """Verify all AWS components are set up correctly"""
    print("=" * 70)
    print("🔍 Verifying AWS Swarm Coordination Setup")
    print("=" * 70)
    print()
    
    all_checks_passed = True
    
    try:
        # Initialize clients
        lambda_client = boto3.client('lambda', region_name=AWS_REGION)
        iot = boto3.client('iot', region_name=AWS_REGION)
        iam = boto3.client('iam', region_name=AWS_REGION)
        dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)
        logs = boto3.client('logs', region_name=AWS_REGION)
        sts = boto3.client('sts', region_name=AWS_REGION)
        
        account_id = sts.get_caller_identity()['Account']
        
        # Check 1: Lambda Function
        print("1️⃣  Checking Lambda Function...")
        try:
            lambda_response = lambda_client.get_function(FunctionName=LAMBDA_FUNCTION_NAME)
            print(f"   ✅ Lambda exists: {LAMBDA_FUNCTION_NAME}")
            print(f"   📦 Runtime: {lambda_response['Configuration']['Runtime']}")
            print(f"   💾 Memory: {lambda_response['Configuration']['MemorySize']} MB")
            print(f"   ⏱️  Timeout: {lambda_response['Configuration']['Timeout']} seconds")
            print(f"   🔗 ARN: {lambda_response['Configuration']['FunctionArn']}")
            
            # Check Lambda permissions
            try:
                policy = lambda_client.get_policy(FunctionName=LAMBDA_FUNCTION_NAME)
                print(f"   ✅ Lambda has IoT invoke permission")
            except:
                print(f"   ⚠️  Lambda may not have IoT invoke permission")
        except lambda_client.exceptions.ResourceNotFoundException:
            print(f"   ❌ Lambda NOT found: {LAMBDA_FUNCTION_NAME}")
            all_checks_passed = False
        except Exception as e:
            print(f"   ❌ Lambda check failed: {e}")
            all_checks_passed = False
        print()
        
        # Check 2: IoT Rule
        print("2️⃣  Checking IoT Core Rule...")
        try:
            rule = iot.get_topic_rule(ruleName=IOT_RULE_NAME)
            print(f"   ✅ IoT Rule exists: {IOT_RULE_NAME}")
            print(f"   📝 SQL: {rule['rule']['sql']}")
            print(f"   🎯 Actions: {len(rule['rule']['actions'])} action(s)")
            print(f"   🔛 Status: {'Enabled' if not rule['rule'].get('ruleDisabled', False) else 'Disabled'}")
            
            # Show actions
            for i, action in enumerate(rule['rule']['actions'], 1):
                if 'lambda' in action:
                    print(f"   📤 Action {i}: Lambda → {action['lambda']['functionArn'].split(':')[-1]}")
                elif 'dynamoDBv2' in action:
                    print(f"   📤 Action {i}: DynamoDB → {action['dynamoDBv2']['putItem']['tableName']}")
        except iot.exceptions.ResourceNotFoundException:
            print(f"   ❌ IoT Rule NOT found: {IOT_RULE_NAME}")
            all_checks_passed = False
        except Exception as e:
            print(f"   ❌ IoT Rule check failed: {e}")
            all_checks_passed = False
        print()
        
        # Check 3: IAM Roles
        print("3️⃣  Checking IAM Roles...")
        roles_to_check = [
            ("DroneSwarmLambdaRole", "Lambda execution"),
            ("IoTLambdaInvokeRole", "IoT rule to invoke Lambda")
        ]
        
        for role_name, purpose in roles_to_check:
            try:
                role = iam.get_role(RoleName=role_name)
                print(f"   ✅ Role exists: {role_name} ({purpose})")
                
                # Check attached policies
                attached_policies = iam.list_attached_role_policies(RoleName=role_name)
                if attached_policies['AttachedPolicies']:
                    print(f"      📋 Attached policies: {len(attached_policies['AttachedPolicies'])}")
                    for policy in attached_policies['AttachedPolicies']:
                        print(f"         • {policy['PolicyName']}")
            except iam.exceptions.NoSuchEntityException:
                print(f"   ❌ Role NOT found: {role_name}")
                all_checks_passed = False
            except Exception as e:
                print(f"   ⚠️  Role check error for {role_name}: {e}")
        print()
        
        # Check 4: DynamoDB Table
        print("4️⃣  Checking DynamoDB Table...")
        try:
            table = dynamodb.Table(TABLE_NAME)
            table.load()
            print(f"   ✅ Table exists: {TABLE_NAME}")
            print(f"   📊 Status: {table.table_status}")
            print(f"   🔑 Keys: {table.key_schema}")
            print(f"   📈 Item count: {table.item_count}")
            
            # Try to get recent items
            response = table.scan(Limit=5)
            if response['Items']:
                print(f"   ✅ Contains data: {len(response['Items'])} recent items")
                latest = response['Items'][0]
                if 'drone_id' in latest:
                    print(f"      Latest: {latest.get('drone_id')} at {latest.get('timestamp', 'N/A')}")
            else:
                print(f"   ⚠️  Table is empty (no data yet)")
        except Exception as e:
            print(f"   ❌ DynamoDB check failed: {e}")
            all_checks_passed = False
        print()
        
        # Check 5: CloudWatch Logs
        print("5️⃣  Checking CloudWatch Logs...")
        try:
            log_group_name = f"/aws/lambda/{LAMBDA_FUNCTION_NAME}"
            log_group = logs.describe_log_groups(logGroupNamePrefix=log_group_name)
            
            if log_group['logGroups']:
                print(f"   ✅ Log group exists: {log_group_name}")
                
                # Try to get recent log streams
                streams = logs.describe_log_streams(
                    logGroupName=log_group_name,
                    orderBy='LastEventTime',
                    descending=True,
                    limit=1
                )
                
                if streams['logStreams']:
                    latest_stream = streams['logStreams'][0]
                    last_event = datetime.fromtimestamp(latest_stream['lastEventTimestamp'] / 1000)
                    print(f"   📝 Latest log: {last_event.strftime('%Y-%m-%d %H:%M:%S')}")
                    print(f"   📊 Log streams: {len(streams['logStreams'])}")
                else:
                    print(f"   ⚠️  No log streams yet (Lambda hasn't been invoked)")
            else:
                print(f"   ⚠️  Log group not found (Lambda hasn't been invoked)")
        except Exception as e:
            print(f"   ⚠️  CloudWatch check: {e}")
        print()
        
        # Check 6: IoT Endpoint
        print("6️⃣  Checking IoT Endpoint...")
        try:
            endpoint = iot.describe_endpoint(endpointType='iot:Data-ATS')
            print(f"   ✅ IoT Endpoint: {endpoint['endpointAddress']}")
            print(f"   📡 Region: {AWS_REGION}")
        except Exception as e:
            print(f"   ❌ IoT Endpoint check failed: {e}")
            all_checks_passed = False
        print()
        
        # Summary
        print("=" * 70)
        if all_checks_passed:
            print("🎉 All checks PASSED! System is ready.")
            print()
            print("Next steps:")
            print("1. Start your drone simulation")
            print("2. Run: python3 swarm_aware_stream.py /drone_0_pcl_render_node/cloud drone_0")
            print("3. Subscribe to 'drone/swarm/state' in IoT Core MQTT test client")
            print("4. Watch real-time swarm coordination!")
        else:
            print("⚠️  Some checks FAILED. Please run setup again:")
            print("   python3 setup_swarm_coordination.py")
        print("=" * 70)
        
    except Exception as e:
        print(f"\n❌ Verification failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    verify_setup()


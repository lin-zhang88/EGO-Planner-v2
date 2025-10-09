#!/usr/bin/env python3
"""
Real-time Drone Data Visualization
Simple web dashboard for viewing drone telemetry data
"""

import boto3
import time
import json
from datetime import datetime, timedelta
from decimal import Decimal


class DroneDataVisualizer:
    def __init__(self, region='us-east-1'):
        self.region = region
        self.dynamodb = boto3.resource('dynamodb', region_name=region)
        self.table = self.dynamodb.Table('drone_telemetry')
        
    def get_latest_data(self, drone_id=None, limit=10):
        """Get latest drone data"""
        try:
            if drone_id:
                response = self.table.query(
                    KeyConditionExpression=boto3.dynamodb.conditions.Key('drone_id').eq(drone_id),
                    ScanIndexForward=False,  # Sort descending
                    Limit=limit
                )
            else:
                # Scan for latest data from any drone
                response = self.table.scan(
                    Limit=limit
                )
                # Sort by timestamp manually (scan doesn't support sorting)
                items = sorted(response['Items'], 
                             key=lambda x: float(x['timestamp']), 
                             reverse=True)[:limit]
                response['Items'] = items
            
            return response['Items']
            
        except Exception as e:
            print(f"Error getting data: {e}")
            return []
    
    def get_drone_stats(self):
        """Get statistics for all drones"""
        try:
            # Get data from last hour
            one_hour_ago = datetime.now() - timedelta(hours=1)
            timestamp_threshold = one_hour_ago.timestamp()
            
            stats = {}
            
            # Scan for recent data
            response = self.table.scan(
                FilterExpression=boto3.dynamodb.conditions.Attr('timestamp').gte(Decimal(str(timestamp_threshold)))
            )
            
            for item in response['Items']:
                drone_id = item['drone_id']
                
                if drone_id not in stats:
                    stats[drone_id] = {
                        'total_messages': 0,
                        'total_points': 0,
                        'avg_point_count': 0,
                        'last_seen': None,
                        'message_types': set()
                    }
                
                stats[drone_id]['total_messages'] += 1
                stats[drone_id]['total_points'] += int(item['point_count'])
                stats[drone_id]['message_types'].add(item['message_type'])
                
                # Update last seen
                timestamp = float(item['timestamp'])
                if stats[drone_id]['last_seen'] is None or timestamp > stats[drone_id]['last_seen']:
                    stats[drone_id]['last_seen'] = timestamp
            
            # Calculate averages
            for drone_id in stats:
                if stats[drone_id]['total_messages'] > 0:
                    stats[drone_id]['avg_point_count'] = stats[drone_id]['total_points'] / stats[drone_id]['total_messages']
                stats[drone_id]['message_types'] = list(stats[drone_id]['message_types'])
            
            return stats
            
        except Exception as e:
            print(f"Error getting stats: {e}")
            return {}
    
    def print_dashboard(self):
        """Print a simple text dashboard"""
        
        print("\n" + "="*80)
        print("🚁 REAL-TIME DRONE TELEMETRY DASHBOARD")
        print("="*80)
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Get statistics
        stats = self.get_drone_stats()
        
        if not stats:
            print("📡 No recent data found. Make sure drones are streaming data.")
            return
        
        print(f"\n📊 DRONE STATISTICS (Last Hour)")
        print("-"*50)
        
        for drone_id, data in stats.items():
            last_seen = datetime.fromtimestamp(data['last_seen']).strftime('%H:%M:%S') if data['last_seen'] else 'Never'
            
            print(f"🚁 {drone_id.upper()}")
            print(f"   Messages: {data['total_messages']}")
            print(f"   Total Points: {data['total_points']:,}")
            print(f"   Avg Points/Message: {data['avg_point_count']:.1f}")
            print(f"   Message Types: {', '.join(data['message_types'])}")
            print(f"   Last Seen: {last_seen}")
            print()
        
        # Get latest messages
        print("📡 LATEST MESSAGES")
        print("-"*50)
        
        latest_data = self.get_latest_data(limit=5)
        
        for i, item in enumerate(latest_data, 1):
            timestamp = datetime.fromtimestamp(float(item['timestamp'])).strftime('%H:%M:%S')
            drone_id = item['drone_id']
            point_count = int(item['point_count'])
            frame_id = item['frame_id']
            
            print(f"{i}. [{timestamp}] {drone_id} | {point_count:,} points | {frame_id}")
        
        print("\n" + "="*80)
    
    def monitor_realtime(self, refresh_interval=5):
        """Monitor data in real-time"""
        
        print("🚁 Starting real-time drone monitoring...")
        print("Press Ctrl+C to stop")
        print()
        
        try:
            while True:
                # Clear screen (works on most terminals)
                print("\033[2J\033[H", end="")
                
                # Print dashboard
                self.print_dashboard()
                
                print(f"\n⏱️  Refreshing every {refresh_interval} seconds...")
                print("Press Ctrl+C to stop monitoring")
                
                time.sleep(refresh_interval)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Monitoring stopped.")
    
    def export_data(self, drone_id=None, hours=1):
        """Export data to JSON file"""
        
        try:
            # Calculate time threshold
            cutoff_time = datetime.now() - timedelta(hours=hours)
            timestamp_threshold = cutoff_time.timestamp()
            
            # Get data
            if drone_id:
                response = self.table.query(
                    KeyConditionExpression=boto3.dynamodb.conditions.Key('drone_id').eq(drone_id)
                )
            else:
                response = self.table.scan()
            
            # Filter by time
            recent_data = []
            for item in response['Items']:
                if float(item['timestamp']) >= timestamp_threshold:
                    # Convert Decimal to float for JSON serialization
                    item_json = json.loads(json.dumps(item, default=str))
                    recent_data.append(item_json)
            
            # Save to file
            filename = f"drone_data_{drone_id or 'all'}_{int(time.time())}.json"
            with open(filename, 'w') as f:
                json.dump(recent_data, f, indent=2, default=str)
            
            print(f"✅ Exported {len(recent_data)} records to {filename}")
            
        except Exception as e:
            print(f"❌ Error exporting data: {e}")


def main():
    """Main function"""
    
    print("🚁 Drone Telemetry Visualizer")
    print("="*40)
    
    # Initialize visualizer
    viz = DroneDataVisualizer()
    
    while True:
        print("\nOptions:")
        print("1. View real-time dashboard")
        print("2. Get latest data")
        print("3. Get drone statistics")
        print("4. Export data to JSON")
        print("5. Monitor real-time (auto-refresh)")
        print("0. Exit")
        
        choice = input("\nChoose option: ").strip()
        
        if choice == '1':
            viz.print_dashboard()
        elif choice == '2':
            drone_id = input("Enter drone ID (or press Enter for all): ").strip() or None
            limit = int(input("Number of records [10]: ").strip() or "10")
            
            data = viz.get_latest_data(drone_id, limit)
            for item in data:
                timestamp = datetime.fromtimestamp(float(item['timestamp'])).strftime('%Y-%m-%d %H:%M:%S')
                print(f"[{timestamp}] {item['drone_id']} | {item['point_count']} points | {item['frame_id']}")
        
        elif choice == '3':
            stats = viz.get_drone_stats()
            for drone_id, data in stats.items():
                print(f"{drone_id}: {data['total_messages']} messages, {data['total_points']:,} total points")
        
        elif choice == '4':
            drone_id = input("Enter drone ID (or press Enter for all): ").strip() or None
            hours = int(input("Hours of data to export [1]: ").strip() or "1")
            viz.export_data(drone_id, hours)
        
        elif choice == '5':
            interval = int(input("Refresh interval in seconds [5]: ").strip() or "5")
            viz.monitor_realtime(interval)
        
        elif choice == '0':
            print("👋 Goodbye!")
            break
        
        else:
            print("Invalid option. Please try again.")


if __name__ == '__main__':
    main()

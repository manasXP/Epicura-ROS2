#!/usr/bin/env python3
"""
test_cooking_node.py
Test script for Epicura Cooking ROS2 Node

This script demonstrates how to interact with the cooking node via ROS2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
import json
import time


class CookingNodeTester(Node):
    """Test node for interacting with the Epicura cooking node."""
    
    def __init__(self):
        super().__init__('cooking_node_tester')
        
        # Publishers for sending commands
        self.recipe_pub = self.create_publisher(
            String,
            '/epicura/start_recipe',
            10
        )
        
        self.stop_pub = self.create_publisher(
            String,
            '/epicura/stop_cooking',
            10
        )
        
        # Subscribers for monitoring
        self.status_sub = self.create_subscription(
            String,
            '/epicura/cooking_status',
            self.status_callback,
            10
        )
        
        self.progress_sub = self.create_subscription(
            Float32,
            '/epicura/cooking_progress',
            self.progress_callback,
            10
        )
        
        self.temp_sub = self.create_subscription(
            Float32,
            '/epicura/temperature',
            self.temp_callback,
            10
        )
        
        self.log_sub = self.create_subscription(
            String,
            '/epicura/logs',
            self.log_callback,
            10
        )
        
        # State
        self.current_progress = 0.0
        self.current_temp = 25.0
        self.is_cooking = False
        
        self.get_logger().info('🧪 Cooking Node Tester initialized')
    
    def status_callback(self, msg: String):
        """Handle cooking status updates."""
        try:
            status = json.loads(msg.data)
            self.is_cooking = status.get('is_cooking', False)
            self.get_logger().info(f'📊 Status: {status}')
        except json.JSONDecodeError:
            pass
    
    def progress_callback(self, msg: Float32):
        """Handle progress updates."""
        self.current_progress = msg.data
    
    def temp_callback(self, msg: Float32):
        """Handle temperature updates."""
        self.current_temp = msg.data
    
    def log_callback(self, msg: String):
        """Handle log messages."""
        self.get_logger().info(f'📝 Log: {msg.data}')
    
    def send_test_recipe(self):
        """Send a test recipe to the cooking node."""
        recipe = {
            "id": "test_001",
            "name": "ROS2 Test Recipe",
            "cooking_segments": [
                {
                    "segment_id": 1,
                    "duration": 10
                },
                {
                    "segment_id": 2,
                    "duration": 15
                }
            ]
        }
        
        msg = String()
        msg.data = json.dumps(recipe)
        
        self.get_logger().info('📤 Sending test recipe...')
        self.recipe_pub.publish(msg)
        self.get_logger().info('✅ Test recipe sent')
    
    def send_stop_command(self):
        """Send stop command to the cooking node."""
        msg = String()
        msg.data = 'stop'
        
        self.get_logger().info('🛑 Sending stop command...')
        self.stop_pub.publish(msg)
        self.get_logger().info('✅ Stop command sent')


def main(args=None):
    """Main function for the tester."""
    rclpy.init(args=args)
    
    tester = CookingNodeTester()
    
    try:
        # Wait for node to be ready
        tester.get_logger().info('⏳ Waiting for cooking node to be ready...')
        time.sleep(2)
        
        # Send test recipe
        tester.get_logger().info('\n' + '='*60)
        tester.get_logger().info('🧪 TEST 1: Starting test recipe')
        tester.get_logger().info('='*60)
        tester.send_test_recipe()
        
        # Monitor for a while
        tester.get_logger().info('\n📊 Monitoring for 30 seconds...\n')
        start_time = time.time()
        
        while time.time() - start_time < 30:
            rclpy.spin_once(tester, timeout_sec=0.1)
            
            # Print status every 5 seconds
            if int(time.time() - start_time) % 5 == 0:
                tester.get_logger().info(
                    f'Progress: {tester.current_progress:.1f}% | '
                    f'Temp: {tester.current_temp:.1f}°C | '
                    f'Cooking: {tester.is_cooking}'
                )
                time.sleep(1)  # Prevent multiple prints
        
        # Test stop command
        if tester.is_cooking:
            tester.get_logger().info('\n' + '='*60)
            tester.get_logger().info('🧪 TEST 2: Sending stop command')
            tester.get_logger().info('='*60)
            tester.send_stop_command()
            
            # Wait for stop to take effect
            time.sleep(2)
        
        tester.get_logger().info('\n✅ All tests completed')
        
    except KeyboardInterrupt:
        tester.get_logger().info('⚠️  Tests interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'❌ Test error: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

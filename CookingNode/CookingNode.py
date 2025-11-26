"""
CookingNode.py
Epicura Cooking ROS2 Node - ROS2 Humble interface for the autonomous cooking system
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist
import json
import time
from typing import Optional

# Import Epicura cooking system components
from RecipeModel import Recipe, CookingSegment, IngredientType, TemperatureProfileType, StirringProfileType
from CookingBot import CookingBot
from CookingScheduler import CookingScheduler


class CookingNode(Node):
    """
    ROS2 Node for the Epicura autonomous cooking system.
    
    Provides ROS2 interface for:
    - Starting/stopping recipes
    - Monitoring cooking status
    - Hardware control and monitoring
    - Emergency stop
    - Parameter configuration
    """
    
    def __init__(self):
        """Initialize the CookingNode."""
        super().__init__('epicura_cooking_node')
        
        # Declare ROS2 parameters
        self._declare_parameters()
        
        # Initialize cooking components
        self.cooking_bot = CookingBot()
        self.scheduler = CookingScheduler(self.cooking_bot)
        
        # State tracking
        self.current_recipe_json = None
        
        # Create QoS profiles
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self._create_publishers()
        
        # Subscribers
        self._create_subscribers()
        
        # Services
        # TODO: Add service servers for recipe management
        
        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Initialize hardware
        self.cooking_bot.initialize_hardware()
        
        self.get_logger().info('🍳 Epicura Cooking Node initialized successfully')
        self.get_logger().info(f'   Node name: {self.get_name()}')
        self.get_logger().info(f'   Namespace: {self.get_namespace()}')
    
    def _declare_parameters(self):
        """Declare ROS2 parameters for the cooking node."""
        # Status update rate
        self.declare_parameter('status_update_rate', 1.0)
        
        # Hardware configuration
        self.declare_parameter('enable_hardware', True)
        self.declare_parameter('spi_bus', 0)
        
        # Safety parameters
        self.declare_parameter('max_temperature', 250)  # Celsius
        self.declare_parameter('enable_lpg_monitoring', True)
        self.declare_parameter('emergency_stop_on_lpg_leak', True)
        
        # Stirring parameters
        self.declare_parameter('max_stirring_rpm', 500)
        
        # Dispenser parameters
        self.declare_parameter('asd_cartridge_count', 16)
        self.declare_parameter('liquid_dispenser_flow_rate', 100.0)  # ml/s
        
        self.get_logger().info('📋 Parameters declared')
    
    def _create_publishers(self):
        """Create ROS2 publishers for status and monitoring."""
        # Cooking status
        self.status_pub = self.create_publisher(
            String,
            'epicura/cooking_status',
            self.qos_profile
        )
        
        # Progress
        self.progress_pub = self.create_publisher(
            Float32,
            'epicura/cooking_progress',
            self.qos_profile
        )
        
        # Current segment
        self.segment_pub = self.create_publisher(
            Int32,
            'epicura/current_segment',
            self.qos_profile
        )
        
        # Temperature
        self.temperature_pub = self.create_publisher(
            Float32,
            'epicura/temperature',
            self.qos_profile
        )
        
        # Stirring status
        self.stirring_pub = self.create_publisher(
            Bool,
            'epicura/is_stirring',
            self.qos_profile
        )
        
        # RPM
        self.rpm_pub = self.create_publisher(
            Int32,
            'epicura/stirring_rpm',
            self.qos_profile
        )
        
        # Remaining time
        self.remaining_time_pub = self.create_publisher(
            Float32,
            'epicura/remaining_time',
            self.qos_profile
        )
        
        # Logs
        self.log_pub = self.create_publisher(
            String,
            'epicura/logs',
            self.qos_profile
        )
        
        self.get_logger().info('📡 Publishers created')
    
    def _create_subscribers(self):
        """Create ROS2 subscribers for commands."""
        # Recipe command (JSON string)
        self.recipe_sub = self.create_subscription(
            String,
            'epicura/start_recipe',
            self.recipe_callback,
            self.qos_profile
        )
        
        # Stop command
        self.stop_sub = self.create_subscription(
            String,
            'epicura/stop_cooking',
            self.stop_callback,
            self.qos_profile
        )
        
        # Emergency stop
        self.emergency_stop_sub = self.create_subscription(
            String,
            'epicura/emergency_stop',
            self.emergency_stop_callback,
            self.qos_profile
        )
        
        self.get_logger().info('📥 Subscribers created')
    
    def recipe_callback(self, msg: String):
        """
        Handle incoming recipe command.
        
        Args:
            msg: String message containing recipe JSON
        """
        try:
            self.get_logger().info('📨 Received recipe command')
            
            # Parse recipe JSON
            recipe_dict = json.loads(msg.data)
            recipe = self._parse_recipe_from_dict(recipe_dict)
            
            if recipe is None:
                self.get_logger().error('❌ Failed to parse recipe')
                return
            
            # Start cooking
            self.get_logger().info(f'🍳 Starting recipe: {recipe.name}')
            self.scheduler.cook_recipe(
                recipe,
                on_segment_start=self._on_segment_start,
                on_segment_complete=self._on_segment_complete,
                on_recipe_complete=self._on_recipe_complete,
                on_error=self._on_error
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ Error starting recipe: {e}')
    
    def stop_callback(self, msg: String):
        """
        Handle stop cooking command.
        
        Args:
            msg: String message (content ignored)
        """
        self.get_logger().info('🛑 Received stop command')
        self.scheduler.stop_cooking()
        self.get_logger().info('✅ Cooking stopped')
    
    def emergency_stop_callback(self, msg: String):
        """
        Handle emergency stop command.
        
        Args:
            msg: String message (content ignored)
        """
        self.get_logger().warn('🚨 EMERGENCY STOP activated')
        self.scheduler.stop_cooking()
        self.cooking_bot.emergency_stop()
        self.get_logger().warn('✅ Emergency stop complete')
    
    def publish_status(self):
        """Publish cooking status periodically."""
        # Get status from scheduler
        status = self.scheduler.get_status()
        hardware_status = self.cooking_bot.get_hardware_status()
        
        # Publish individual status topics
        if status['is_cooking']:
            # Status JSON
            status_msg = String()
            status_msg.data = json.dumps({
                'is_cooking': status['is_cooking'],
                'recipe_name': status['current_recipe'],
                'segment': status['current_segment'],
                'segment_index': status['current_segment_index'],
                'elapsed_time': status['elapsed_time'],
                'remaining_time': status['remaining_time'],
                'progress_percent': status['progress_percent']
            })
            self.status_pub.publish(status_msg)
            
            # Progress
            progress_msg = Float32()
            progress_msg.data = float(status['progress_percent'])
            self.progress_pub.publish(progress_msg)
            
            # Current segment
            if status['current_segment'] is not None:
                segment_msg = Int32()
                segment_msg.data = status['current_segment']
                self.segment_pub.publish(segment_msg)
            
            # Remaining time
            remaining_msg = Float32()
            remaining_msg.data = float(status['remaining_time'])
            self.remaining_time_pub.publish(remaining_msg)
        
        # Hardware status (published always)
        # Temperature
        temp_msg = Float32()
        temp_msg.data = float(hardware_status['current_temperature'])
        self.temperature_pub.publish(temp_msg)
        
        # Stirring
        stirring_msg = Bool()
        stirring_msg.data = hardware_status['is_stirring']
        self.stirring_pub.publish(stirring_msg)
        
        # RPM
        rpm_msg = Int32()
        rpm_msg.data = hardware_status['stirring_rpm']
        self.rpm_pub.publish(rpm_msg)
    
    def _parse_recipe_from_dict(self, recipe_dict: dict) -> Optional[Recipe]:
        """
        Parse a Recipe object from a dictionary.
        
        Args:
            recipe_dict: Dictionary containing recipe data
            
        Returns:
            Recipe object or None if parsing fails
        """
        try:
            # This is a simplified parser - you should expand this
            # to handle the full Recipe model structure
            
            # For now, we'll assume the recipe_dict matches the Recipe structure
            # In production, you'd want more robust parsing
            
            segments = []
            for seg_dict in recipe_dict.get('cooking_segments', []):
                segment = CookingSegment(
                    segment_id=seg_dict['segment_id'],
                    duration=seg_dict['duration'],
                    ingredients={},  # TODO: Parse ingredients
                    ISC=[],  # TODO: Parse temperature profiles
                    stirring_profile=[]  # TODO: Parse stirring profiles
                )
                segments.append(segment)
            
            recipe = Recipe(
                id=recipe_dict.get('id'),
                name=recipe_dict['name'],
                cooking_segments=segments
            )
            
            return recipe
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to parse recipe: {e}')
            return None
    
    # Callback functions for cooking events
    def _on_segment_start(self, segment: CookingSegment, idx: int):
        """Called when a segment starts."""
        self.get_logger().info(f'🎯 Segment {idx + 1} started (ID: {segment.segment_id})')
        
        log_msg = String()
        log_msg.data = f'Segment {idx + 1} started'
        self.log_pub.publish(log_msg)
    
    def _on_segment_complete(self, segment: CookingSegment, idx: int):
        """Called when a segment completes."""
        self.get_logger().info(f'✅ Segment {idx + 1} completed (ID: {segment.segment_id})')
        
        log_msg = String()
        log_msg.data = f'Segment {idx + 1} completed'
        self.log_pub.publish(log_msg)
    
    def _on_recipe_complete(self, recipe: Recipe):
        """Called when entire recipe is complete."""
        self.get_logger().info(f'🎉 Recipe "{recipe.name}" completed successfully!')
        
        log_msg = String()
        log_msg.data = f'Recipe "{recipe.name}" completed'
        self.log_pub.publish(log_msg)
    
    def _on_error(self, error: Exception):
        """Called if an error occurs during cooking."""
        self.get_logger().error(f'❌ Cooking error: {error}')
        
        log_msg = String()
        log_msg.data = f'Error: {str(error)}'
        self.log_pub.publish(log_msg)


def main(args=None):
    """
    Main function to start the CookingNode.
    
    Args:
        args: Command line arguments (passed to rclpy.init)
    """
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create the node
        cooking_node = CookingNode()
        
        # Spin the node
        rclpy.spin(cooking_node)
        
    except KeyboardInterrupt:
        cooking_node.get_logger().info('⚠️  Keyboard interrupt detected')
    except Exception as e:
        if 'cooking_node' in locals():
            cooking_node.get_logger().error(f'❌ Fatal error: {e}')
    finally:
        # Cleanup
        if 'cooking_node' in locals():
            cooking_node.get_logger().info('🛑 Shutting down Epicura Cooking Node')
            cooking_node.destroy_node()
        
        # Shutdown ROS2
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

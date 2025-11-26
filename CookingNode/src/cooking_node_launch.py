"""
cooking_node_launch.py
Launch file for Epicura Cooking Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for the Epicura cooking node."""
    
    # Declare launch arguments
    status_update_rate_arg = DeclareLaunchArgument(
        'status_update_rate',
        default_value='1.0',
        description='Rate at which status updates are published (Hz)'
    )
    
    enable_hardware_arg = DeclareLaunchArgument(
        'enable_hardware',
        default_value='true',
        description='Enable actual hardware control'
    )
    
    max_temperature_arg = DeclareLaunchArgument(
        'max_temperature',
        default_value='250',
        description='Maximum temperature limit in Celsius'
    )
    
    max_stirring_rpm_arg = DeclareLaunchArgument(
        'max_stirring_rpm',
        default_value='500',
        description='Maximum stirring RPM'
    )
    
    # Create the cooking node
    cooking_node = Node(
        package='epicura_cooking',
        executable='cooking_node',
        name='epicura_cooking_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'status_update_rate': LaunchConfiguration('status_update_rate'),
            'enable_hardware': LaunchConfiguration('enable_hardware'),
            'max_temperature': LaunchConfiguration('max_temperature'),
            'max_stirring_rpm': LaunchConfiguration('max_stirring_rpm'),
            'spi_bus': 0,
            'enable_lpg_monitoring': True,
            'emergency_stop_on_lpg_leak': True,
            'asd_cartridge_count': 16,
            'liquid_dispenser_flow_rate': 100.0,
        }]
    )
    
    # Log startup info
    startup_info = LogInfo(
        msg='🍳 Launching Epicura Cooking Node...'
    )
    
    return LaunchDescription([
        # Declare arguments
        status_update_rate_arg,
        enable_hardware_arg,
        max_temperature_arg,
        max_stirring_rpm_arg,
        
        # Log startup
        startup_info,
        
        # Launch node
        cooking_node,
    ])

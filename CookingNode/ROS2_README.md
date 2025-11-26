# Epicura Cooking System - ROS2 Integration

ROS2 Humble integration for the Epicura autonomous cooking system.

## Overview

This package provides a ROS2 interface for the Epicura cooking system, enabling:
- Remote recipe execution
- Real-time status monitoring
- Hardware control via ROS2 topics
- Emergency stop capabilities
- Parameter configuration
- Integration with ROS2 ecosystem

## Prerequisites

### System Requirements
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS2 Humble Hawksbill
- Python 3.10+

### Install ROS2 Humble

Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html

**Quick install (Ubuntu):**
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

## Package Structure

```
epicura_cooking/
├── epicura_cooking/
│   ├── __init__.py
│   ├── CookingNode.py          # Main ROS2 node
│   ├── CookingBot.py            # Hardware control
│   ├── CookingScheduler.py     # Timing and scheduling
│   └── RecipeModel.py          # Data models
├── launch/
│   └── cooking_node_launch.py  # Launch file
├── config/
│   └── cooking_params.yaml     # Parameter configuration
├── resource/
│   └── epicura_cooking         # Package marker
├── package.xml                  # Package metadata
├── setup.py                     # Python package setup
├── setup.cfg                    # Setup configuration
└── README.md                    # This file
```

## Installation

### 1. Create ROS2 Workspace

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/epicura_ws/src
cd ~/epicura_ws/src
```

### 2. Clone/Copy Package

Copy the epicura_cooking package to your workspace:

```bash
cd ~/epicura_ws/src
# Copy your epicura_cooking folder here
```

Your folder structure should be:
```
~/epicura_ws/
└── src/
    └── epicura_cooking/
        ├── epicura_cooking/
        │   ├── __init__.py
        │   ├── CookingNode.py
        │   ├── CookingBot.py
        │   ├── CookingScheduler.py
        │   └── RecipeModel.py
        ├── launch/
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

### 3. Install Dependencies

```bash
cd ~/epicura_ws
rosdep install -i --from-path src --rosdistro humble -y
```

### 4. Build Package

```bash
cd ~/epicura_ws
colcon build --packages-select epicura_cooking
```

### 5. Source the Workspace

```bash
source ~/epicura_ws/install/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/epicura_ws/install/setup.bash" >> ~/.bashrc
```

## Usage

### Starting the Node

**Method 1: Using ros2 run**
```bash
ros2 run epicura_cooking cooking_node
```

**Method 2: Using launch file (recommended)**
```bash
ros2 launch epicura_cooking cooking_node_launch.py
```

**With custom parameters:**
```bash
ros2 launch epicura_cooking cooking_node_launch.py \
    status_update_rate:=2.0 \
    max_temperature:=300 \
    enable_hardware:=false
```

### ROS2 Topics

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/epicura/cooking_status` | `std_msgs/String` | JSON status of cooking process |
| `/epicura/cooking_progress` | `std_msgs/Float32` | Progress percentage (0-100) |
| `/epicura/current_segment` | `std_msgs/Int32` | Current segment ID |
| `/epicura/temperature` | `std_msgs/Float32` | Current temperature (°C) |
| `/epicura/is_stirring` | `std_msgs/Bool` | Stirring status |
| `/epicura/stirring_rpm` | `std_msgs/Int32` | Current stirring RPM |
| `/epicura/remaining_time` | `std_msgs/Float32` | Remaining time (seconds) |
| `/epicura/logs` | `std_msgs/String` | Cooking event logs |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/epicura/start_recipe` | `std_msgs/String` | Start recipe (JSON format) |
| `/epicura/stop_cooking` | `std_msgs/String` | Stop current cooking |
| `/epicura/emergency_stop` | `std_msgs/String` | Emergency stop all systems |

### Monitoring Status

**View all topics:**
```bash
ros2 topic list
```

**Monitor cooking progress:**
```bash
ros2 topic echo /epicura/cooking_progress
```

**Monitor temperature:**
```bash
ros2 topic echo /epicura/temperature
```

**View full status:**
```bash
ros2 topic echo /epicura/cooking_status
```

### Starting a Recipe

**From command line:**
```bash
# Simple example
ros2 topic pub --once /epicura/start_recipe std_msgs/String \
  "{data: '{\"name\": \"Test Recipe\", \"cooking_segments\": []}'}"
```

**From Python script:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# Initialize ROS2
rclpy.init()
node = Node('recipe_publisher')

# Create publisher
pub = node.create_publisher(String, '/epicura/start_recipe', 10)

# Prepare recipe
recipe = {
    "id": "recipe_001",
    "name": "Test Recipe",
    "cooking_segments": [
        {
            "segment_id": 1,
            "duration": 30,
            # ... more fields
        }
    ]
}

# Publish
msg = String()
msg.data = json.dumps(recipe)
pub.publish(msg)

# Cleanup
node.destroy_node()
rclpy.shutdown()
```

### Stopping Cooking

**Normal stop:**
```bash
ros2 topic pub --once /epicura/stop_cooking std_msgs/String "{data: 'stop'}"
```

**Emergency stop:**
```bash
ros2 topic pub --once /epicura/emergency_stop std_msgs/String "{data: 'emergency'}"
```

### Parameters

**List all parameters:**
```bash
ros2 param list /epicura_cooking_node
```

**Get parameter value:**
```bash
ros2 param get /epicura_cooking_node max_temperature
```

**Set parameter:**
```bash
ros2 param set /epicura_cooking_node max_temperature 280
```

**Available parameters:**
- `status_update_rate` (float): Status update frequency in Hz
- `enable_hardware` (bool): Enable/disable hardware control
- `spi_bus` (int): SPI bus number
- `max_temperature` (int): Maximum temperature limit (°C)
- `enable_lpg_monitoring` (bool): Enable LPG leak detection
- `emergency_stop_on_lpg_leak` (bool): Auto-stop on LPG leak
- `max_stirring_rpm` (int): Maximum stirring RPM
- `asd_cartridge_count` (int): Number of spice cartridges
- `liquid_dispenser_flow_rate` (float): Flow rate in ml/s

## ROS2 Node Architecture

```
┌─────────────────────────────────────────────────────────┐
│              Epicura Cooking Node                       │
│                                                         │
│  ┌─────────────────┐      ┌──────────────────┐       │
│  │ CookingScheduler│◄────►│  CookingBot      │       │
│  │  (Timing)       │      │  (Hardware)      │       │
│  └────────┬────────┘      └────────┬─────────┘       │
│           │                         │                  │
│           │         ┌──────────────┴─────────┐       │
│           │         │                         │       │
│  ┌────────▼─────────▼──────┐    ┌────────────▼────┐ │
│  │  ROS2 Publishers         │    │ ROS2 Subscribers│ │
│  │  - Status                │    │ - Start Recipe  │ │
│  │  - Progress              │    │ - Stop          │ │
│  │  - Temperature           │    │ - Emergency     │ │
│  │  - Stirring              │    │                 │ │
│  └──────────────────────────┘    └─────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

## Integration with Other ROS2 Nodes

### Example: External Controller Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import json

class CookingController(Node):
    def __init__(self):
        super().__init__('cooking_controller')
        
        # Subscribe to cooking status
        self.status_sub = self.create_subscription(
            String,
            '/epicura/cooking_status',
            self.status_callback,
            10
        )
        
        # Publish recipe commands
        self.recipe_pub = self.create_publisher(
            String,
            '/epicura/start_recipe',
            10
        )
    
    def status_callback(self, msg):
        status = json.loads(msg.data)
        self.get_logger().info(f'Progress: {status["progress_percent"]}%')
    
    def start_recipe(self, recipe_dict):
        msg = String()
        msg.data = json.dumps(recipe_dict)
        self.recipe_pub.publish(msg)

def main():
    rclpy.init()
    controller = CookingController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visualization with RViz2

While the cooking system doesn't have 3D visualization, you can use `rqt` tools:

```bash
# Plot progress over time
rqt_plot /epicura/cooking_progress /epicura/temperature

# View all topics
rqt_graph

# Monitor topics
rqt_topic
```

## Debugging

### Check Node Status
```bash
ros2 node info /epicura_cooking_node
```

### Monitor Logs
```bash
ros2 run rqt_console rqt_console
```

### Test Communication
```bash
# Check if node is running
ros2 node list

# Echo a topic
ros2 topic echo /epicura/cooking_status

# Check topic frequency
ros2 topic hz /epicura/cooking_progress
```

## Development

### Running Tests
```bash
cd ~/epicura_ws
colcon test --packages-select epicura_cooking
```

### Code Style
```bash
# Check with flake8
flake8 src/epicura_cooking/epicura_cooking/

# Format with black
black src/epicura_cooking/epicura_cooking/
```

## Troubleshooting

### Node Won't Start
```bash
# Check dependencies
rosdep check epicura_cooking

# Rebuild package
cd ~/epicura_ws
colcon build --packages-select epicura_cooking --symlink-install
source install/setup.bash
```

### Topics Not Publishing
```bash
# Verify node is running
ros2 node list | grep epicura

# Check topic list
ros2 topic list | grep epicura
```

### Import Errors
```bash
# Ensure workspace is sourced
source ~/epicura_ws/install/setup.bash

# Verify package is installed
ros2 pkg list | grep epicura
```

## Future Enhancements

- [ ] Custom ROS2 messages for recipes and status
- [ ] ROS2 services for recipe management
- [ ] Action server for long-running cooking operations
- [ ] Integration with MoveIt for robotic arms
- [ ] TF2 transforms for hardware positioning
- [ ] Diagnostics integration for health monitoring
- [ ] ROS2 bag recording for recipe playback
- [ ] Dynamic reconfigure for runtime parameter changes

## Contributing

When adding new features:
1. Update the node's publishers/subscribers as needed
2. Add parameters to `_declare_parameters()`
3. Update this README with new topics/parameters
4. Add launch file arguments if needed

## License

Proprietary - Epicura Autonomous Cooking System

## Support

For issues and questions, contact: manas@epicura.ai

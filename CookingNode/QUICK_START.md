# Epicura Cooking System - Quick Start Guide

Get up and running with the Epicura ROS2 cooking system in minutes.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble installed
- 15 minutes

## 5-Minute Setup

### 1. Install ROS2 (if not already installed)

```bash
# Quick install
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Create Workspace

```bash
mkdir -p ~/epicura_ws/src/epicura_cooking
cd ~/epicura_ws/src/epicura_cooking
```

### 3. Add Files

Place all provided files in the following structure:

```
~/epicura_ws/src/epicura_cooking/
├── epicura_cooking/
│   ├── __init__.py
│   ├── CookingNode.py
│   ├── CookingBot.py
│   ├── CookingScheduler.py
│   └── RecipeModel.py
├── launch/
│   └── cooking_node_launch.py
├── config/
│   └── cooking_params.yaml
├── resource/
│   └── epicura_cooking
├── package.xml
├── setup.py
└── setup.cfg
```

### 4. Build & Run

```bash
cd ~/epicura_ws
colcon build --packages-select epicura_cooking
source install/setup.bash
ros2 launch epicura_cooking cooking_node_launch.py
```

## Quick Commands

### Launch Node
```bash
ros2 launch epicura_cooking cooking_node_launch.py
```

### Check Status
```bash
# List topics
ros2 topic list | grep epicura

# Monitor progress
ros2 topic echo /epicura/cooking_progress

# View temperature
ros2 topic echo /epicura/temperature
```

### Send Test Recipe
```bash
# Simple test
ros2 topic pub --once /epicura/start_recipe std_msgs/String \
  "{data: '{\"name\":\"Test\",\"cooking_segments\":[]}'}"
```

### Stop Cooking
```bash
ros2 topic pub --once /epicura/stop_cooking std_msgs/String "{data: 'stop'}"
```

## Topics Reference

### Published (Status)
- `/epicura/cooking_status` - JSON status
- `/epicura/cooking_progress` - Progress %
- `/epicura/temperature` - Current temp (°C)
- `/epicura/is_stirring` - Stirring on/off
- `/epicura/stirring_rpm` - Current RPM
- `/epicura/remaining_time` - Time left (sec)

### Subscribed (Commands)
- `/epicura/start_recipe` - Start cooking
- `/epicura/stop_cooking` - Stop cooking
- `/epicura/emergency_stop` - Emergency stop

## Common Tasks

### Monitor Everything
```bash
# Terminal 1
ros2 topic echo /epicura/cooking_progress

# Terminal 2
ros2 topic echo /epicura/temperature

# Terminal 3
ros2 topic echo /epicura/logs
```

### View Node Info
```bash
ros2 node info /epicura_cooking_node
```

### Change Parameters
```bash
# List parameters
ros2 param list /epicura_cooking_node

# Get value
ros2 param get /epicura_cooking_node max_temperature

# Set value
ros2 param set /epicura_cooking_node max_temperature 300
```

## Troubleshooting

### Node Won't Start
```bash
cd ~/epicura_ws
colcon build --packages-select epicura_cooking
source install/setup.bash
```

### Import Errors
```bash
# Make sure __init__.py exists
touch ~/epicura_ws/src/epicura_cooking/epicura_cooking/__init__.py

# Rebuild
cd ~/epicura_ws
colcon build --packages-select epicura_cooking --symlink-install
source install/setup.bash
```

### Topics Not Visible
```bash
# Check if node is running
ros2 node list

# Should show: /epicura_cooking_node

# If not, check logs
ros2 run epicura_cooking cooking_node
```

## Test the System

### Run Test Script
```bash
# Terminal 1: Start node
ros2 launch epicura_cooking cooking_node_launch.py

# Terminal 2: Run test
cd ~/epicura_ws/src/epicura_cooking
python3 test_cooking_node.py
```

## Development Mode

For quick iteration during development:

```bash
cd ~/epicura_ws
colcon build --packages-select epicura_cooking --symlink-install
source install/setup.bash
```

With `--symlink-install`, Python file changes don't require rebuild!

## Example Recipe (Python)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

rclpy.init()
node = Node('recipe_sender')
pub = node.create_publisher(String, '/epicura/start_recipe', 10)

recipe = {
    "id": "quick_001",
    "name": "Quick Test",
    "cooking_segments": [
        {"segment_id": 1, "duration": 30}
    ]
}

msg = String()
msg.data = json.dumps(recipe)
pub.publish(msg)

node.destroy_node()
rclpy.shutdown()
```

## Visualization

```bash
# Plot progress and temperature
rqt_plot /epicura/cooking_progress /epicura/temperature

# View node graph
rqt_graph
```

## Next Steps

1. **Read full docs**: See `ROS2_README.md` for detailed usage
2. **Hardware setup**: Configure SPI and hardware interfaces
3. **Create recipes**: Build your recipe database
4. **Customize**: Modify parameters in `cooking_params.yaml`

## Useful Commands Cheat Sheet

```bash
# Build
colcon build --packages-select epicura_cooking

# Source
source install/setup.bash

# Run
ros2 launch epicura_cooking cooking_node_launch.py

# List topics
ros2 topic list

# Echo topic
ros2 topic echo /epicura/cooking_status

# Publish
ros2 topic pub --once /topic_name std_msgs/String "{data: 'value'}"

# Node info
ros2 node info /node_name

# Parameters
ros2 param list /node_name
ros2 param get /node_name param_name
ros2 param set /node_name param_name value
```

## Getting Help

```bash
# ROS2 help
ros2 --help

# Topic help
ros2 topic --help

# Launch help
ros2 launch --help

# Node help
ros2 node --help
```

## Support

- **Installation issues**: See `INSTALL.md`
- **Usage questions**: See `ROS2_README.md`
- **Architecture**: See `README.md`
- **Contact**: manas@epicura.ai

---

**🎉 You're ready to cook with ROS2!**

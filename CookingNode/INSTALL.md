# Epicura Cooking System - Installation Guide

Step-by-step guide to install and set up the Epicura ROS2 cooking system.

## System Requirements

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS2**: Humble Hawksbill
- **Python**: 3.10 or higher
- **RAM**: 4GB minimum (8GB recommended)
- **Storage**: 10GB free space

## Installation Steps

### 1. Install ROS2 Humble

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install software-properties-common curl -y

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools python3-colcon-common-extensions -y

# Source ROS2 setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Create Workspace

```bash
# Create workspace directory
mkdir -p ~/epicura_ws/src
cd ~/epicura_ws/src
```

### 3. Set Up Package Files

Create the package structure:

```bash
# Create package directory
mkdir -p epicura_cooking/epicura_cooking
mkdir -p epicura_cooking/launch
mkdir -p epicura_cooking/config
mkdir -p epicura_cooking/resource
mkdir -p epicura_cooking/test

# Navigate to package
cd epicura_cooking
```

Now copy/create the following files in their respective locations:

**Package Root (`~/epicura_ws/src/epicura_cooking/`):**
- `package.xml`
- `setup.py`
- `setup.cfg`

**Python Package (`~/epicura_ws/src/epicura_cooking/epicura_cooking/`):**
- `__init__.py` (empty file)
- `CookingNode.py`
- `CookingBot.py`
- `CookingScheduler.py`
- `RecipeModel.py`

**Launch Files (`~/epicura_ws/src/epicura_cooking/launch/`):**
- `cooking_node_launch.py`

**Config Files (`~/epicura_ws/src/epicura_cooking/config/`):**
- `cooking_params.yaml`

**Resource (`~/epicura_ws/src/epicura_cooking/resource/`):**
- `epicura_cooking` (marker file)

### 4. Create __init__.py

```bash
# Create empty __init__.py
touch ~/epicura_ws/src/epicura_cooking/epicura_cooking/__init__.py
```

### 5. Install Dependencies

```bash
cd ~/epicura_ws
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

### 6. Build the Package

```bash
cd ~/epicura_ws
colcon build --packages-select epicura_cooking

# Or with symlink-install for development
colcon build --packages-select epicura_cooking --symlink-install
```

### 7. Source the Workspace

```bash
source ~/epicura_ws/install/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/epicura_ws/install/setup.bash" >> ~/.bashrc
```

## Verification

### 1. Check Package Installation

```bash
# List ROS2 packages
ros2 pkg list | grep epicura

# Expected output:
# epicura_cooking
```

### 2. Check Executables

```bash
# List package executables
ros2 pkg executables epicura_cooking

# Expected output:
# epicura_cooking cooking_node
```

### 3. Test Node Launch

```bash
# Terminal 1: Launch the cooking node
ros2 run epicura_cooking cooking_node
```

In another terminal:

```bash
# Terminal 2: Check if node is running
ros2 node list

# Expected output:
# /epicura_cooking_node
```

### 4. Verify Topics

```bash
# List topics
ros2 topic list | grep epicura

# Expected topics:
# /epicura/cooking_progress
# /epicura/cooking_status
# /epicura/current_segment
# /epicura/emergency_stop
# /epicura/is_stirring
# /epicura/logs
# /epicura/remaining_time
# /epicura/start_recipe
# /epicura/stirring_rpm
# /epicura/stop_cooking
# /epicura/temperature
```

## Quick Start

### Launch with Default Parameters

```bash
ros2 launch epicura_cooking cooking_node_launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch epicura_cooking cooking_node_launch.py \
    status_update_rate:=2.0 \
    enable_hardware:=false \
    max_temperature:=300
```

### Monitor Status

```bash
# Terminal 1: Monitor progress
ros2 topic echo /epicura/cooking_progress

# Terminal 2: Monitor temperature
ros2 topic echo /epicura/temperature

# Terminal 3: Monitor logs
ros2 topic echo /epicura/logs
```

## Troubleshooting

### Build Errors

**Error: "Package not found"**
```bash
# Make sure workspace is properly structured
cd ~/epicura_ws/src
tree epicura_cooking  # Should show proper structure

# Rebuild
cd ~/epicura_ws
colcon build --packages-select epicura_cooking
```

**Error: "setup.py not found"**
```bash
# Verify setup.py exists
ls ~/epicura_ws/src/epicura_cooking/setup.py

# Check setup.py content
cat ~/epicura_ws/src/epicura_cooking/setup.py
```

### Runtime Errors

**Error: "No module named 'epicura_cooking'"**
```bash
# Source workspace
source ~/epicura_ws/install/setup.bash

# Verify installation
python3 -c "import epicura_cooking"
```

**Error: "Node not found"**
```bash
# Check if package is built
ls ~/epicura_ws/install/epicura_cooking

# Rebuild if necessary
cd ~/epicura_ws
colcon build --packages-select epicura_cooking
source install/setup.bash
```

### Import Errors

**Error: "Cannot import RecipeModel"**
```bash
# Make sure all Python files are in the package directory
ls ~/epicura_ws/src/epicura_cooking/epicura_cooking/

# Files should include:
# - __init__.py
# - CookingNode.py
# - CookingBot.py
# - CookingScheduler.py
# - RecipeModel.py
```

## Development Setup

For active development, use symlink install:

```bash
cd ~/epicura_ws
colcon build --packages-select epicura_cooking --symlink-install
```

This allows you to edit Python files without rebuilding.

## Testing

### Run Test Script

```bash
# Terminal 1: Launch node
ros2 launch epicura_cooking cooking_node_launch.py

# Terminal 2: Run test script
cd ~/epicura_ws/src/epicura_cooking
python3 test_cooking_node.py
```

### Manual Testing

```bash
# Start a simple recipe
ros2 topic pub --once /epicura/start_recipe std_msgs/String \
  "{data: '{\"name\": \"Test\", \"cooking_segments\": []}'}"

# Stop cooking
ros2 topic pub --once /epicura/stop_cooking std_msgs/String "{data: 'stop'}"
```

## Uninstallation

```bash
# Remove built package
cd ~/epicura_ws
rm -rf build/epicura_cooking install/epicura_cooking log/

# Remove source (optional)
rm -rf src/epicura_cooking
```

## Next Steps

After installation:
1. Read [ROS2_README.md](ROS2_README.md) for detailed usage
2. Review [README.md](README.md) for architecture overview
3. Check example recipes and configurations
4. Configure hardware interfaces for your setup

## Support

For installation issues:
- Check ROS2 Humble documentation: https://docs.ros.org/en/humble/
- Review colcon documentation: https://colcon.readthedocs.io/
- Contact: manas@epicura.ai

## Additional Resources

- ROS2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- Python Node Tutorial: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- Launch Files: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html

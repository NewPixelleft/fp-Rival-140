#!/bin/bash

# Stop if any command fails
set -e

# Navigate to the workspace directory
cd "$(dirname "$0")"

echo "=== Setting up environment ==="
source scripts/setup_env.bash

echo "=== Building workspace ==="
colcon build --symlink-install --executor sequential

echo "=== Sourcing workspace ==="
source install/local_setup.bash

echo "=== Launching Gazebo ==="
# ros2 launch sim_bringup dummy.launch.py
ros2 launch sim_bringup controller.launch.py
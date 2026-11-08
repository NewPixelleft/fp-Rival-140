set -e

# Navigate to the workspace directory
cd "$(dirname "$0")"
echo "=== Launching Gazebo ==="
ros2 launch sim_bringup controller.launch.py
ros2 launch sim_bringup dummy.launch.py
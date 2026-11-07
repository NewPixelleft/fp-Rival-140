set -e

cd "$(dirname "$0")"

source scripts/setup_env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
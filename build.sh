set -e
cd "$(dirname "$0")"
echo "=== Setting up environment ==="
source scripts/setup_env.bash

echo "=== Building workspace ==="
colcon build --symlink-install --executor sequential

echo "=== Sourcing workspace ==="
source install/local_setup.bash
#!/usr/bin/env bash
# Runs inside the container. Builds the message package and go1_control once (cached in a volume),
# points the controllers at the container's Python and isolates ROS, then runs the command.
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

CACHE=/var/cache/go1
UNITREE=/ws/unitree_ws/src/unitree_ros2
if [ ! -f "$CACHE/install/setup.bash" ]; then
    echo "entrypoint: building ros2_unitree_legged_msgs and go1_control (first run)"
    cd /tmp
    colcon build --build-base "$CACHE/build" --install-base "$CACHE/install" \
        --base-paths "$UNITREE/ros2_unitree_legged_msgs" /ws/go1_sim/go1_control
fi
set +u
source "$CACHE/install/setup.bash"
set -u

# Webots starts its controllers with the Python from this file; the path differs on every computer.
for c in go1_sim referee; do
    printf '[python]\nCOMMAND = /opt/sim-venv/bin/python3\n' > "/ws/go1_sim/controllers/$c/runtime.ini"
done

# Same isolation as on the laptop: domain 42, this container only.
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS

exec "$@"

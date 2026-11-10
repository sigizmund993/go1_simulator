#!/usr/bin/env bash
# Runs the Go1 simulator in Docker. Run it from anywhere; it finds the repo itself.
#
#   ./docker/run.sh                  build the image if needed, open Webots (needs a display)
#   ./docker/run.sh level1           open the TrueTechArena level 1 world instead
#   ./docker/run.sh build            build (or rebuild) the image
#   ./docker/run.sh exec CMD...      run a command inside the running simulator container,
#                                    e.g. ./docker/run.sh exec ros2 topic list
#   ./docker/run.sh run [program]    run a go1_control program (default main) inside the container
#
# Expects unitree_ws next to this repo: <parent>/go1_sim and <parent>/unitree_ws.
set -euo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
UNITREE=$(cd "$REPO/.." && pwd)/unitree_ws
IMAGE=go1-sim
NAME=go1-sim
BUILD_VOLUME=go1-sim-build

die() { echo "ERROR: $*" >&2; exit 1; }

[ -d "$UNITREE/src/unitree_ros2/ros2_unitree_legged_msgs" ] \
    || die "no $UNITREE/src/unitree_ros2 (ros2_unitree_legged_msgs). Clone unitree_ros2 next to this repo."
[ -d "$REPO/policies" ] || die "no $REPO/policies: copy the walking policy files there (see README)."

build_image() {
    docker build -t "$IMAGE" "$REPO"
}

running() {
    docker ps --format '{{.Names}}' | grep -qx "$NAME"
}

start() {
    local world=$1
    shift
    if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
        build_image
    fi
    if running; then
        die "container $NAME is already running. Stop it with: docker stop $NAME"
    fi
    local display_args=()
    if [ -n "${DISPLAY:-}" ]; then
        display_args=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
        command -v xhost >/dev/null && xhost +local:docker >/dev/null 2>&1 || true
    else
        echo "warning: no DISPLAY, Webots cannot open its window" >&2
    fi
    exec docker run --rm -it --name "$NAME" "${display_args[@]}" \
        -v "$REPO:/ws/go1_sim" \
        -v "$UNITREE:/ws/unitree_ws:ro" \
        -v "$BUILD_VOLUME:/var/cache/go1" \
        "$IMAGE" webots --mode=realtime "/ws/go1_sim/worlds/$world.wbt"
}

exec_in() {
    running || die "no simulator running. Start it first: ./docker/run.sh"
    docker exec -it "$NAME" bash -c 'source /opt/ros/jazzy/setup.bash && source /var/cache/go1/install/setup.bash && export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && exec "$@"' bash "$@"
}

case "${1:-}" in
    "" ) start go1 ;;
    level1) start level1 ;;
    build) build_image ;;
    exec) shift; exec_in "$@" ;;
    run)
        shift
        exec_in ros2 run go1_control "${1:-main}" "${@:2}"
        ;;
    *) die "unknown command: $1 (see the top of this file)" ;;
esac

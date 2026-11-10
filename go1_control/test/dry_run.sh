#!/usr/bin/env bash
# Exercises Robot on the Pi with every topic in /dry.
# udp_high does not listen there, so nothing reaches the robot.
set -uo pipefail
set +u
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ws/install/setup.bash"
set -u

MSG=ros2_unitree_legged_msgs/msg
HERE=$(cd "$(dirname "$0")" && pwd)
ARGS=(--ros-args -r __ns:=/dry)
LOG=$(mktemp)
STATE=
RECORDER=

stop() { [ -n "$1" ] && kill "$1" 2>/dev/null && wait "$1" 2>/dev/null; }

cleanup() {
    stop "$STATE"
    stop "$RECORDER"
    rm -f "$LOG"
}
trap cleanup EXIT

fake_state() {
    stop "$STATE"
    ros2 topic pub -r 20 /dry/high_state "$MSG/HighState" "$1" >/dev/null 2>&1 &
    STATE=$!
    sleep 3
}

record() {
    python3 -u "$HERE/dry_recorder.py" "${ARGS[@]}" > "$LOG" 2>/dev/null &
    RECORDER=$!
    sleep 3
}

# Prints each run of equal commands once, with how many packets it lasted.
recorded() {
    stop "$RECORDER"
    RECORDER=
    uniq -c "$LOG"
}

echo "== real /high_cmd before start"
timeout 12 ros2 topic info /high_cmd | grep -E 'Publisher|Subscription'

echo "== no state: Robot must refuse to start"
python3 "$HERE/dry_scenario.py" no_state "${ARGS[@]}" 2>&1 | cut -c1-110

fake_state "{head: [254, 239]}"

echo "== all steps"
record
python3 "$HERE/dry_scenario.py" all "${ARGS[@]}" 2>&1 | cut -c1-110
echo "-- commands sent (packets, mode, gait, vx, wz, height):"
recorded

echo "== main.py through ros2 run"
record
timeout 30 ros2 run go1_control main "${ARGS[@]}" 2>&1 | cut -c1-110
echo "exit code: ${PIPESTATUS[0]}"
recorded

echo "== rate while running"
python3 "$HERE/dry_scenario.py" walk "${ARGS[@]}" >/dev/null 2>&1 &
WALK=$!
sleep 3
timeout 8 ros2 topic hz /dry/high_cmd 2>/dev/null | grep -m1 'average rate'
kill -INT "$WALK" 2>/dev/null
kill -TERM "$WALK" 2>/dev/null
wait "$WALK" 2>/dev/null

for sig in INT TERM; do
    echo "== SIG$sig while walking: expect idle packets last and a clean exit"
    record
    timeout -s "$sig" 5 python3 "$HERE/dry_scenario.py" walk "${ARGS[@]}" 2>&1 | cut -c1-110
    sleep 1
    recorded | tail -2
    if pgrep -f '[d]ry_scenario.py' >/dev/null; then echo "STILL RUNNING"; else echo "stopped"; fi
done

echo "== tilt 0.8 rad: move must be refused"
fake_state "{head: [254, 239], imu: {rpy: [0.8, 0.0, 0.0]}}"
record
python3 "$HERE/dry_scenario.py" tilted "${ARGS[@]}" 2>&1 | cut -c1-110
recorded

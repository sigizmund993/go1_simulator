"""Prints every high_cmd in /dry as one line, for dry_run.sh."""

import sys

import rclpy
from rclpy.node import Node
from ros2_unitree_legged_msgs.msg import HighCmd

if '__ns:=/dry' not in sys.argv:
    sys.exit('run only through dry_run.sh: --ros-args -r __ns:=/dry')


def on_command(msg):
    print(msg.mode, msg.gait_type, round(msg.velocity[0], 3), round(msg.yaw_speed, 3),
          round(msg.body_height, 3), list(msg.head), msg.level_flag)


rclpy.init()
node = Node('dry_recorder')
node.create_subscription(HighCmd, 'high_cmd', on_command, 100)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

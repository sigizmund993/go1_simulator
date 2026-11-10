"""Webots controller of the simulated Go1: the same ROS 2 topics as udp_high on the real robot.

Subscribes to high_cmd, publishes high_state and scan. Start it with go1_sim.sh.
"""
import math
import os

# Nothing from the simulator may reach the real robot, which listens on domain 0 on the Wi-Fi.
# Set before rclpy is imported, whatever way Webots was started.
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_STATIC_PEERS', None)
if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
    os.environ['ROS_DOMAIN_ID'] = '42'

from controller import Robot  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from ros2_unitree_legged_msgs.msg import HighCmd, HighState  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from locomotion import LEGS, Locomotion  # noqa: E402
from sport import HIGHLEVEL, PACKET_HEAD, SportMode  # noqa: E402

# Motor order in HighState, as indices into the policy order used by Locomotion.
STATE_LEGS = ['FR', 'FL', 'RR', 'RL']
MOTOR_INDEX = [LEGS.index(leg) * 3 + part for leg in STATE_LEGS for part in range(3)]
SCAN_PERIOD_MS = 100
# The real lidar driver reports this for beams that hit nothing.
SCAN_NO_RETURN = 100000.0


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


class Simulator:
    def __init__(self):
        self.robot = Robot()
        self.step_ms = int(self.robot.getBasicTimeStep())
        self.loco = Locomotion(self.robot)
        self.gps = self.robot.getDevice('gps')
        self.imu = self.robot.getDevice('imu')
        self.gyro = self.robot.getDevice('gyro')
        self.accelerometer = self.robot.getDevice('accelerometer')
        self.lidar = self.robot.getDevice('lidar')
        self.gps.enable(self.step_ms)
        self.accelerometer.enable(self.step_ms)
        self.lidar.enable(SCAN_PERIOD_MS)

        rclpy.init()
        self.node = rclpy.create_node('go1_sim')
        self.sport = SportMode(self.loco, warn=self.node.get_logger().warn)
        self.state_pub = self.node.create_publisher(HighState, 'high_state', 10)
        self.scan_pub = self.node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.node.create_subscription(HighCmd, 'high_cmd', self.on_cmd, 10)
        # Like udp_high: zeros until the first command, then the reply to the latest command.
        self.state = HighState()
        self.got_cmd = False
        self.origin = None
        self.scan_due_ms = 0
        self.time_ms = 0

    def on_cmd(self, msg):
        self.sport.command(msg)
        self.got_cmd = True

    def fill_state(self):
        x, y, z = self.gps.getValues()
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        if not math.isfinite(x):
            return
        if self.origin is None:
            self.origin = (x, y, yaw)
        x0, y0, yaw0 = self.origin
        # Position and heading count from where the robot was switched on, as odometry does.
        c0, s0 = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        vx, vy, vz = self.gps.getSpeedVector()
        c, s = math.cos(yaw), math.sin(yaw)

        state = HighState()
        state.head = PACKET_HEAD
        state.level_flag = HIGHLEVEL
        state.mode = self.sport.mode
        state.gait_type = self.sport.gait_type
        state.imu.quaternion = quaternion(roll, pitch, yaw - yaw0)
        state.imu.gyroscope = [float(v) for v in self.gyro.getValues()]
        state.imu.accelerometer = [float(v) for v in self.accelerometer.getValues()]
        state.imu.rpy = [roll, pitch, math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))]
        for motor, index in zip(state.motor_state, MOTOR_INDEX):
            motor.q = float(self.loco.q[index])
            motor.dq = float(self.loco.dq[index])
            motor.tau_est = float(self.loco.torque[index])
        state.bms.soc = 100
        state.position = [c0 * dx + s0 * dy, -s0 * dx + c0 * dy, z]
        state.body_height = z
        state.velocity = [c * vx + s * vy, -s * vx + c * vy, vz]
        state.yaw_speed = float(self.gyro.getValues()[2])
        self.state = state

    def publish_scan(self):
        ranges = self.lidar.getRangeImage()
        if not ranges:
            return
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        count = len(ranges)
        # Webots sweeps clockwise from behind the robot; ROS wants counter-clockwise.
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / count
        scan.angle_max = scan.angle_min + scan.angle_increment * (count - 1)
        scan.scan_time = SCAN_PERIOD_MS / 1000.0
        scan.time_increment = scan.scan_time / count
        scan.range_min = self.lidar.getMinRange()
        scan.range_max = self.lidar.getMaxRange()
        scan.ranges = [r if math.isfinite(r) else SCAN_NO_RETURN for r in reversed(ranges)]
        self.scan_pub.publish(scan)

    def run(self):
        while True:
            self.loco.tick()
            if self.robot.step(self.step_ms) == -1:
                break
            self.time_ms += self.step_ms
            self.got_cmd = False
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if self.got_cmd:
                self.fill_state()
            self.state_pub.publish(self.state)
            if self.time_ms >= self.scan_due_ms:
                self.scan_due_ms += SCAN_PERIOD_MS
                self.publish_scan()
        self.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    Simulator().run()

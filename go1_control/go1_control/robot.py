"""Controls the Go1 through udp_high: a Robot class with limits and safety stops.

    with Robot() as robot:
        robot.move(vx=0.1, duration=1.0)
        robot.sit_down()
        robot.stand_up()

Start it when the dog is already standing and udp_high is running.
"""

from dataclasses import dataclass
from enum import Enum
import math
import signal
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ros2_unitree_legged_msgs.msg import HighCmd, HighState
from sensor_msgs.msg import LaserScan

MODE_IDLE = 0
MODE_FORCE_STAND = 1
MODE_WALK = 2
MODE_STAND_DOWN = 5
MODE_STAND_UP = 6
MODE_DAMPING = 7

GAIT_IDLE = 0
GAIT_TROT = 1

PACKET_HEAD = [0xFE, 0xEF]
HIGHLEVEL = 0xEE
SHUTDOWN_PACKETS = 5
POLL_PERIOD = 0.02


class RobotError(Exception):
    """The robot refused a command: wrong posture, too much tilt or no state from udp_high."""


class Posture(Enum):
    STANDING = 'standing'
    GOING_DOWN = 'going_down'
    DOWN = 'down'
    GOING_UP = 'going_up'
    DAMPED = 'damped'


@dataclass
class Limits:
    max_vx: float = 0.3
    max_vy: float = 0.2
    max_wz: float = 0.5
    max_body_offset: float = 0.1
    cmd_timeout: float = 0.5
    state_timeout: float = 0.5
    max_tilt: float = 0.5
    transition_time: float = 2.0
    foot_raise_height: float = 0.08


@dataclass
class Command:
    mode: int = MODE_IDLE
    gait_type: int = GAIT_IDLE
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    body_height: float = 0.0
    foot_raise_height: float = 0.0


def clamp(value, limit):
    if not math.isfinite(value):
        raise ValueError(f'not a finite number: {value}')
    return max(-limit, min(limit, value))


class Controller:
    """Decides which high-level command to send. Has no ROS state so it can be unit tested."""

    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.posture = Posture.STANDING
        self._velocity = (0.0, 0.0, 0.0)
        self._velocity_time = None
        self._body_offset = 0.0
        self._state_time = None
        self._tilted = False
        self._transition_start = None

    @property
    def tilted(self):
        return self._tilted

    def state_is_fresh(self, now):
        return self._state_time is not None and now - self._state_time <= self.limits.state_timeout

    def set_velocity(self, vx, vy, wz, now):
        lim = self.limits
        self._velocity = (clamp(vx, lim.max_vx), clamp(vy, lim.max_vy), clamp(wz, lim.max_wz))
        self._velocity_time = now

    def set_body_offset(self, offset):
        self._body_offset = clamp(offset, self.limits.max_body_offset)

    def update_state(self, roll, pitch, now):
        self._state_time = now
        self._tilted = abs(roll) > self.limits.max_tilt or abs(pitch) > self.limits.max_tilt
        if self._tilted:
            # A stale velocity must not resume walking once the robot is level again.
            self._velocity_time = None

    def request_down(self, now):
        if self.posture is not Posture.STANDING:
            return False, f'cannot stand down from {self.posture.value}'
        self.posture = Posture.GOING_DOWN
        self._transition_start = now
        self._velocity_time = None
        return True, 'standing down'

    def request_up(self, now):
        if self.posture is not Posture.DOWN:
            return False, f'cannot stand up from {self.posture.value}'
        self.posture = Posture.GOING_UP
        self._transition_start = now
        return True, 'standing up'

    def request_damp(self):
        if self.posture is not Posture.DOWN:
            return False, f'damping is allowed only when down, not {self.posture.value}'
        self.posture = Posture.DAMPED
        return True, 'damping'

    def _walk_allowed(self, now):
        if self._tilted or self._velocity_time is None or not self.state_is_fresh(now):
            return False
        return (now - self._velocity_time <= self.limits.cmd_timeout
                and any(v != 0.0 for v in self._velocity))

    def step(self, now):
        if self.posture is Posture.GOING_DOWN:
            if now - self._transition_start >= self.limits.transition_time:
                self.posture = Posture.DOWN
            return Command(mode=MODE_STAND_DOWN)
        if self.posture is Posture.DOWN:
            return Command(mode=MODE_STAND_DOWN)
        if self.posture is Posture.DAMPED:
            return Command(mode=MODE_DAMPING)
        if self.posture is Posture.GOING_UP:
            if now - self._transition_start >= self.limits.transition_time:
                self.posture = Posture.STANDING
            return Command(mode=MODE_STAND_UP)

        if self._walk_allowed(now):
            vx, vy, wz = self._velocity
            return Command(
                mode=MODE_WALK, gait_type=GAIT_TROT, vx=vx, vy=vy, wz=wz,
                body_height=self._body_offset,
                foot_raise_height=self.limits.foot_raise_height)
        if self._body_offset != 0.0 and not self._tilted:
            return Command(mode=MODE_FORCE_STAND, body_height=self._body_offset)
        return Command(mode=MODE_IDLE)


def _raise_interrupt(_signum, _frame):
    raise KeyboardInterrupt


class Robot:
    """Sends high_cmd from a background thread. Use it from the main thread, in a with block."""

    def __init__(self, limits=None, rate=50.0, ready_timeout=5.0):
        self._controller = Controller(limits)
        self._lock = threading.Lock()
        self._state = None
        self._scan = None
        # Keep the context alive after Ctrl-C so the final idle packets can still be published.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        # A plain SIGTERM would end the program with a walking command as the last one sent.
        signal.signal(signal.SIGTERM, _raise_interrupt)
        self._node = Node('go1_robot')
        self._pub = self._node.create_publisher(HighCmd, 'high_cmd', 10)
        self._node.create_subscription(
            HighState, 'high_state', self._on_state, qos_profile_sensor_data)
        self._node.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self._node.create_timer(1.0 / rate, self._on_timer)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        try:
            self._wait_for_state(ready_timeout)
        except BaseException:
            self.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, exc_type, _exc, _traceback):
        self.close()
        return exc_type is KeyboardInterrupt

    @property
    def state(self):
        """The latest HighState message from the robot."""
        with self._lock:
            return self._state

    @property
    def scan(self):
        """The latest /scan message from the lidar, or None if there is none yet."""
        with self._lock:
            return self._scan

    @property
    def posture(self):
        with self._lock:
            return self._controller.posture

    def move(self, vx=0.0, vy=0.0, wz=0.0, duration=None):
        """Walks at m/s and rad/s, clamped to Limits.

        With duration, blocks and then stops. Without it, returns at once and the robot stops
        unless move is called again within Limits.cmd_timeout, so it suits a control loop.
        Raises RobotError if the robot cannot walk now.
        """
        self._send_velocity(vx, vy, wz)
        if duration is None:
            return
        end = time.monotonic() + duration
        while time.monotonic() < end:
            time.sleep(POLL_PERIOD)
            self._send_velocity(vx, vy, wz)
        self.stop()

    def stop(self):
        with self._lock:
            self._controller.set_velocity(0.0, 0.0, 0.0, time.monotonic())

    def set_body_height(self, offset):
        """Raises or lowers the body by offset metres from normal height, clamped to Limits."""
        with self._lock:
            self._controller.set_body_offset(offset)

    def sit_down(self):
        """Lowers the body and blocks until the transition time has passed."""
        self._request(self._controller.request_down, time.monotonic())
        self._wait_for_posture(Posture.DOWN)

    def stand_up(self):
        """Stands up after sit_down and blocks until the transition time has passed."""
        self._request(self._controller.request_up, time.monotonic())
        self._wait_for_posture(Posture.STANDING)

    def damp(self):
        """Makes the motors slack after sit_down. The robot cannot stand up again from this."""
        self._request(self._controller.request_damp)

    def close(self):
        self._executor.shutdown()
        self._spin_thread.join()
        # A posture request made just before closing has to reach the robot too.
        with self._lock:
            standing = self._controller.posture is Posture.STANDING
            command = Command() if standing else self._controller.step(time.monotonic())
        for _ in range(SHUTDOWN_PACKETS):
            self._publish(command)
            time.sleep(POLL_PERIOD)
        self._node.destroy_node()
        rclpy.try_shutdown()

    def _send_velocity(self, vx, vy, wz):
        now = time.monotonic()
        with self._lock:
            controller = self._controller
            if controller.posture is not Posture.STANDING:
                raise RobotError(f'cannot move while {controller.posture.value}')
            if controller.tilted:
                raise RobotError('cannot move: tilt limit exceeded')
            if not controller.state_is_fresh(now):
                raise RobotError('cannot move: no fresh high_state')
            controller.set_velocity(vx, vy, wz, now)

    def _request(self, request, *args):
        with self._lock:
            accepted, message = request(*args)
        self._node.get_logger().info(message)
        if not accepted:
            raise RobotError(message)

    def _wait_for_state(self, timeout):
        end = time.monotonic() + timeout
        while self.state is None:
            if time.monotonic() >= end:
                raise RobotError(f'no high_state for {timeout} s: is udp_high running?')
            time.sleep(POLL_PERIOD)

    def _wait_for_posture(self, posture):
        while self.posture is not posture:
            time.sleep(POLL_PERIOD)

    def _on_state(self, msg):
        # udp_high publishes zeros until Legged_sport has answered.
        if list(msg.head) != PACKET_HEAD:
            return
        with self._lock:
            was_tilted = self._controller.tilted
            self._state = msg
            self._controller.update_state(msg.imu.rpy[0], msg.imu.rpy[1], time.monotonic())
            tilted = self._controller.tilted
        if tilted and not was_tilted:
            self._node.get_logger().warn(
                f'tilt limit exceeded: roll={msg.imu.rpy[0]:.2f} pitch={msg.imu.rpy[1]:.2f}, '
                'walking blocked')
        elif was_tilted and not tilted:
            self._node.get_logger().info('tilt back within limits')

    def _on_scan(self, msg):
        with self._lock:
            self._scan = msg

    def _on_timer(self):
        with self._lock:
            command = self._controller.step(time.monotonic())
        self._publish(command)

    def _publish(self, command):
        msg = HighCmd()
        msg.head = PACKET_HEAD
        msg.level_flag = HIGHLEVEL
        msg.mode = command.mode
        msg.gait_type = command.gait_type
        msg.velocity = [float(command.vx), float(command.vy)]
        msg.yaw_speed = float(command.wz)
        msg.body_height = float(command.body_height)
        msg.foot_raise_height = float(command.foot_raise_height)
        self._pub.publish(msg)

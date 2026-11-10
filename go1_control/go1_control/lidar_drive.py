"""Walks forward and keeps off walls with the lidar: ros2 run go1_control lidar_drive

Stop with Ctrl-C. Start it when the dog is standing, on open ground, with the lidar running.
"""
import math
import time

from go1_control.avoid import avoid
from go1_control.robot import Robot

WANTED = (0.2, 0.0, 0.0)   # m/s, m/s, rad/s: straight ahead
DURATION = 20.0            # seconds
PERIOD = 0.1               # seconds between commands; move() must be called within cmd_timeout


def wait_for_scan(robot, timeout=5.0):
    end = time.monotonic() + timeout
    while robot.scan is None:
        if time.monotonic() >= end:
            raise RuntimeError('no /scan: is the lidar running?')
        time.sleep(0.05)


def main():
    with Robot() as robot:
        wait_for_scan(robot)
        end = time.monotonic() + DURATION
        while time.monotonic() < end:
            scan = robot.scan
            vx, vy, wz = avoid(WANTED, scan.ranges, scan.angle_min, scan.angle_increment,
                               scan.range_max)
            robot.move(vx=vx, vy=vy, wz=wz)
            time.sleep(PERIOD)
        robot.stop()


if __name__ == '__main__':
    main()

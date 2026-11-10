"""Your program for the robot. Run it with: ros2 run go1_control main"""

from go1_control.robot import Robot


def main():
    with Robot() as robot:
        robot.move(vx=0.1, duration=1.0)
        robot.sit_down()
        robot.stand_up()


if __name__ == '__main__':
    main()

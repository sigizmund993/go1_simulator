"""Scenarios for dry_run.sh. Refuses to run outside /dry, where udp_high does not listen."""

import sys
import time

from go1_control.robot import Robot, RobotError

if '__ns:=/dry' not in sys.argv:
    sys.exit('run only through dry_run.sh: --ros-args -r __ns:=/dry')


def refused(call, *args, **kwargs):
    try:
        call(*args, **kwargs)
    except RobotError as error:
        print(f'  refused as expected: {error}')
    else:
        print('  FAIL: was not refused')


def all_steps():
    with Robot() as robot:
        print('state head:', list(robot.state.head))
        print('move 0.1 for 1 s (mode 2)')
        robot.move(vx=0.1, duration=1.0)
        print('idle 1 s (mode 0)')
        time.sleep(1.0)
        print('move above limits for 1 s (mode 2, clamped)')
        robot.move(vx=5.0, wz=5.0, duration=1.0)
        print('single move, no repeat: watchdog must stop it')
        robot.move(vx=0.1)
        time.sleep(1.5)
        print('body height -1 for 1 s (mode 1, clamped)')
        robot.set_body_height(-1.0)
        time.sleep(1.0)
        robot.set_body_height(0.0)
        print('stand_up while standing')
        refused(robot.stand_up)
        print('damp while standing')
        refused(robot.damp)
        print('sit_down (mode 5)')
        robot.sit_down()
        print('move while down')
        refused(robot.move, vx=0.1)
        print('stand_up (mode 6)')
        robot.stand_up()
        print('sit_down, damp (mode 7)')
        robot.sit_down()
        robot.damp()
        print('stand_up while damped')
        refused(robot.stand_up)


def tilted():
    with Robot() as robot:
        time.sleep(0.5)
        print('move while tilted')
        refused(robot.move, vx=0.1, duration=1.0)


def no_state():
    print('no state from udp_high')
    refused(Robot, ready_timeout=2.0)


def walk_forever():
    with Robot() as robot:
        print('walking until a signal')
        while True:
            robot.move(vx=0.1, duration=1.0)
    print('with block left after the signal')


{'all': all_steps, 'tilted': tilted, 'no_state': no_state, 'walk': walk_forever}[sys.argv[1]]()

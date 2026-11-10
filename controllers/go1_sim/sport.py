"""Stands in for Legged_sport: turns high-level commands into calls to Locomotion.

Has no Webots or ROS imports, so it can be unit tested.

Only transitions that are known to work on the real robot are accepted. A command the real robot
has never been tested with (walking while lying, standing up from damping) is refused here, so
a program that works in the simulator does not rely on it.
"""
from locomotion import DOWN_POSE, stand_pose

MODE_IDLE = 0
MODE_FORCE_STAND = 1
MODE_WALK = 2
MODE_STAND_DOWN = 5
MODE_STAND_UP = 6
MODE_DAMPING = 7

PACKET_HEAD = [0xFE, 0xEF]
HIGHLEVEL = 0xEE

MAX_VX = 0.8
MAX_VY = 0.4
MAX_WZ = 1.5
MAX_BODY_OFFSET = 0.1
TRANSITION_TIME = 1.5
HEIGHT_TIME = 0.5


def clamp(value, limit):
    return max(-limit, min(limit, value))


class SportMode:
    def __init__(self, locomotion, warn=print):
        self._loco = locomotion
        self._warn = warn
        self._warned = set()
        self.posture = 'up'
        self.mode = MODE_IDLE
        self.gait_type = 0

    def _refuse(self, mode, reason):
        if (mode, reason) not in self._warned:
            self._warned.add((mode, reason))
            self._warn(f'mode {mode} ignored: {reason}')
        return False

    def command(self, msg):
        """Applies a HighCmd. Returns False if the command was ignored."""
        if list(msg.head) != PACKET_HEAD or msg.level_flag != HIGHLEVEL:
            return self._refuse(msg.mode, 'head must be [0xFE, 0xEF] and level_flag 0xEE')
        mode = msg.mode
        if mode == MODE_DAMPING:
            self._loco.damp()
            self.posture = 'damped'
        elif mode == MODE_STAND_DOWN:
            self._loco.pose(DOWN_POSE, TRANSITION_TIME)
            self.posture = 'down'
        elif mode == MODE_STAND_UP:
            if self.posture == 'damped':
                return self._refuse(mode, 'send mode 5 after damping first')
            self._loco.pose(stand_pose(), TRANSITION_TIME)
            self.posture = 'up'
        elif mode in (MODE_IDLE, MODE_FORCE_STAND, MODE_WALK):
            if self.posture != 'up':
                return self._refuse(mode, f'the robot is {self.posture}, stand up with mode 6 first')
            if self.mode in (MODE_STAND_UP, MODE_STAND_DOWN) and self._loco.busy:
                return False
            vx, vy = msg.velocity[0], msg.velocity[1]
            if mode == MODE_WALK and (vx or vy or msg.yaw_speed):
                self._loco.walk(clamp(vx, MAX_VX), clamp(vy, MAX_VY), clamp(msg.yaw_speed, MAX_WZ))
            else:
                offset = clamp(msg.body_height, MAX_BODY_OFFSET) if mode == MODE_FORCE_STAND else 0.0
                self._loco.pose(stand_pose(offset), HEIGHT_TIME)
        else:
            return self._refuse(mode, 'not supported by the simulator')
        self.mode = mode
        self.gait_type = msg.gait_type if mode == MODE_WALK else 0
        return True

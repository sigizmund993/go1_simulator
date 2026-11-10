"""Leg control for the simulated Go1: a neural walking policy, fixed poses and damping.

Based on policy_walk.py from the TrueTechArena Go2 simulator. The policy was trained for the Go2;
the Go1 has the same leg lengths and joint layout, so it is reused here as is.

Joint order everywhere in this file is the policy order: FL, FR, RL, RR x hip, thigh, calf.
"""
from collections import deque
import json
import math
import os

import MNN
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
POLICY_DIR = os.path.join(HERE, '..', '..', 'policies')
LEGS = ['FL', 'FR', 'RL', 'RR']
JOINTS = [f'{leg}_{part}_joint' for leg in LEGS for part in ('hip', 'thigh', 'calf')]

LEG_LENGTH = 0.213
STAND_THIGH = 0.79
DOWN_POSE = np.array([0.0, 1.36, -2.65] * 4)
POSE_KP = 80.0
POSE_KD = 1.0
DAMP_KD = 2.0
SETTLE_TIME = 0.8
# Measured in Webots: the policy walks slower than commanded by about this much at any speed
# (m/s, m/s, rad/s), so a non-zero command is raised by it.
SPEED_OFFSET = np.array([0.13, 0.13, 0.2])
POLICY_PERIOD = 0.02


def stand_pose(offset=0.0):
    """Joint angles that keep the feet under the hips with the body raised by offset metres."""
    nominal = 2 * LEG_LENGTH * math.cos(STAND_THIGH)
    ratio = max(0.5, min(0.98, (nominal + offset) / (2 * LEG_LENGTH)))
    thigh = math.acos(ratio)
    return np.array([0.0, thigh, -2 * thigh] * 4)


class Policy:
    def __init__(self, path):
        self._net = MNN.Interpreter(path)
        # One thread makes the inference repeatable.
        self._session = self._net.createSession({'numThread': 1})
        self._input = self._net.getSessionInput(self._session)
        self.input_size = int(np.prod(self._input.getShape()))

    def __call__(self, observation):
        tensor = MNN.Tensor(
            (1, self.input_size), MNN.Halide_Type_Float, observation.astype(np.float32).copy(),
            MNN.Tensor_DimensionType_Caffe)
        self._input.copyFrom(tensor)
        self._net.runSession(self._session)
        output = self._net.getSessionOutput(self._session)
        host = MNN.Tensor(
            output.getShape(), MNN.Halide_Type_Float, np.zeros(output.getShape(), np.float32),
            MNN.Tensor_DimensionType_Caffe)
        output.copyToHostTensor(host)
        return np.array(host.getData(), np.float32).reshape(-1)[:12]


class Locomotion:
    """Call tick() before every robot.step(). walk, pose and damp choose what the legs do."""

    def __init__(self, robot):
        with open(os.path.join(POLICY_DIR, 'free_walk_export_cfg.json')) as f:
            cfg = json.load(f)
        self._history_size = cfg['include_history_steps']
        self._clip_obs = cfg['clip_obs']
        self._clip_act = cfg['clip_act']
        self._kp = np.array(cfg['kp'], float)
        self._kd = np.array(cfg['kd'], float)
        self._torque_limit = np.array(cfg['torque_limits'], float)
        self._init_pose = np.array(cfg['init_jpos'], float)
        norm = cfg['normalization']
        self._act_mean = np.array(norm['act_mean'])
        self._act_scale = np.array(norm['act_scale'])
        self._ob_mean = np.array(norm['ob_mean'])
        self._ob_scale = np.array(norm['ob_scale'])
        self._policy = Policy(os.path.join(POLICY_DIR, cfg['free_walk_model_name']))

        step_ms = int(robot.getBasicTimeStep())
        self._dt = step_ms / 1000.0
        self._decimation = max(1, round(POLICY_PERIOD / self._dt))
        self._motors = [robot.getDevice(name) for name in JOINTS]
        self._sensors = [robot.getDevice(name + '_sensor') for name in JOINTS]
        for sensor in self._sensors:
            sensor.enable(step_ms)
        self._gyro = robot.getDevice('gyro')
        self._imu = robot.getDevice('imu')
        self._gyro.enable(step_ms)
        self._imu.enable(step_ms)

        self._mode = 'pose'
        self._velocity = np.zeros(3)
        self._goal = stand_pose()
        self._start = None
        self._pose_step = 0
        self._pose_steps = 1
        self._last_action = np.zeros(12)
        self._history = deque(maxlen=self._history_size)
        self._target = self._init_pose
        self._q_prev = None
        self._time = 0.0
        self._step = 0
        self.q = np.zeros(12)
        self.dq = np.zeros(12)
        self.torque = np.zeros(12)

    @property
    def busy(self):
        """True while the robot is still moving to a pose."""
        return (self._time < SETTLE_TIME
                or (self._mode == 'pose' and self._pose_step < self._pose_steps))

    def walk(self, vx, vy, wz):
        if self._mode != 'walk':
            self._mode = 'walk'
            self._history.clear()
            self._last_action = np.zeros(12)
            self._velocity = np.array([vx, vy, wz], float)

    def pose(self, goal, seconds):
        """Moves the joints to goal over the given time and holds them there."""
        if self._mode == 'pose' and np.array_equal(goal, self._goal):
            return
        self._mode = 'pose'
        self._goal = np.asarray(goal, float)
        self._start = None
        self._pose_step = 0
        self._pose_steps = max(1, int(seconds / POLICY_PERIOD))

    def damp(self):
        self._mode = 'damp'

    def _gravity(self):
        roll, pitch, _ = self._imu.getRollPitchYaw()
        return np.array([math.sin(pitch), -math.sin(roll) * math.cos(pitch),
                         -math.cos(roll) * math.cos(pitch)])

    def tick(self):
        self._time += self._dt
        q = np.array([sensor.getValue() for sensor in self._sensors])
        # Position sensors return NaN until the first robot.step().
        if not np.isfinite(q).all():
            return
        if self._q_prev is None:
            self._q_prev = q
        dq = (q - self._q_prev) / self._dt
        self._q_prev = q
        policy_tick = self._step % self._decimation == 0
        self._step += 1

        kp, kd = self._kp, self._kd
        if self._time < SETTLE_TIME:
            self._target = self._init_pose
        elif self._mode == 'damp':
            kp, kd = 0.0, DAMP_KD
            self._target = q
        elif self._mode == 'pose':
            kp, kd = POSE_KP, POSE_KD
            if policy_tick:
                if self._start is None:
                    self._start = q.copy()
                self._pose_step = min(self._pose_steps, self._pose_step + 1)
                part = self._pose_step / self._pose_steps
                self._target = self._start + part * (self._goal - self._start)
        elif policy_tick:
            command = self._velocity + np.sign(self._velocity) * SPEED_OFFSET
            raw = np.concatenate([command, self._gyro.getValues(), self._gravity(),
                                  q, dq, self._last_action])
            frame = np.clip((raw - self._ob_mean) * self._ob_scale, -self._clip_obs, self._clip_obs)
            while len(self._history) < self._history_size:
                self._history.append(frame.copy())
            self._history.append(frame)
            action = np.clip(self._policy(np.concatenate(list(self._history)[::-1])),
                             -self._clip_act, self._clip_act)
            self._last_action = action
            self._target = self._act_mean + self._act_scale * action

        torque = np.clip(kp * (self._target - q) - kd * dq, -self._torque_limit, self._torque_limit)
        for motor, value in zip(self._motors, torque):
            motor.setTorque(float(value))
        self.q, self.dq, self.torque = q, dq, torque

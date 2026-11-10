import pytest

from go1_control.robot import (
    Controller, GAIT_TROT, Limits, MODE_DAMPING, MODE_FORCE_STAND, MODE_IDLE, MODE_STAND_DOWN,
    MODE_STAND_UP, MODE_WALK, Posture,
)


def ready(now=0.0):
    controller = Controller(Limits())
    controller.update_state(0.0, 0.0, now)
    return controller


def test_idle_by_default():
    assert ready().step(0.0).mode == MODE_IDLE


def test_walks_with_fresh_velocity():
    controller = ready()
    controller.set_velocity(0.1, 0.0, 0.2, 0.0)
    command = controller.step(0.1)
    assert (command.mode, command.gait_type) == (MODE_WALK, GAIT_TROT)
    assert (command.vx, command.wz) == (0.1, 0.2)
    assert command.foot_raise_height > 0.0


def test_velocity_is_clamped_both_ways():
    controller = ready()
    controller.set_velocity(5.0, -5.0, -5.0, 0.0)
    command = controller.step(0.0)
    assert (command.vx, command.vy, command.wz) == (0.3, -0.2, -0.5)


def test_stops_when_velocity_goes_stale():
    controller = ready()
    controller.update_state(0.0, 0.0, 1.0)
    controller.set_velocity(0.1, 0.0, 0.0, 0.0)
    assert controller.step(0.6).mode == MODE_IDLE


def test_zero_velocity_is_idle():
    controller = ready()
    controller.set_velocity(0.0, 0.0, 0.0, 0.0)
    assert controller.step(0.0).mode == MODE_IDLE


def test_no_walk_without_robot_state():
    controller = Controller(Limits())
    controller.set_velocity(0.1, 0.0, 0.0, 0.0)
    assert controller.step(0.0).mode == MODE_IDLE


def test_no_walk_when_state_goes_stale():
    controller = ready(0.0)
    controller.set_velocity(0.1, 0.0, 0.0, 0.5)
    assert controller.step(0.6).mode == MODE_IDLE


def test_tilt_blocks_walking_and_needs_a_new_command():
    controller = ready()
    controller.set_velocity(0.1, 0.0, 0.0, 0.0)
    controller.update_state(0.6, 0.0, 0.1)
    assert controller.step(0.1).mode == MODE_IDLE
    controller.update_state(0.0, 0.0, 0.2)
    assert controller.step(0.2).mode == MODE_IDLE
    controller.set_velocity(0.1, 0.0, 0.0, 0.3)
    assert controller.step(0.3).mode == MODE_WALK


def test_tilt_also_checks_negative_pitch():
    controller = ready()
    controller.update_state(0.0, -0.6, 0.1)
    assert controller.tilted


def test_body_offset_is_clamped_and_uses_force_stand():
    controller = ready()
    controller.set_body_offset(-1.0)
    command = controller.step(0.0)
    assert (command.mode, command.body_height) == (MODE_FORCE_STAND, -0.1)


def test_stand_down_then_damp_then_refuses_up():
    controller = ready()
    assert controller.request_down(0.0)[0]
    assert controller.step(1.0).mode == MODE_STAND_DOWN
    assert not controller.request_damp()[0]
    assert controller.step(2.0).mode == MODE_STAND_DOWN
    assert controller.posture is Posture.DOWN
    assert controller.request_damp()[0]
    assert controller.step(3.0).mode == MODE_DAMPING
    assert not controller.request_up(3.0)[0]


def test_stand_up_returns_to_standing():
    controller = ready()
    controller.request_down(0.0)
    controller.step(2.0)
    assert controller.request_up(2.0)[0]
    assert controller.step(3.0).mode == MODE_STAND_UP
    assert controller.step(4.0).mode == MODE_STAND_UP
    assert controller.posture is Posture.STANDING
    assert controller.step(4.1).mode == MODE_IDLE


def test_no_walking_while_down():
    controller = ready()
    controller.request_down(0.0)
    controller.set_velocity(0.1, 0.0, 0.0, 0.1)
    assert controller.step(0.1).mode == MODE_STAND_DOWN


def test_velocity_sent_before_stand_down_is_dropped_after_stand_up():
    controller = ready(4.0)
    controller.set_velocity(0.1, 0.0, 0.0, 0.0)
    controller.request_down(0.0)
    controller.step(2.0)
    controller.request_up(2.0)
    controller.step(4.0)
    assert controller.step(4.1).mode == MODE_IDLE


def test_requests_refused_from_wrong_posture():
    controller = ready()
    assert not controller.request_up(0.0)[0]
    assert not controller.request_damp()[0]


def test_non_finite_velocity_is_refused():
    controller = ready()
    with pytest.raises(ValueError):
        controller.set_velocity(float('nan'), 0.0, 0.0, 0.0)
    assert controller.step(0.0).mode == MODE_IDLE


def test_state_freshness():
    controller = Controller(Limits())
    assert not controller.state_is_fresh(0.0)
    controller.update_state(0.0, 0.0, 0.0)
    assert controller.state_is_fresh(0.5)
    assert not controller.state_is_fresh(0.6)

import math

from go1_control.avoid import avoid

N = 360
INC = 2 * math.pi / N
MIN = -math.pi


def scan_with(obstacles):
    """obstacles: {angle_rad: range_m}; every other beam sees nothing."""
    ranges = [50.0] * N
    for angle, r in obstacles.items():
        ranges[round((angle - MIN) / INC) % N] = r
    return ranges


def run(obstacles, cmd=(0.2, 0.0, 0.0)):
    return avoid(cmd, scan_with(obstacles), MIN, INC, 40.0)


def test_open_space_keeps_command():
    assert run({}) == (0.2, 0.0, 0.0)


def test_wall_ahead_stops_forward_motion():
    vx, _, _ = run({0.0: 0.3})
    assert vx <= 0.0


def test_wall_on_left_pushes_right_and_turns_right():
    _, vy, wz = run({math.pi / 2: 0.3})
    assert vy < 0.0 and wz < 0.0


def test_wall_on_right_pushes_left_and_turns_left():
    _, vy, wz = run({-math.pi / 2: 0.3})
    assert vy > 0.0 and wz > 0.0


def test_far_wall_does_not_change_command():
    assert run({0.0: 2.0}) == (0.2, 0.0, 0.0)


def test_body_returns_and_no_return_are_ignored():
    assert run({0.0: 0.05}) == (0.2, 0.0, 0.0)
    assert run({0.0: 50.0}) == (0.2, 0.0, 0.0)


def test_closer_wall_pushes_harder():
    near = run({math.pi / 2: 0.2})
    far = run({math.pi / 2: 0.5})
    assert abs(near[1]) > abs(far[1])


def test_wall_dead_ahead_stops_and_turns_left():
    vx, _, wz = run({0.0: 0.4})
    assert vx <= 0.0 and wz > 0.0


def test_wide_wall_does_not_explode_the_command():
    wall = {math.radians(a): 0.5 for a in range(40, 80)}
    _, vy, wz = run(wall)
    assert abs(vy) <= 1.0 and abs(wz) <= 2.0

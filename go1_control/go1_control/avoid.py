"""Keeps the Go1 away from walls using /scan. Pure functions, no ROS imports, so they can be tested.

Every beam closer than safe_distance pushes the command away from that obstacle. The result is
added to the wanted velocity, so the robot still follows the program but slows down and turns
away from walls. Angles follow ROS: 0 is forward, positive is to the left (counter-clockwise).
"""

from dataclasses import dataclass
import math

# Readings closer than this are the robot's own body, further than range_max are "no return".
BODY_RADIUS = 0.15
NO_RETURN = 50.0


@dataclass
class AvoidParams:
    safe_distance: float = 0.8
    gain_x: float = 0.5
    gain_y: float = 0.5
    gain_yaw: float = 1.0
    # Beams within this angle of straight ahead can stop forward motion.
    front_sector: float = math.radians(35)


def obstacles(ranges, angle_min, angle_increment, range_max, safe_distance):
    """Returns (x, y, strength) for every beam nearer than safe_distance, in the body frame."""
    points = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < BODY_RADIUS or r >= min(range_max, NO_RETURN):
            continue
        if r >= safe_distance:
            continue
        angle = angle_min + i * angle_increment
        strength = (safe_distance - r) / safe_distance
        points.append((r * math.cos(angle), r * math.sin(angle), strength))
    return points


def avoid(cmd, ranges, angle_min, angle_increment, range_max, params=None):
    """cmd is (vx, vy, wz) wanted by the program. Returns the (vx, vy, wz) to send."""
    p = params or AvoidParams()
    vx, vy, wz = cmd
    push_x = push_y = 0.0
    front_block = False
    for x, y, s in obstacles(ranges, angle_min, angle_increment, range_max, p.safe_distance):
        d = math.hypot(x, y)
        # Push directly away from the obstacle, harder the closer it is.
        push_x -= s * x / d
        push_y -= s * y / d
        if abs(math.atan2(y, x)) < p.front_sector and vx > 0.0:
            front_block = True
    # A wide wall gives many beams, so the push is capped to keep one wall from dominating.
    size = math.hypot(push_x, push_y)
    if size > 1.0:
        push_x, push_y = push_x / size, push_y / size
    if front_block:
        # Nothing ahead: do not walk forward into it, only back off or turn.
        vx = min(vx, 0.0)
        if abs(push_y) < 0.1:
            # Wall straight ahead: no side to turn away from, so turn left.
            push_y = 0.1
    vx += p.gain_x * push_x
    vy += p.gain_y * push_y
    wz += p.gain_yaw * push_y
    return vx, vy, wz

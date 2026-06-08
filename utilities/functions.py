import math


def constrain_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi,pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(val: float, low: float, high: float) -> float:
    return max(min(val, high), low)


def sign(x: float) -> float:
    """Compute the sign of a scalr value and return it"""
    if x > 0.0:
        return 1.0
    elif x < 0.0:
        return -1.0
    else:
        return 0.0

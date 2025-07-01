import math

def angle_wrap(a):
    """Wrap an angle (in radians) to the range [-π, π]."""
    if a > math.pi:
        a -= 2 * math.pi
    elif a < -math.pi:
        a += 2 * math.pi
    return a    
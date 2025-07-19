
import math
def angle_wrap(a):
    """Wrap angle to [-pi, pi] range"""
    if a > math.pi:
        a = a - 2 * math.pi
    elif a < -math.pi:
        a = a + 2 * math.pi
    return a

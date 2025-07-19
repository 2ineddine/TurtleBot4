import numpy as np

def AngleWrap(a):
    return ((a + np.pi) % (2*np.pi)) - np.pi

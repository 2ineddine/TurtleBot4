import numpy as np
from math import sin, cos, pi

def tcomp(tab, tbc):
    """
    Composes two transformations (tac = tab ⊕ tbc)
    
    Parameters:
    tab : numpy array (3,) - first transformation [x, y, theta]
    tbc : numpy array (3,) - second transformation [x, y, theta]
    
    Returns:
    tac : numpy array (3,) - composed transformation
    """
    # Input validation
    if tab.shape != (3,):
        raise ValueError('TCOMP: tab is not a transformation! Expected shape (3,)')
    if tbc.shape != (3,):
        raise ValueError('TCOMP: tbc is not a transformation! Expected shape (3,)')
    
    # Angle composition with wrapping
    result = tab[2] + tbc[2]
    result = AngleWrap(result)
    
    # Position composition
    s = sin(tab[2])
    c = cos(tab[2])
    rotation_matrix = np.array([[c, -s], [s, c]])
    composed_position = tab[:2] + rotation_matrix @ tbc[:2]
    
    # Return composed transformation
    tac = np.array([composed_position[0], composed_position[1], result])
    return tac

def AngleWrap(a):
    """Wrap angle to [-π, π] range"""
    while a > pi:
        a -= 2 * pi
    while a <= -pi:
        a += 2 * pi
    return a
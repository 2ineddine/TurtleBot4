import numpy as np
from math import sin, cos

def tinv(tab):
    """
    Calculates the inverse of one or more transformations
    
    Parameters:
    tab : numpy array (3,) or (3N,) - transformation(s) to invert
    
    Returns:
    tba : numpy array - inverted transformation(s)
    """
    if len(tab.shape) == 1:  # Single transformation
        if tab.shape[0] != 3:
            raise ValueError('tinv: tab must be a 3-element vector or multiple of 3')
        return tinv1(tab)
    else:  # Multiple transformations
        if tab.shape[0] % 3 != 0:
            raise ValueError('tinv: tab length must be multiple of 3')
        tba = np.zeros_like(tab)
        for t in range(0, tab.shape[0], 3):
            tba[t:t+3] = tinv1(tab[t:t+3])
        return tba

def tinv1(tab):
    """
    Calculates the inverse of a single transformation
    
    Parameters:
    tab : numpy array (3,) - [x, y, theta] transformation
    
    Returns:
    tba : numpy array (3,) - inverted transformation
    """
    s = sin(tab[2])
    c = cos(tab[2])
    tba = np.array([
        -tab[0]*c - tab[1]*s,
        tab[0]*s - tab[1]*c,
        -tab[2]
    ])
    return tba
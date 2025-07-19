import numpy as np
import math

def tinv(tab):
    """
    Calculates the inverse of one or more transformations
    
    Author:  Jose Neira
    Version: 1.0, 7-Dic-2000
    
    Args:
        tab: Transformation(s) as numpy array. Can be:
             - Single transformation: [x, y, theta] (3 elements)
             - Multiple transformations: [x1, y1, theta1, x2, y2, theta2, ...] (3n elements)
    
    Returns:
        tba: Inverse transformation(s) with same shape as input
    """
    tab = np.array(tab)
    tba = np.zeros(tab.shape)
    
    # Process transformations in groups of 3
    for t in range(0, tab.shape[0], 3):
        tba[t:t+3] = tinv1(tab[t:t+3])
    
    return tba

def tinv1(tab):
    """
    Calculates the inverse of one transformation
    
    Args:
        tab: Single transformation [x, y, theta]
        
    Returns:
        tba: Inverse transformation [x_inv, y_inv, theta_inv]
    """
    s = math.sin(tab[2])
    c = math.cos(tab[2])
    
    tba = np.array([
        -tab[0] * c - tab[1] * s,
         tab[0] * s - tab[1] * c,
        -tab[2]
    ])
    
    return tba
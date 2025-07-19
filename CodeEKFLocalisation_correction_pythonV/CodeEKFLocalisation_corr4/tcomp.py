import numpy as np

def tcomp(tab, tab2):
    s = np.sin(tab[2])
    c = np.cos(tab[2])
    
    x = tab[0] + tab2[0]*c - tab2[1]*s
    y = tab[1] + tab2[0]*s + tab2[1]*c
    theta = tab[2] + tab2[2]
    
    # Angle wrapping
    theta = ((theta + np.pi) % (2*np.pi)) - np.pi
    
    return np.array([x, y, theta])

import numpy as np
import math
def tcomp(tab, tbc):
    """Compose two transformations"""
    tab = np.array(tab)
    tbc = np.array(tbc)
    
    if tab.shape[0] != 3:
        raise ValueError('TCOMP: tab is not a transformation!!!')
    if tbc.shape[0] != 3:
        raise ValueError('TCOMP: tbc is not a transformation!!!')
    
    result = tab[2] + tbc[2]
    if result > math.pi or result <= -math.pi:
        result = angle_wrap(result)
    
    s = math.sin(tab[2])
    c = math.cos(tab[2])
    rotation_matrix = np.array([[c, -s], [s, c]])
    
    tac = np.concatenate([
        tab[0:2] + rotation_matrix @ tbc[0:2],
        [result]
    ])
    
    return tac

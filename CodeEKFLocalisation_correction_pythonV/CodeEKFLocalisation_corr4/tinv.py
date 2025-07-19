import numpy as np

def tinv(tab):
    tba = np.zeros(tab.shape)
    for t in range(0, tab.shape[0], 3):
        tba[t:t+3] = tinv1(tab[t:t+3])
    return tba

def tinv1(tab):
    s = np.sin(tab[2])
    c = np.cos(tab[2])
    tba = np.array([-tab[0]*c - tab[1]*s,
                    tab[0]*s - tab[1]*c,
                    -tab[2]])
    return tba

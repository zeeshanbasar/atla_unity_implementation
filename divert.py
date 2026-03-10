import numpy as np

def divert(r, rho):

# Dist from barrier

    d = r - rho

# Params

    l1 = np.array([1,1,1])
    l2 = np.array([1,1,1])*9500.0
    l3 = np.array([1,1,1])*500.0

# Augmentation

    phi = l2/(d**2 + l1)
    b = np.exp(-phi)
    p = (d*l2*l3*b)/((d**2 + l1)**2)

    
    return d, p
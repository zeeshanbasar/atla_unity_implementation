import numpy as np

def sliding(r, rf, v, vf, tgo, lam=2):

#  Surface 1

    s1 = r - rf
    s1_dot = v - vf

# Surface 2

    s2 = s1_dot + (lam/tgo)*s1


    return s1, s2
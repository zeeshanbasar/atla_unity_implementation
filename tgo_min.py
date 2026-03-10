import numpy as np

def tgo_init(r0, v0, rf, vf):

    g = np.array([0,0,-3.7114])

    A = -2.0*(np.dot(v0,v0) + np.dot(vf,v0) + np.dot(vf,vf))
    B = 12.0*np.dot((rf - r0),(v0 + vf))
    C = -18.0*np.dot((rf - r0),(rf - r0))
    D = 0.5*np.dot(g,g)

    r = np.roots([D, 0, A, B, C])
    r = np.real(r[np.logical_and(np.imag(r) == 0,np.real(r) >= 0)])

    tgo = np.min(r)

    return tgo
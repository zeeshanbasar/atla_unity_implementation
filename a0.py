import numpy as np
from sliding import sliding
from barriers import barriers
from divert import divert
from saturation import saturation

def initAcc(r0, v0, rf, vf, eps, tf, highest, t=0):

# Planet params

    g = np.array([0,0,-3.7114])

# Sliding term

    _, s20 = sliding(r0, rf, v0, vf, tf)

# ZEM/ZEV term

    ZEM0 = rf - (r0 + v0*tf + 0.5*g*tf**2)
    ZEV0 = vf - (v0 + g*tf)

# Barriers

    rho, t_last = barriers(r0, rf, highest, rho_last=np.zeros((3,1)), t_last=0, t=0)

# Divert

    _,p0 = divert(r0, rho)

# Init acceleration

    m0 = 1905.0
    m_min = m0 - 450.0
    T_max = 8.0*3100.0/np.sqrt(3)
    k1, k2 = 0.2, 0.8*np.diag([1,1,1])
    ap_max = 0.3*(T_max/m_min)
    phi0 = k1*ap_max + np.dot((k2/12),np.abs(p0))*(tf**2)

    a0 = ((6*ZEM0/tf**2) - (2*ZEV0/tf) + (p0/12)*(tf**2) - phi0*saturation(s20, eps))


    return a0, t_last, rho
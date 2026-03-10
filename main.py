import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D 
import scipy.io

from rk45 import rk45
from tgo_min import tgo_init
from a0 import initAcc
from barriers import barriers
from divert import divert
from sliding import sliding
from saturation import saturation

# Initial conditions
r0 = np.array([704, 145, 721])
v0 = np.array([-55.942177385232,22.8833761299102,-83.8070545831464])
m0 = 1905.0

# Terminal conditions
rf = np.array([453, 786, 145])
vf = np.array([0,0,0.5])
tf = tgo_init(r0, v0, rf, vf) + 20

# ------------ FIX, this part comes from Unity ------------ #

# Crude estimate of highest point
highest = np.array([725, 581, 224]) #<-- right now i have just taken a wild guess about where it actually is

# --------------------------------------------------------- #

# Params
eps = 0.1
N = int(10*tf + 1)
t0 = 0.0
a0, t_last, rho_init = initAcc(r0, v0, rf, vf, eps, tf, highest)
Isp = 225
tau = 5/90
m_min = m0 - 450
g = np.array([0,0,-3.7114])
ge = 9.807
k1, k2 = 0.2, 0.8*np.diag([1,1,1])
Tmax = 8*3100/np.sqrt(3)
ap_max = 0.3*(Tmax/m_min)

y0 = np.concat([r0,v0,[m0],a0])
state = {'t_last': 0.0,
         "rho_last": rho_init}

def f(t,y):

    tgo = tf - t
    r = y[0:3]
    v = y[3:6]

    ZEM = rf - (r + v*tgo + 0.5*g*tgo**2)
    ZEV = vf - (v + g*tgo)

    rho, t_last_new = barriers(r,rf,highest,state['rho_last'],state['t_last'],t=t)
    state['t_last'] = t_last_new
    state['rho_last'] = rho

    _, p = divert(r, rho)

    _, s2 = sliding(r,rf,v,vf,tgo)
    phi = k1*ap_max + np.dot((k2/12),np.abs(p))*(tgo**2)

    a_ogl = 6*ZEM/(tgo**2) - 2*ZEV/tgo
    divert_acc = (p/12)*tgo**2
    a_smc = -phi*saturation(s2,eps)

    a = a_ogl + divert_acc + a_smc

    T = np.array([y[7],y[8],y[9]])*y[6]

    T_mag = np.linalg.norm(T)
    T_dir = T/T_mag
    T_max = 10*3100

    if T_mag > T_max:
        T = T_max*T_dir
    else:
        T = T_mag*T_dir

    Tu, Tl = 1.05*T, 0.95*T
    T = np.array([np.random.uniform(Tu[0],Tl[0]),
                  np.random.uniform(Tu[1],Tl[1]),
                  np.random.uniform(Tu[2],Tl[2])])
    
    ap = np.array([0,0,0])

    dx = np.array([y[3],
                   y[4],
                   y[5],
                   g[0] + (T[0]/y[6]) + ap[0],
                   g[1] + (T[1]/y[6]) + ap[1],
                   g[2] + (T[2]/y[6]) + ap[2],
                   -np.linalg.norm(T)/(Isp*ge),
                   (a[0] - y[7])/tau,
                   (a[1] - y[8])/tau,
                   (a[2] - y[9])/tau])
    
    return dx

t,y = rk45(f,t0,y0,rf,tf=tf)

np.savez('rk45_results.npz', t=t, y=y)


pass
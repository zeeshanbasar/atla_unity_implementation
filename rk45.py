import numpy as np

def rk45(f, t0, y0, rf, tf, h=0.001, tol=1e-6):
    """
    Runge-Kutta-Fehlberg (RK45) adaptive step size method.
    
    f: function dy/dt = f(t, y)
    t0: initial time
    y0: initial conditions
    rf: final conditions
    tf: final time
    h: initial step size
    tol: tolerance for error control
    
    Returns early if any element of y becomes inf, -inf, or nan.
    """
    # Butcher tableau coefficients for RK45
    a = np.array([0, 1/4, 3/8, 12/13, 1, 1/2])
    b = np.array([
        [0, 0, 0, 0, 0],
        [1/4, 0, 0, 0, 0],
        [3/32, 9/32, 0, 0, 0],
        [1932/2197, -7200/2197, 7296/2197, 0, 0],
        [439/216, -8, 3680/513, -845/4104, 0],
        [-8/27, 2, -3544/2565, 1859/4104, -11/40]
    ])
    c4 = np.array([25/216, 0, 1408/2565, 2197/4104, -1/5, 0])
    c5 = np.array([16/135, 0, 6656/12825, 28561/56430, -9/50, 2/55])
    
    t = t0
    y = np.array(y0, dtype=float)
    t_vals = [t]
    y_vals = [y.copy()]
    
    while t < tf:
        if t + h > tf:
            h = tf - t
        
        k = np.zeros((6, len(y)))
        k[0] = f(t, y)
        k[1] = f(t + a[1]*h, y + h*b[1,0]*k[0])
        k[2] = f(t + a[2]*h, y + h*(b[2,0]*k[0] + b[2,1]*k[1]))
        k[3] = f(t + a[3]*h, y + h*(b[3,0]*k[0] + b[3,1]*k[1] + b[3,2]*k[2]))
        k[4] = f(t + a[4]*h, y + h*(b[4,0]*k[0] + b[4,1]*k[1] + b[4,2]*k[2] + b[4,3]*k[3]))
        k[5] = f(t + a[5]*h, y + h*(b[5,0]*k[0] + b[5,1]*k[1] + b[5,2]*k[2] + b[5,3]*k[3] + b[5,4]*k[4]))
        
        y4 = y + h*np.sum(c4[:, np.newaxis]*k, axis=0)
        y5 = y + h*np.sum(c5[:, np.newaxis]*k, axis=0)
        
        # Check for invalid values before accepting the step
        if not np.all(np.isfinite(y5)):
            # Stop integration and return results up to this point
            break

        if np.linalg.norm(y5[0:3] - rf) < 0.1:
            break
        
        error = np.max(np.abs(y5 - y4))
        
        if error < tol or h < 1e-10:
            t += h
            y = y5
            t_vals.append(t)
            y_vals.append(y.copy())
            
            # Check again after accepting the step
            if not np.all(np.isfinite(y)):
                break
        
        if error > 0:
            h = 0.9 * h * (tol/error)**0.2
        else:
            h = 2 * h
    
    return np.array(t_vals), np.array(y_vals)
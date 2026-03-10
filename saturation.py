import numpy as np

def saturation(x, width=1.0):
    """
    Saturation function with boundary layer width.
    
    x: input value or array
    width: boundary layer width (default=1.0)
    
    Returns: sat(x) = x/width if |x| < width, sign(x) otherwise
    """
    return np.where(np.abs(x) < width, x / width, np.sign(x))
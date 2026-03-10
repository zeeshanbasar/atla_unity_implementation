import numpy as np
import socket
import json
from skimage.morphology import extrema
from skimage.measure import regionprops, label
from TerrainMap.unityTerrainController import UnityTerrainController

theta = 1000
delta_r = 95.5
m = 0.5
controller = UnityTerrainController()

def barriers(r, rf, highest, rho_last, t_last, t):
    rho = np.zeros((3,))

    if np.linalg.norm(r-rf) < theta:
        delta_t = 0.05
    else:
        delta_t = 0.1
        
    if abs(t-t_last) >= delta_t or t == 0.0:
        
        t_last = t

        temp = r.tolist()
        controller.move_and_export(temp[0],temp[2],temp[1])
        terrain_data = controller.load_terrain_data()
        x,y,z = terrain_data['x'], terrain_data['y'], terrain_data['z']

        im = extrema.h_maxima(z, 10)
        labeled = label(im)
        regionProps = regionprops(labeled)
        centroid_row, centroid_col = regionProps[0].centroid
        
        r_peak = np.array([x[int(centroid_col)],y[int(centroid_row)],z[int(centroid_row),int(centroid_col)]])

        if r[2] - highest[2] > delta_r:
            rho[2] = highest[2] + delta_r
            b = highest
        else:
            rho[2] = r_peak[2] + delta_r
            b = r_peak

        for coord in range(2):
            if b[coord] > rf[coord]:
                i = ((b[coord] - rf[coord])**m)/(b[2] - rf[2])
                j = ((b[coord] - rf[coord])**m) - i*b[2]
                k = rf[2]
                rho[coord] = ((i*r[2] + j)**(1/m)) + k
            elif b[coord] < rf[coord]:
                p = ((-b[coord] + rf[coord])**m)/(b[2] - rf[2])
                q = ((-b[coord] + rf[coord])**m) - p*b[2]
                T = -rf[2]
                rho[coord] = -(((p*r[2] + q)**(1/m)) + T)
            else:
                rho[coord] = np.inf



        return rho.reshape(3,), t_last
    
    else:

        rho = rho_last

        return rho.reshape(3,), t_last

        
    
# r = np.array([340,350,500])
# v = np.array([0,0,0])
# rf = np.array([0,0,0])
# t = 0.0
# highest = np.array([0,0,0])
# t_last = 0.0

# barriers(r,v,rf,t,highest,t_last)
# pass
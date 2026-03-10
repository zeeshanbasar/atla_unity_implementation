import numpy as np
import matplotlib.pyplot as plt
import time
from TerrainMap.unityTerrainController import UnityTerrainController

output_path = r"C:\[[WORK]]\[[AERO]]\0. Thesis\CodeBase\8. OTBU Py Migration\rk45_results.npz"
data = np.load(output_path)

t = data['t']
y = data['y']

r = y[:,0:3]
v = y[:,3:6]
m = y[:,6]
a = y[:,7:10]

controller = UnityTerrainController()

for i in range(len(r)):

    controller.move_camera(r[i,0],r[i,2],r[i,1])
    time.sleep(0.001)


plt.plot(t,m,label='Mass (kg)')
plt.grid()
plt.legend()
plt.show()

plt.plot(t,a,label=['ax','ay','az'])
plt.grid()
plt.legend()
plt.show()

pass


import matplotlib.pyplot as plt
import control as ctr
import numpy as np

G = ctr.tf([1, 1],[1,1])
fig1 = plt.figure()
mag, phase, omega = ctr.nyquist(G,[0.01,1000])
plt.grid()
fig2 = plt.figure()
mag, phase, omega = ctr.bode(G,np.geomspace(0.01,100,50))
plt.show()
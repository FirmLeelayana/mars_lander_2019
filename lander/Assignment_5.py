import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt('trajectories2.txt')
plt.figure(1)
plt.clf()
plt.xlabel('descent rate (ms-1)')
plt.grid()
plt.plot(results[:, 1], results[:, 0])
plt.legend()
plt.show()
# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 1000
dt = 1.01
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []

# Verlet integration
# calculate new position and velocity
x_list.append(x)
v_list.append(v)
a = -k * x / m
x = x + dt * v
v = v + dt * a

for i in range(len(t_array)-1):

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate new position and velocity
    a = -k * x / m
    x2 = x
    x = 2 * x - x_list[-2]  + (dt**2) * a
    v = (1 / dt) * (x - x2)

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)


# plot the position-time graph
plt.figure(1)
plt.clf()
plt.title('Verlet Graph')
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()

# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, initial position, velocity, distance from centre of planet, gravitational field strength (in KMETERS!)
M = 6.42e23
x = np.array([3400,0,0])
v = np.array([0,0,0])
r = np.linalg.norm(x)
#3389 - radius of mars
G = 6.67e-11

# simulation time, timestep and time
t_max = 100
dt = 0.01
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []

# Euler integration
for t in t_array:
    if abs(r) < 3389:
        x = np.array([3389,0,0])
        v = np.array([0,0,0])
        a = np.array([0,0,0])
        x_list.append(x)
        v_list.append(v)
    else:
        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity and distance from centre
        a = - ((G * M)/((abs(r)*1000)**3))*x
        x = x + dt * v
        v = v + dt * a
        r = np.linalg.norm(x)

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)


# plot the position-time graph
plt.figure(1)
plt.clf()
plt.title('Euler Graph')
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array[:, 0], label='altitude (km)')
#plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()

# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 2

m_verlet = 1
k_verlet = 1
x_verlet = 0
v_verlet = 1

# simulation time, timestep and time
t_max = 1000
dt = 1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []

x_list_verlet = []
v_list_verlet = []

# Euler integration
for t in t_array:

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)
    
    
    x_list_verlet.append(x)
    v_list_verlet.append(v)
    
    
    # calculate new position and velocity
    a = -k * x / m
    x = x + dt * v
    v = v + dt * a


        # applying the verlet method, within the same loop
    x_verlet = 2*x_verlet - x_list_verlet[-1] + ((dt)**2)*(-k_verlet*x_verlet/m_verlet)


    v_verlet = (x_verlet-x_list_verlet[-1])/dt





# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)

x_array_verlet = np.array(x_list_verlet)
v_array_verlet = np.array(v_list_verlet)


# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m) euler')
plt.plot(t_array, v_array, label='v (m/s) euler')
plt.plot(t_array, x_array_verlet, label='x (m) verlet')
plt.plot(t_array, v_array_verlet, label='v (m/s) verlet')

plt.legend()
plt.show()

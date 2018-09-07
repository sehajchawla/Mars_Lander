import numpy as np
import matplotlib.pyplot as plt

def vector_magnitude(vect):
	magnitude = ((vect[0])**2) + ((vect[1])**2) + ((vect[2])**2)
	magnitude = magnitude**(0.5)
	return(magnitude)

m = 1000 #mass of satellite
M = 6.42*(10**23) #mass of mars
R = 3390000 #mars radius
dt = 0.01 #time step
G = 6.67*(10**(-11)) #gravitational constant
t_max = 200 #length of simulation
x = np.array([R+1000, 0, 0])
escape_vel = ((G*M)/(vector_magnitude(x)))**0.5
print(escape_vel)
v = np.array([0, 0, 0])
x_list = []
v_list = []
x_coordinate = []
y_coordinate = []
z_coordinate = []
a_list = []
t_array = np.arange(0, t_max, dt)



for t in t_array:
	
	x_list.append(x)
	v_list.append(v)

	a = -((G*M)/((vector_magnitude(x))**4))*x
	v = v + dt*a
	x = x + dt*v

	if vector_magnitude(x) < R:
		break

for i in x_list:
	x_coordinate.append(i[0])
	y_coordinate.append(i[1])
	z_coordinate.append(i[2])

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)

plt.figure(1)
plt.clf()
plt.grid()
plt.scatter(x_coordinate, y_coordinate)
plt.show()

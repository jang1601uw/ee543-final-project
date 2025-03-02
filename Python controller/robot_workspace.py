import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("/Users/jang1601/Desktop/EE 543")

from ee543funcs import *

from mpl_toolkits import mplot3d

d2r = 2*np.pi/360.0
deg2rad = d2r

# Number of samples
num_samples = 5000 

# Robot arm configuration
L1 = 61.8
L2 = 100.
L3 = 52.5 
L4 = 184.39

theta_bounds = np.array([-90,90])

t1_i = 180
t2_i = 90
t3_i = 90
t4_i = 0 

# take num_samples random samples within joint constraints
t1 = np.random.uniform(theta_bounds[0],theta_bounds[1],num_samples)
t2 = np.random.uniform(theta_bounds[0],theta_bounds[1],num_samples)
t3 = np.random.uniform(theta_bounds[0],theta_bounds[1],num_samples)
t4 = np.random.uniform(theta_bounds[0],theta_bounds[1],num_samples)

P = np.zeros((3,num_samples))

for i in range(num_samples):
    t1_val = t1[i]
    t2_val = t2[i]
    t3_val = t3[i]
    t4_val = t4[i]

    DH = np.array([
        [0, 0, L1, t1_i+t1_val],
        [90, 0, 0, t2_i+t2_val],
        [0, L2, 0, t3_i+t3_val],
        [90, 0, 0, t4_val],
        [0, 0, L3+L4, 0]
    ])

    T0_1 = Link_N(DH[0,0],DH[0,1],DH[0,2],DH[0,3])
    T1_2 = Link_N(DH[1,0],DH[1,1],DH[1,2],DH[1,3])
    T2_3 = Link_N(DH[2,0],DH[2,1],DH[2,2],DH[2,3])
    T3_4 = Link_N(DH[3,0],DH[3,1],DH[3,2],DH[3,3])
    T4_e = Link_N(DH[4,0],DH[4,1],DH[4,2],DH[4,3])

    T = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_e

    P[:,i] = T[0:3,3]

x_pts = P[0,:]
y_pts = P[1,:]
z_pts = P[2,:]

# Plot in 3D
fig = plt.figure(figsize = (8,8))
ax = plt.axes(projection='3d')
ax.grid()
ax.scatter(x_pts,y_pts,z_pts)
ax.set_xlabel('x', labelpad=20)
ax.set_ylabel('y', labelpad=20)
ax.set_zlabel('z', labelpad=20)

plt.show()

fig2 = plt.figure("Figure 2")
plt.scatter(x_pts,z_pts)
plt.show()

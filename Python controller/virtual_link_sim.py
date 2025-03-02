import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

import sys
sys.path.append("/Users/jang1601/Desktop/EE 543")

from ee543funcs import *

# Define the lengths of the robot links (adjust as needed)
link_lengths = [61.8, 100., 52.5, 184.39] # Example link lengths for a 4DOF robot

# Function to calculate the end effector position based on joint angles
def forward_kinematics(theta1, theta2, theta3, theta4):
    # 3D positions of the links
    x = np.array([0, 
                  link_lengths[0] * np.cos(theta1), 
                  link_lengths[1] * np.cos(theta1 + theta2), 
                  link_lengths[2] * np.cos(theta1 + theta2 + theta3),
                  link_lengths[3] * np.cos(theta1 + theta2 + theta3 + theta4)])
    
    y = np.array([0, 
                  link_lengths[0] * np.sin(theta1), 
                  link_lengths[1] * np.sin(theta1 + theta2), 
                  link_lengths[2] * np.sin(theta1 + theta2 + theta3),
                  link_lengths[3] * np.sin(theta1 + theta2 + theta3 + theta4)])

    # Adding z-coordinate for the 3D plot (can also be used for more complex robots)
    z = np.array([0, 
                  link_lengths[0] * np.sin(theta1) * 0.5, 
                  link_lengths[1] * np.cos(theta2) * 0.5, 
                  link_lengths[2] * np.sin(theta3) * 0.5, 
                  link_lengths[3] * np.cos(theta4) * 0.5])

    return x, y, z

# Initialize the figure and axes for 3D plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-400, 400)
ax.set_ylim(-400, 400)
ax.set_zlim(-400, 400)

# Line object for plotting robot links
line, = ax.plot([], [], [], marker='o', markersize=6, lw=2)

# Function to initialize the animation
def init():
    line.set_data([], [])
    line.set_3d_properties([], [])
    return line,

# Function to update the animation
def update(frame):
    # Define joint angles (theta1, theta2, theta3, theta4) for the current frame
    theta1 = np.sin(frame * 0.1) * np.pi / 2
    theta2 = np.cos(frame * 0.1) * np.pi / 4
    theta3 = np.sin(frame * 0.1) * np.pi / 4
    theta4 = np.cos(frame * 0.1) * np.pi / 6

    # Get the x, y, and z positions of the robot's joints using forward kinematics
    x, y, z = forward_kinematics(theta1, theta2, theta3, theta4)
    
    # Update the plot with the new joint positions
    line.set_data(x, y)
    line.set_3d_properties(z)
    return line,

# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 100, 1), init_func=init, blit=True)

# Display the animation
plt.show()

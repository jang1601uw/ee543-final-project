import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

# Define robot link lengths
link_lengths = [61.8, 100, 100, 100]  # Lengths of each arm segment in mm

# Define initial joint angles (in degrees)
joint_angles = [0, 0, 0, 0]  

# Function to compute transformation matrix using DH parameters
def dh_transformation(alpha, a, d, theta):
    theta = np.radians(theta)
    alpha = np.radians(alpha)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0, 0, 0, 1]
    ])

# Function to compute robot joint positions
def compute_joint_positions(joint_angles):
    # DH Parameters: [alpha, a, d, theta]
    dh_params = [
        [0, 0, link_lengths[0], joint_angles[0]],
        [90, 0, 0, joint_angles[1]],
        [0, link_lengths[1], 0, joint_angles[2]],
        [0, link_lengths[2], 0, joint_angles[3]]
    ]
    
    T = np.eye(4)  # Identity matrix as starting point
    positions = [np.array([0, 0, 0])]  # Start at base
    
    for params in dh_params:
        T = T @ dh_transformation(*params)
        pos = T[:3, 3]  # Extract x, y, z position
        positions.append(pos)
    
    return np.array(positions)

# Function to plot robot arm
def plot_robot(joint_angles):
    positions = compute_joint_positions(joint_angles)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-200, 200])
    ax.set_ylim([-200, 200])
    ax.set_zlim([0, 300])
    
    # Plot robot arm
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], marker='o', markersize=8, color='b', linewidth=3)
    
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("3D Robot Arm Simulation")
    
    plt.show()

# Animate robot arm
for i in range(20):
    joint_angles = [i * 5, (i * 5) % 90, (i * 5) % 90, (i * 5) % 90]  # Simulating movement
    plot_robot(joint_angles)
    time.sleep(0.2)

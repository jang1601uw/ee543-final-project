import time
import numpy as np
import keyboard
import sys
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_controller import robot_controller  # Import your robot controller

def link_n(alpha, a, d, theta):
    """Compute transformation matrix using DH parameters."""
    theta_n = np.radians(theta)
    alpha_n_1 = np.radians(alpha)
    
    T = np.array([
        [np.cos(theta_n), -np.sin(theta_n), 0, a],
        [np.sin(theta_n) * np.cos(alpha_n_1), np.cos(theta_n) * np.cos(alpha_n_1), -np.sin(alpha_n_1), -np.sin(alpha_n_1) * d],
        [np.sin(theta_n) * np.sin(alpha_n_1), np.cos(theta_n) * np.sin(alpha_n_1), np.cos(alpha_n_1), np.cos(alpha_n_1) * d],
        [0, 0, 0, 1]
    ])
    return T

def compute_transformation_matrices(theta1, theta2, theta3, theta4):
    """Compute transformation matrices for a 4DOF robotic arm."""
    theta_offsets = [180, 90, 90, 0]

    T_0i = link_n(0, 0, 0, theta1 + theta_offsets[0])
    T_i1 = link_n(0, 0, 61.8, 0)
    T_12 = link_n(90, 0, 0, theta2 + theta_offsets[1])
    T_23 = link_n(0, 100, 0, theta3 + theta_offsets[2])
    T_34 = link_n(90, 0, 52.5, theta4 + theta_offsets[3])
    T_4e = link_n(0, 0, 184.39, 0)

    T1 = T_0i @ T_i1
    T2 = T1 @ T_12
    T3 = T2 @ T_23
    T4 = T3 @ T_34
    T_end = T4 @ T_4e

    return T1, T2, T3, T4, T_end

def update_plot():
    """Update the robot arm plot after 1 second of inactivity."""
    global theta1, theta2, theta3, theta4

    T1, T2, T3, T4, T_end = compute_transformation_matrices(theta1, theta2, theta3, theta4)

    x_points[:] = [0, T1[0, 3], T2[0, 3], T3[0, 3], T4[0, 3], T_end[0, 3]]
    y_points[:] = [0, T1[1, 3], T2[1, 3], T3[1, 3], T4[1, 3], T_end[1, 3]]
    z_points[:] = [0, T1[2, 3], T2[2, 3], T3[2, 3], T4[2, 3], T_end[2, 3]]

    arm_plot.set_data(x_points, y_points)
    arm_plot.set_3d_properties(z_points)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

def print_menu():
    os.system('cls' if os.name == 'nt' else 'clear')
    print('-----------------------------------------')
    print('EE543 Arm Keyboard Controller:')
    print('-----------------------------------------')
    print('[Exit]: 9')
    print('[Joint 1    +]: 1 | [Joint 1     -]: q')
    print('[Joint 2    +]: 2 | [Joint 2     -]: w')
    print('[Joint 3    +]: 3 | [Joint 3     -]: e')
    print('[Joint 4    +]: 4 | [Joint 4     -]: r')
    print('[Grasper Open]: 5 | [Grasper Close]: t')
    print('[Homing]: h')
    print('-----------------------------------------')

# Initialize robot controller
RC = robot_controller()
RC.communication_begin()
RC.joints_homing()

# Set movement parameters
keyboard_increment = 1
goals = np.zeros(RC.joint_num)
speeds = np.ones(RC.joint_num) * 80  # deg/s

# Initial joint angles
theta1, theta2, theta3, theta4 = 0, 0, 0, 0
last_input_time = time.time()

# Set up matplotlib figure
plt.ion()
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
arm_plot, = ax.plot([], [], [], '-o', color='k', markersize=8)

x_points = [0] * 6
y_points = [0] * 6
z_points = [0] * 6

ax.set_xlim([-200, 200])
ax.set_ylim([-200, 200])
ax.set_zlim([0, 400])
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.set_title("4DOF Robotic Arm")

# Display menu
print_menu()

while True:
    command = False

    # Check for keyboard input
    if keyboard.is_pressed('9'):
        os.system('cls' if os.name == 'nt' else 'clear')
        sys.exit('Closing Keyboard Controller')

    if keyboard.is_pressed('1'):
        theta1 += keyboard_increment
        goals[0] += keyboard_increment
        command = True

    if keyboard.is_pressed('q'):
        theta1 -= keyboard_increment
        goals[0] -= keyboard_increment
        command = True

    if keyboard.is_pressed('2'):
        theta2 += keyboard_increment
        goals[1] += keyboard_increment
        command = True

    if keyboard.is_pressed('w'):
        theta2 -= keyboard_increment
        goals[1] -= keyboard_increment
        command = True

    if keyboard.is_pressed('3'):
        theta3 += keyboard_increment
        goals[2] += keyboard_increment
        command = True

    if keyboard.is_pressed('e'):
        theta3 -= keyboard_increment
        goals[2] -= keyboard_increment
        command = True

    if keyboard.is_pressed('4'):
        theta4 += keyboard_increment
        goals[3] += keyboard_increment
        command = True

    if keyboard.is_pressed('r'):
        theta4 -= keyboard_increment
        goals[3] -= keyboard_increment
        command = True

    if keyboard.is_pressed('h'):
        print("Homing robot...")
        RC.joints_homing()  # Move to home position
        time.sleep(1)  # Allow movement to complete
        goals = RC.robot_homing_joint_poses.copy()  # Get updated positions

        # Update theta values from homing position
        theta1, theta2, theta3, theta4 = goals[:4]
        command = True

    if keyboard.is_pressed('5'):
        RC.gripper_set_percentage(0)

    if keyboard.is_pressed('t'):
        RC.gripper_set_percentage(100)

    # If a command was given, send it to the robot and update last input time
    if command:
        goals = np.clip(goals, RC.servo_angle_min, RC.servo_angle_max)
        RC.joints_goto(goals, speeds)
        last_input_time = time.time()

    # Update the plot **only if 0.5 seconds has passed without input**
    if time.time() - last_input_time > 0.5:
        update_plot()

    time.sleep(0.0001)  # Small delay to reduce CPU usage

plt.close()


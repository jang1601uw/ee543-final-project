import time
import numpy as np
from scipy.optimize import minimize
from pynput import keyboard
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_controller import robot_controller

d2r = 2*np.pi/360.0
deg2rad = d2r
rad2deg = 1/d2r

# Global variable to track the last key pressed
last_key = None

# which mode for inverse kinematics (symbolic or numerical)
symbolic = False

def on_press(key):
    global last_key
    try:
        last_key = key.char
    except AttributeError:
        pass

# Disable the default quit key ('q') in matplotlib so that pressing it won't close the plot.
plt.rcParams['keymap.quit'] = ''

# Global variable to store status message.
status_str = ""

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

    T_01 = link_n(0, 0, 0, theta1 + theta_offsets[0])
    T_1i = link_n(0, 0, 61.8, 0)
    T_12 = link_n(90, 0, 0, theta2 + theta_offsets[1])
    T_23 = link_n(0, 100, 0, theta3 + theta_offsets[2])
    T_34 = link_n(90, 0, 52.5, theta4 + theta_offsets[3])
    T_4e = link_n(0, 0, 184.39, 0)

    T0 = T_01
    T1 = T_01 @ T_1i
    T2 = T1 @ T_12
    T3 = T2 @ T_23
    T4 = T3 @ T_34
    T_end = T4 @ T_4e
    ee_loc = T_end[:3,3]

    return T0, T1, T2, T3, T4, T_end, ee_loc

def plot_axes(ax, T, length=30):
    """Plot coordinate axes for a transformation matrix T."""
    origin = T[:3, 3]
    x_axis = T[:3, 0] * length
    y_axis = T[:3, 1] * length
    z_axis = T[:3, 2] * length
    
    ax.quiver(*origin, *x_axis, color='r', linewidth=2)
    ax.quiver(*origin, *y_axis, color='g', linewidth=2)
    ax.quiver(*origin, *z_axis, color='b', linewidth=2)

def update_plot():
    """Update the robot arm plot."""
    global theta1, theta2, theta3, theta4, status_str

    T0, T1, T2, T3, T4, T_end, _ = compute_transformation_matrices(theta1, theta2, theta3, theta4)

    # Define points for the arm.
    # Index 0 is the base (0,0,0) which is not labeled.
    # Points 1-4 represent joints 1-4 and point 5 the end effector.
    x_points[:] = [0, T1[0, 3], T2[0, 3], T3[0, 3], T4[0, 3], T_end[0, 3]]
    y_points[:] = [0, T1[1, 3], T2[1, 3], T3[1, 3], T4[1, 3], T_end[1, 3]]
    z_points[:] = [0, T1[2, 3], T2[2, 3], T3[2, 3], T4[2, 3], T_end[2, 3]]

    # Clear the axes.
    ax.cla()
    ax.set_xlim([-200, 200])
    ax.set_ylim([-200, 200])
    ax.set_zlim([0, 400])
    ax.set_xlabel("X-axis", fontsize=8)
    ax.set_ylabel("Y-axis", fontsize=8)
    ax.set_zlabel("Z-axis", fontsize=8)
    ax.set_title("4DOF Robotic Arm", fontsize=10)

    # Plot coordinate axes at selected joints.
    plot_axes(ax, T0)
    plot_axes(ax, T2)
    plot_axes(ax, T3)
    plot_axes(ax, T4)

    # Plot connecting lines for the arm.
    ax.plot(x_points, y_points, z_points, '-k', alpha=0.5)

    # Plot individual joints with distinct colors and legend labels.
    ax.plot([x_points[0]], [y_points[0]], [z_points[0]], 'o', color='r', markersize=8, label="Joint 1")
    ax.plot([x_points[2]], [y_points[2]], [z_points[2]], 'o', color='g', markersize=8, label="Joint 2")
    ax.plot([x_points[3]], [y_points[3]], [z_points[3]], 'o', color='b', markersize=8, label="Joint 3")
    ax.plot([x_points[4]], [y_points[4]], [z_points[4]], 'o', color='m', markersize=8, label="Joint 4")
    ax.plot([x_points[5]], [y_points[5]], [z_points[5]], 'o', color='c', markersize=8, label="End Effector")

    # Place the legend at the left middle outside of the plot.
    ax.legend(fontsize=8, loc='center right', bbox_to_anchor=(0.1, 0.5))

    # Display end-effector position and joint angles.
    status_text = (f"End Position: X={T_end[0, 3]:.2f}, Y={T_end[1, 3]:.2f}, Z={T_end[2, 3]:.2f}\n"
                   f"Angles: θ1={theta1:.2f}, θ2={theta2:.2f}, θ3={theta3:.2f}, θ4={theta4:.2f}")
    ax.text2D(0.05, 0.95, status_text, transform=ax.transAxes, fontsize=8,
              verticalalignment='top', bbox=dict(facecolor='white', alpha=0.5))
    # Display the current status message (moving text) in the top right.
    ax.text2D(0.95, 0.95, "Status: " + status_str, transform=ax.transAxes, fontsize=8,
              horizontalalignment='right', verticalalignment='top', bbox=dict(facecolor='white', alpha=0.5))

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
    print('[Up      +]: "up" | [Down        -]: "down"')
    print('[Right+]: "right" | [Left        -]: "left"')
    print('[Fwd        +]: f | [Bk          -]: b')
    print('[Grasper Open]: 5 | [Grasper Close]: t')
    print('[Homing]: h')
    print('-----------------------------------------')

def print_no_newline(string):
    global status_str
    status_str = string  # Update the global status string.
    print("\r" + string, end="", flush=True)

def normalize_angle(angle, range=(-180, 180)):
    """
    Normalizes an angle to be within the specified range.

    Args:
        angle: The angle to normalize (in deg).
        range: A tuple specifying the desired range (min, max). 
               Defaults to [-180, 180].

    Returns:
        The normalized angle (in deg).
    """
    min_range, max_range = range
    width = max_range - min_range
    
    normalized_angle = angle
    while normalized_angle >= max_range:
        normalized_angle -= width
    while normalized_angle < min_range:
        normalized_angle += width
    
    return normalized_angle

def symbolic_sol(desired_pos):
    X = desired_pos[0]
    Y = desired_pos[1]
    Z = desired_pos[2]

    l1 = 61.8
    l2 = 100.
    l3 = 52.5

    t1 = 0
    t2 = 0
    t3 = 0

    joints = np.array([t1, t2, t3])
    return joints

def numerical_sol(desired_pos):
    # inspired by ChatGPT: https://chat.openai.com/share/88b7d977-e666-4705-b4cd-42edc23a91c3

    initial_guess = np.array([theta1, theta2, theta3]) * deg2rad
    res = minimize(objective_function, initial_guess, args=(desired_pos), method='SLSQP') # in terms of SLSQP works well, however BFGS does not work. You can also test other methods, they may have different performance, especially 
    if res.success:
        return [normalize_angle(angle) for angle in res.x * rad2deg]
    else:
        print("Inverse kinematics did not converge")
        return initial_guess * rad2deg


def objective_function(joints, desired_pos):
    """
    The objective function that calculates the error between the current end-effector
    position and the desired position.
    """
    _, _, _, _, _, _, ee_loc = compute_transformation_matrices(joints[0], joints[1], joints[2], 0)

    # Calculate error
    error = np.linalg.norm(np.array(ee_loc).astype(np.float64).flatten() - np.array(desired_pos))
    return error

def inverse_kinematics(symbolic, desired_pos):
    if symbolic:
        joints = symbolic_sol(desired_pos)
    else:
        joints = numerical_sol(desired_pos)
    return joints

global last_key

# Initialize robot controller.
RC = robot_controller()
RC.communication_begin()
RC.joints_homing()

# Start keyboard listener
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Set movement parameters.
keyboard_increment = 1
goals = np.zeros(RC.joint_num)
speeds = np.ones(RC.joint_num) * 80  # deg/s

# Initial joint angles.
theta1, theta2, theta3, theta4 = 0, 0, 0, 0
_, _, _, _, _, _, current_pos = compute_transformation_matrices(theta1,theta2,theta3,theta4)
desired_pos = current_pos
last_input_time = time.time()
last_plot_update_time = time.time()  # For managing plot update frequency

# Set up matplotlib figure.
plt.ion()
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
# Create an initial arm plot (will be redrawn each update).
arm_plot, = ax.plot([], [], [], '-o', color='k', markersize=8)

x_points = [0] * 6
y_points = [0] * 6
z_points = [0] * 6

ax.set_xlim([-200, 200])
ax.set_ylim([-200, 200])
ax.set_zlim([0, 400])
ax.set_xlabel("X-axis", fontsize=8)
ax.set_ylabel("Y-axis", fontsize=8)
ax.set_zlabel("Z-axis", fontsize=8)
ax.set_title("4DOF Robotic Arm", fontsize=10)

# Display menu.
print_menu()

try:
    while True:
        if last_key is not None:
            current_key = last_key
            last_key = None  # Reset the global key after reading
        command = False

        # Check for keyboard input.
        if current_key == '9':
            os.system('cls' if os.name == 'nt' else 'clear')
            raise SystemExit("Closing Keyboard Controller")

        if current_key == '1':
            print_no_newline(" Moving: Joint 1 +++         ")
            theta1 += keyboard_increment
            goals[0] += keyboard_increment
            command = True

        if current_key == 'q':
            print_no_newline(" Moving: Joint 1 ---         ")
            theta1 -= keyboard_increment
            goals[0] -= keyboard_increment
            command = True

        if current_key == '2':
            print_no_newline(" Moving: Joint 2 +++         ")
            theta2 += keyboard_increment
            goals[1] += keyboard_increment
            command = True

        if current_key == 'w':
            print_no_newline(" Moving: Joint 2 ---         ")
            theta2 -= keyboard_increment
            goals[1] -= keyboard_increment
            command = True

        if current_key == '3':
            print_no_newline(" Moving: Joint 3 +++         ")
            theta3 += keyboard_increment
            goals[2] += keyboard_increment
            command = True

        if current_key == 'e':
            print_no_newline(" Moving: Joint 3 ---         ")
            theta3 -= keyboard_increment
            goals[2] -= keyboard_increment
            command = True

        if current_key == '4':
            print_no_newline(" Moving: Joint 4 +++         ")
            theta4 += keyboard_increment
            goals[3] += keyboard_increment
            command = True

        if current_key == 'r':
            print_no_newline(" Moving: Joint 4 ---         ")
            theta4 -= keyboard_increment
            goals[3] -= keyboard_increment
            command = True

        if current_key == 'up':
            print_no_newline(" Moving: End Effector up ---         ")
            desired_pos[2] = current_pos[2] + keyboard_increment
            theta1, theta2, theta3 = inverse_kinematics(symbolic, desired_pos)
            goals[:2] = [theta1, theta2, theta3]
            command = True

        if current_key == 'down':
            print_no_newline(" Moving: End Effector down ---         ")
            desired_pos[2] = current_pos[2] - keyboard_increment
            theta1, theta2, theta3 = inverse_kinematics(symbolic, desired_pos)
            goals[:2] = [theta1, theta2, theta3]
            command = True

        if current_key == 'left':
            print_no_newline(" Moving: End Effector left ---         ")
            desired_pos[0] = current_pos[0] - keyboard_increment
            theta1, theta2, theta3 = inverse_kinematics(symbolic, desired_pos)
            goals[:2] = [theta1, theta2, theta3]
            command = True

        if current_key == 'right':
            print_no_newline(" Moving: End Effector right ---         ")
            desired_pos[0] = current_pos[0] + keyboard_increment
            theta1, theta2, theta3 = inverse_kinematics(symbolic, desired_pos)
            goals[:2] = [theta1, theta2, theta3]
            command = True

        if current_key == 'f':
            print_no_newline(" Moving: End Effector forward ---         ")
            desired_pos[1] = current_pos[1] + keyboard_increment
            theta1, theta2, theta3 = inverse_kinematics(symbolic, desired_pos)
            goals[:2] = [theta1, theta2, theta3]
            command = True

        if current_key == 'b':
            print_no_newline(" Moving: End Effector backward ---         ")
            desired_pos[1] = current_pos[1] - keyboard_increment
            theta1, theta2, theta3 = inverse_kinematics(symbolic, desired_pos)
            goals[:2] = [theta1, theta2, theta3]
            command = True

        if current_key == 'h':
            # When homing is pressed, slow down the motor.
            print("Homing robot (slow movement)...")
            homing_speed = np.ones(RC.joint_num) * 5  # Slower speed for homing (deg/s)
            RC.joints_homing()  # Command homing movement.
            time.sleep(1.0)  # Increase delay to allow slow movement.
            goals = RC.robot_homing_joint_poses.copy()  # Get updated positions.
            theta1, theta2, theta3, theta4 = goals[:4]
            RC.joints_goto(goals, homing_speed)  # Send homing command with slower speeds.
            command = True

        if current_key == '5':
            print_no_newline(" Grasper Open....                  ")
            RC.gripper_set_percentage(0)

        if current_key == 't':
            print_no_newline(" Grasper Close....                  ")
            RC.gripper_set_percentage(100)

        # If a command was given, send it to the robot and update last input time.
        if command:
            goals = np.clip(goals, RC.servo_angle_min, RC.servo_angle_max)
            RC.joints_goto(goals, speeds)
            last_input_time = time.time()
            # Update plot every 0.01 seconds when there is input.
            if time.time() - last_plot_update_time >= 0.01:
                update_plot()
                _, _, _, _, _, _, current_pos = compute_transformation_matrices(theta1, theta2, theta3, theta4)
                desired_pos = current_pos
                last_plot_update_time = time.time()
        # When no command is given, update the plot after a short delay.
        elif time.time() - last_input_time > 0.01:
            if time.time() - last_plot_update_time >= 0.01:
                update_plot()
                _, _, _, _, _, _, current_pos = compute_transformation_matrices(theta1, theta2, theta3, theta4)
                desired_pos = current_pos
                last_plot_update_time = time.time()

        time.sleep(0.0001)  # Small delay to reduce CPU usage.

except SystemExit:
    plt.ioff()
finally:
    listener.stop()
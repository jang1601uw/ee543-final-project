import time
import numpy as np
import keyboard
import sys
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_controller import robot_controller
from scipy.optimize import minimize   # Needed for the IK optimization

# Disable the default quit key ('q') in matplotlib so that pressing it won't close the plot.
plt.rcParams['keymap.quit'] = ''

# Global variable to store status message.
status_str = ""

# Conversion constants and helper function for angle normalization.
d2r = 2 * np.pi / 360.0
deg2rad = d2r
rad2deg = 1 / d2r

def normalize_angle(angle, range=(-180, 180)):
    """
    Normalizes an angle to be within the specified range.
    
    Args:
        angle: The angle to normalize (in deg).
        range: Tuple (min, max). Defaults to [-180, 180].
    
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

def link_n(alpha, a, d, theta):
    """
    Compute transformation matrix using DH parameters.
    
    Args:
        alpha, a, d, theta: DH parameters (theta in degrees).
    
    Returns:
        A 4x4 transformation matrix.
    """
    theta_n = np.radians(theta)
    alpha_n = np.radians(alpha)
    T = np.array([
        [np.cos(theta_n), -np.sin(theta_n), 0, a],
        [np.sin(theta_n)*np.cos(alpha_n), np.cos(theta_n)*np.cos(alpha_n), -np.sin(alpha_n), -np.sin(alpha_n)*d],
        [np.sin(theta_n)*np.sin(alpha_n), np.cos(theta_n)*np.sin(alpha_n),  np.cos(alpha_n),  np.cos(alpha_n)*d],
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

    return T0, T1, T2, T3, T4, T_end


def get_dh_params(joints):
    """
    Given the current joint angles (in deg), return the DH parameter table.
    
    The table is defined as:
      Row 1: [0,    0,     0,          theta1 + 180]
      Row 2: [0,    0,     61.8,       0]
      Row 3: [90,   0,     0,          theta2 + 90]
      Row 4: [0,    100,   0,          theta3 + 90]
      Row 5: [90,   0,     52.5,       theta4 + 0]
      Row 6: [0,    0,     184.39,     0]
    
    Args:
        joints: List [theta1, theta2, theta3, theta4] in degrees.
    
    Returns:
        A list of DH parameter rows.
    """
    theta1, theta2, theta3, theta4 = joints
    return [
        [0,    0,     0,          theta1 + 180],
        [0,    0,     61.8,       0],
        [90,   0,     0,          theta2 + 90],
        [0,    100,   0,          theta3 + 90],
        [90,   0,     52.5,       theta4 + 0],
        [0,    0,     184.39,     0]
    ]

def forward_kinematics_from_dh(joints):
    """
    Compute the end-effector transformation matrix given joint angles.
    This function uses the DH table returned by get_dh_params().
    
    Args:
        joints: List [theta1, theta2, theta3, theta4] in degrees.
    
    Returns:
        A 4x4 transformation matrix representing the end-effector pose.
    """
    dh_params = get_dh_params(joints)
    T = np.eye(4)
    for params in dh_params:
        alpha, a, d, theta = params
        T = T @ link_n(alpha, a, d, theta)
    return T

def objective_function(joints_rad, desired_pos, dh_params):
    """
    Compute the error between the computed end-effector position and desired_pos.
    
    Args:
        joints_rad: Joint angles in radians.
        desired_pos: Desired [X, Y, Z] position.
        dh_params: A function that returns the DH table given joint angles (in deg).
    
    Returns:
        Euclidean error.
    """
    joints_deg = joints_rad * rad2deg
    T_end = forward_kinematics_from_dh(joints_deg)
    current_pos = T_end[:3, 3]
    return np.linalg.norm(current_pos - np.array(desired_pos))

def inverse_kinematics(dh_params, desired_pos, initial_guess):
    """
    Solve the inverse kinematics problem using numerical minimization.
    
    Args:
        dh_params: A function that returns the DH parameter table given joint angles.
        desired_pos: Desired end-effector position [X, Y, Z].
        initial_guess: Initial guess for joint angles [theta1, theta2, theta3, theta4] in degrees.
    
    Returns:
        A list of joint angles (in degrees) that minimize the error.
    """
    initial_guess = np.array(initial_guess) * deg2rad
    res = minimize(objective_function, initial_guess, args=(desired_pos, dh_params), method='SLSQP')
    if res.success:
        return [normalize_angle(angle) for angle in res.x * rad2deg]
    else:
        print("Inverse kinematics did not converge")
        return initial_guess * rad2deg

# ---------------------------------------------------------------------------
# Plotting and Control Functions
# ---------------------------------------------------------------------------
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

    # Compute the chain of transformations from the current joint angles.
    T0, T1, T2, T3, T4, T_end = compute_transformation_matrices(theta1, theta2, theta3, theta4)

    # Define points for the arm.
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

    # Display end-effector position and joint angles at the top left.
    status_text = (f"End Position: X={T_end[0, 3]:.2f}, Y={T_end[1, 3]:.2f}, Z={T_end[2, 3]:.2f}\n"
                   f"Angles: θ1={theta1:.2f}, θ2={theta2:.2f}, θ3={theta3:.2f}, θ4={theta4:.2f}")
    ax.text2D(0.01, 0.99, status_text, transform=ax.transAxes, fontsize=8,
              verticalalignment='top', horizontalalignment='left', bbox=dict(facecolor='white', alpha=0.5))
    # Display the current status message in the top right.
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
    print('[Grasper Open]: 5 | [Grasper Close]: t')
    print('[Homing]: h')
    print('[Inverse Kinematics]: i')
    print('-----------------------------------------')

def print_no_newline(string):
    global status_str
    status_str = string  # Update the global status string.
    print("\r" + string, end="", flush=True)

# Initialize robot controller.
RC = robot_controller()
RC.communication_begin()
RC.joints_homing()

# Set movement parameters.
keyboard_increment = 1
goals = np.zeros(RC.joint_num)
speeds = np.ones(RC.joint_num) * 120 # deg/s

# Initial joint angles.
theta1, theta2, theta3, theta4 = 0, 0, 0, 0
last_input_time = time.time()
last_plot_update_time = time.time()  # For managing plot update frequency

# Flag to prevent multiple IK triggers per key press.
ik_triggered = False

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
        command = False

        # Check for keyboard input.
        if keyboard.is_pressed('9'):
            os.system('cls' if os.name == 'nt' else 'clear')
            raise SystemExit("Closing Keyboard Controller")

        if keyboard.is_pressed('1'):
            print_no_newline(" Moving: Joint 1 +++         ")
            theta1 += keyboard_increment
            goals[0] += keyboard_increment
            command = True

        if keyboard.is_pressed('q'):
            print_no_newline(" Moving: Joint 1 ---         ")
            theta1 -= keyboard_increment
            goals[0] -= keyboard_increment
            command = True

        if keyboard.is_pressed('2'):
            print_no_newline(" Moving: Joint 2 +++         ")
            theta2 += keyboard_increment
            goals[1] += keyboard_increment
            command = True

        if keyboard.is_pressed('w'):
            print_no_newline(" Moving: Joint 2 ---         ")
            theta2 -= keyboard_increment
            goals[1] -= keyboard_increment
            command = True

        if keyboard.is_pressed('3'):
            print_no_newline(" Moving: Joint 3 +++         ")
            theta3 += keyboard_increment
            goals[2] += keyboard_increment
            command = True

        if keyboard.is_pressed('e'):
            print_no_newline(" Moving: Joint 3 ---         ")
            theta3 -= keyboard_increment
            goals[2] -= keyboard_increment
            command = True

        if keyboard.is_pressed('4'):
            print_no_newline(" Moving: Joint 4 +++         ")
            theta4 += keyboard_increment
            goals[3] += keyboard_increment
            command = True

        if keyboard.is_pressed('r'):
            print_no_newline(" Moving: Joint 4 ---         ")
            theta4 -= keyboard_increment
            goals[3] -= keyboard_increment
            command = True

        if keyboard.is_pressed('h'):
            # When homing is pressed, slow down the motor.
            print("Homing robot (slow movement)...")
            homing_speed = np.ones(RC.joint_num) * 1  # Slower speed for homing (deg/s)
            RC.joints_homing()  # Command homing movement.
            goals = RC.robot_homing_joint_poses.copy()  # Get updated positions.
            theta1, theta2, theta3, theta4 = goals[:4]
            RC.joints_goto(goals, homing_speed)  # Send homing command with slower speeds.
            command = True

        if keyboard.is_pressed('5'):
            print_no_newline(" Grasper Open....                  ")
            RC.gripper_set_percentage(0)

        if keyboard.is_pressed('t'):
            print_no_newline(" Grasper Close....                  ")
            RC.gripper_set_percentage(100)

        # New keyboard command for inverse kinematics.
        if keyboard.is_pressed('i') and not ik_triggered:
            ik_triggered = True
            print_no_newline(" Performing Inverse Kinematics...  ")
            # Define a desired end-effector position (adjust as needed).
            desired_pos = [217, 0, 132]
            # Use current joint angles as the initial guess.
            initial_guess = [theta1, theta2, theta3, theta4]
            new_angles = inverse_kinematics(get_dh_params, desired_pos, initial_guess)
            print("IK result:", new_angles)  # Debug: show computed angles
            # Update joint variables and goals with the new angles.
            theta1, theta2, theta3, theta4 = new_angles
            goals[0:4] = new_angles[0:4]
            command = True

        # Reset the IK trigger when the key is released.
        if not keyboard.is_pressed('i'):
            ik_triggered = False

        # If a command was given, send it to the robot and update last input time.
        if command:
            goals = np.clip(goals, RC.servo_angle_min, RC.servo_angle_max)
            RC.joints_goto(goals, speeds)
            last_input_time = time.time()
            # Update plot every 0.01 seconds when there is input.
            if time.time() - last_plot_update_time >= 0.01:
                update_plot()
                last_plot_update_time = time.time()
        # When no command is given, update the plot after a short delay.
        elif time.time() - last_input_time > 0.01:
            if time.time() - last_plot_update_time >= 0.01:
                update_plot()
                last_plot_update_time = time.time()

        time.sleep(0.0001)  # Small delay to reduce CPU usage.

except SystemExit:
    plt.ioff()

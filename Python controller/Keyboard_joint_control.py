"""""""""""""""""""""""""""""

University of Washington, 2024

Author: Tin Chiang

Note: Modified code from Haonan Peng's Raven keyboard controller

Original code: https://github.dev/uw-biorobotics/raven2_CRTK_Python_controller/blob/main/python_controller/run_r2_keyboard_controller.py
"""""""""""""""""""""""""""""

import time
import numpy as np
import keyboard
import sys, os
from robot_controller import robot_controller

import matplotlib.pyplot as plt


def print_manu():
    print('  ')
    print('-----------------------------------------')
    print('EE543 Arm Keyboard Controller:')
    print('-----------------------------------------')
    print('[Exit]: 9')
    print('[Joint 1    +]: 1 | [Joint 1     -]: q')
    print('[Joint 2    +]: 2 | [Joint 2     -]: w')
    print('[Joint 3    +]: 3 | [Joint 3     -]: e')
    print('[Joint 4    +]: 4 | [Joint 4     -]: r')
    print('[Grasper Open]: 5 | [Grasper Close]: t')

    print('-----------------------------------------')
    print('-----------------------------------------')
    print('Current command:\n')
    return None

def print_no_newline(string):
    sys.stdout.write("\r" + string)
    sys.stdout.flush()
    return None

def init_visualization():
    # Create a 3D plot for each joint position
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set axis limits
    ax.set_xlim([-400, 400])
    ax.set_ylim([-400, 400])

    # Labels and viewing angle
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("3D Plot of 4DOF Robotic Arm with Each Joint Position")
    ax.legend()
    ax.view_init(elev=20, azim=30)


# init the Robot Controller
RC = robot_controller()
RC.communication_begin()

# Force homing the robot
RC.joints_homing()

keyboard_increment = 0.5

goals = np.zeros(RC.joint_num)
speeds = np.ones(RC.joint_num) * 80 # deg/s

x = 0
working = 1
command = False
mod_dh_sim = True

print_manu()

if mod_dh_sim:
    init_visualization()

while working==1:

    #get the keyboard input
    input_key = keyboard.read_event().name

    if input_key == '9':
        # RC.communication_end()
        os.system('cls' if os.name == 'nt' else 'clear')
        sys.exit('Closing Keyboard controller')
        

    elif input_key == '1':
        print_no_newline(" Moving: Joint 1 +++         ")
        goals[0] += keyboard_increment
        command = True


    elif input_key == 'q':
        print_no_newline(" Moving: Joint 1 ---         ")
        goals[0] -= keyboard_increment
        command = True

    elif input_key == '2':
        print_no_newline(" Moving: Joint 2 +++         ")
        goals[1] += keyboard_increment
        command = True
              
    elif input_key == 'w':
        print_no_newline(" Moving: Joint 2 ---         ")
        goals[1] -= keyboard_increment
        command = True

    elif input_key == '3':
        print_no_newline(" Moving: Joint 3 +++         ")
        goals[2] += keyboard_increment
        command = True
        
    elif input_key == 'e':
        print_no_newline(" Moving: Joint 3 ---         ")
        goals[2] -= keyboard_increment
        command = True

    elif input_key == '4':
        print_no_newline(" Moving: Joint 4 +++         ")
        goals[3] += keyboard_increment
        command = True
        
    elif input_key == 'r':
        print_no_newline(" Moving: Joint 4 ---         ")
        goals[3] -= keyboard_increment
        command = True
    
    elif input_key == 'h':
        print_no_newline(" Homing....                  ")
        goals = RC.robot_homing_joint_poses.copy()
        command = True
        
    elif input_key == '5':
        print_no_newline(" Grasper Open....                  ")
        RC.gripper_set_percentage(0)


    elif input_key == 't':
        print_no_newline(" Grasper Close....                  ")
        RC.gripper_set_percentage(100)


    else:
        print_no_newline(' Unknown command             ')

    
    if command:
        # make sure the goals is within joint limit
        goals = np.clip(goals, RC.servo_angle_min, RC.servo_angle_max) 
        sys.stdout.write("\033[1B") # move curser down
        RC.joints_goto(goals, speeds)
        sys.stdout.write("\033[1A") # move curser up
        command = False
    
    if mod_dh_sim:
        # the transformation matrices from first to last link 
        T1, T2, T3, T4, T_end = RC.update_forward_kinematics(*goals[:4])
        
        # Extract joint positions
        base_position = np.array([0, 0, 0])
        T1_position = T1[0:3, 3]
        T2_position = T2[0:3, 3]
        T3_position = T3[0:3, 3]
        T4_position = T4[0:3, 3]
        end_effector_position = T_end[0:3, 3]

        ax = RC.update_plot(ax,base_position,T1_position,T2_position,T3_position,T4_position,end_effector_position)


    

    


    
    


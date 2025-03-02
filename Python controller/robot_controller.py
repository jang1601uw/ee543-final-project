import time
import numpy as np
import serial
import sys
import matplotlib.pyplot as plt
from Keyboard_joint_control import goals

np.set_printoptions(precision=2, suppress=False)
np.set_printoptions(formatter={'all': lambda x: f'{x:.2f}'})

d2r = 2*np.pi/360.0
deg2rad = d2r

class robot_controller():
    def __init__(self) -> None:
        #define robot parameter
        self.joint_num = 4
        self.joints_goto_tolerance = 10e-3

        #define robot state
        self.robotstate_joint_poses = np.zeros(self.joint_num)
        self.robotstate_joint_vels = np.zeros(self.joint_num)
        self.robotState_endeffector_orientation = np.zeros(3)
        self.robotstate_endeffector_pose = np.zeros(3)
        self.robotstate_gripper_close = False

        #define homing position in joint space
        self.robot_homing_joint_poses = np.zeros(self.joint_num)

        """
        ---------------------------------------------------------------
         Below are the parameters related to robot link geometry
        ---------------------------------------------------------------
        """

        #define the DH parameter for the arm link (check out the robot drawings for the dimensions)
        a1, a2, a3, a4, a5 = 0, 0, 100, 0, 0
        alpha1, alpha2, alpha3, alpha4, alpha5 = 0, 90, 0, 90, 0
        d1, d2, d3, d4, d5 = 61.8, 0, 0, 0, 236.89
        th1, th2, th3, th4, th5 = goals[0], goals[1], goals[2], goals[3], 0

        # [a, alpha, d, theta (will be replaced by joint_positions)]
        self.dh_params = [ # this is for 4 joints setting
            [alpha1, a1, d1, th1],  # Joint 1
            [alpha2, a2, d2, th2],  # Joint 2
            [alpha3, a3, d3, th3],  # Joint 3
            [alpha4, a4, d4, th4],  # Joint 4
            [alpha5, a5, d5, th5]   # Joint 5
        ]

        self.angle_offsets = np.array([180, 90, 90, 0]) # this is for 4 joints setting

        # the transformation matrices from first to last link 
        self.T_matrices = np.empty(self.joint_num) # no value when init

        #define the base frame
        self.base_frame = np.eye(self.joint_num)

        """
        ---------------------------------------------------------------
         Below are the parameters related to hardware and communciation
        ---------------------------------------------------------------
        """

        #here define the specification for MG996R servo motors
        self.servo_angle_max = 90 #degree
        self.servo_angle_min = -90 #degree
        self.servo_pulse_max = 440 #+90 for mg996R, This is the 'maximum' pulse length count (out of 4096)
        self.servo_pulse_min = 70 #-90 for mg996R, This is the 'minimum' pulse length count (out of 4096)

        # Here define the operating parameters for sliding gripper
        # Slider gripper is also controlled by an MG996R servo motor
        self.gripper_open_angle = 0 # degree
        self.gripper_close_angle = -90 # degree


        #define the serial communication parameter
        self.com_port = 'COM3' # change it if needed
        self.com_baudrate = 115200 #bps
        self.com_frequency = 30 #Hz
        

    """
    ---------------------------------------------------------------
     Functions below set up the serial communication
    ---------------------------------------------------------------
    """

    def communication_begin(self):
        self.ser = serial.Serial(self.com_port, self.com_baudrate)
        # Reset input/output buffer and wait for initialization
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(1)

        # Wait for Arduino to initialize
        while True:
            if self.ser.read() == b'I':
                break

        # Send signaling byte
        self.ser.write(b'S')
        time.sleep(0.1)
    
    def communication_end(self):
        self.ser.close()
    

    """
    ---------------------------------------------------------------
     Functions below set up the visualization
    ---------------------------------------------------------------
    """
    def virtual_link_visulaization(self):
        base_position = np.array([0, 0, 0])
        T1_position = self.update_forward_kinematics[0][0:3, 3]
        T2_position = self.update_forward_kinematics[1][0:3, 3]
        T3_position = self.update_forward_kinematics[2][0:3, 3]
        T4_position = self.update_forward_kinematics[3][0:3, 3]
        end_effector_position = self.update_forward_kinematics[4][0:3, 3]

        # Create a 3D plot for each joint position
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Define points for plotting
        x_points = [base_position[0], T1_position[0], T2_position[0], T3_position[0], T4_position[0], end_effector_position[0]]
        y_points = [base_position[1], T1_position[1], T2_position[1], T3_position[1], T4_position[1], end_effector_position[1]]
        z_points = [base_position[2], T1_position[2], T2_position[2], T3_position[2], T4_position[2], end_effector_position[2]]

        # Plot each joint position with different markers
        ax.scatter(base_position[0], base_position[1], base_position[2], color='r', marker='o', s=100, label='Base')
        ax.scatter(T1_position[0], T1_position[1], T1_position[2], color='g', marker='o', s=100, label='Joint 1')
        ax.scatter(T2_position[0], T2_position[1], T2_position[2], color='b', marker='o', s=100, label='Joint 2')
        ax.scatter(T3_position[0], T3_position[1], T3_position[2], color='y', marker='o', s=100, label='Joint 3')
        ax.scatter(T4_position[0], T4_position[1], T4_position[2], color='c', marker='o', s=100, label='Joint 4')
        ax.scatter(end_effector_position[0], end_effector_position[1], end_effector_position[2], color='m', marker='o', s=100, label='End Effector')

        # Connect the joints with lines
        ax.plot(x_points, y_points, z_points, linestyle='-', color='k')

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

        # Show plot
        plt.show()

        return None
    
    

    """
    ---------------------------------------------------------------
     Functions below setup the transformation matrix for 
     forward kinematics
    ---------------------------------------------------------------
    """

    # input: DH parameters of a specific link, angle in degree, length in mm
    # output: the transformation matrix of that link
    def dh_to_transformation_matrix(self, alpha, a, d, theta):
        # convert angles from degrees to rad
        alpha = alpha*deg2rad
        theta = theta*deg2rad
        
        # cos/sin for a and th
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = np.cos(theta)
        st = np.sin(theta)

        T = np.array([[ct,    -st,   0,   a],
                    [st*ca, ct*ca, -sa, -sa*d],
                    [st*sa, ct*sa, ca,  ca*d],
                    [0,     0,     0,   1]])
        
        return T
    
    def update_forward_kinematics(self):
        T0_1 = self.dh_to_transformation_matrix(self, self.dh_params[0,0], self.dh_params[0,1], self.dh_to_transformation_matrix[0,2], self.dh_to_transformation_matrix[0,3])
        T1_2 = self.dh_to_transformation_matrix(self, self.dh_params[1,0], self.dh_params[1,1], self.dh_to_transformation_matrix[1,2], self.dh_to_transformation_matrix[1,3])
        T2_3 = self.dh_to_transformation_matrix(self, self.dh_params[2,0], self.dh_params[2,1], self.dh_to_transformation_matrix[2,2], self.dh_to_transformation_matrix[2,3])
        T3_4 = self.dh_to_transformation_matrix(self, self.dh_params[3,0], self.dh_params[3,1], self.dh_to_transformation_matrix[3,2], self.dh_to_transformation_matrix[3,3])
        T4_e = self.dh_to_transformation_matrix(self, self.dh_params[4,0], self.dh_params[4,1], self.dh_to_transformation_matrix[4,2], self.dh_to_transformation_matrix[4,3])
        
        T0_e = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_e

        pose_end_effector = T0_e @ self.base_frame

        return T0_1, T1_2, T2_3, T3_4, T4_e, pose_end_effector



    """
    ---------------------------------------------------------------
     Functions below set up the visualization
    ---------------------------------------------------------------
    """
    def virtual_link_visulaization(self):
        base_position = np.array([0, 0, 0])
        T1_position = self.update_forward_kinematics[0][0:3, 3]
        T2_position = self.update_forward_kinematics[1][0:3, 3]
        T3_position = self.update_forward_kinematics[2][0:3, 3]
        T4_position = self.update_forward_kinematics[3][0:3, 3]
        end_effector_position = self.update_forward_kinematics[4][0:3, 3]

        # Create a 3D plot for each joint position
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Define points for plotting
        x_points = [base_position[0], T1_position[0], T2_position[0], T3_position[0], T4_position[0], end_effector_position[0]]
        y_points = [base_position[1], T1_position[1], T2_position[1], T3_position[1], T4_position[1], end_effector_position[1]]
        z_points = [base_position[2], T1_position[2], T2_position[2], T3_position[2], T4_position[2], end_effector_position[2]]

        # Plot each joint position with different markers
        ax.scatter(base_position[0], base_position[1], base_position[2], color='r', marker='o', s=100, label='Base')
        ax.scatter(T1_position[0], T1_position[1], T1_position[2], color='g', marker='o', s=100, label='Joint 1')
        ax.scatter(T2_position[0], T2_position[1], T2_position[2], color='b', marker='o', s=100, label='Joint 2')
        ax.scatter(T3_position[0], T3_position[1], T3_position[2], color='y', marker='o', s=100, label='Joint 3')
        ax.scatter(T4_position[0], T4_position[1], T4_position[2], color='c', marker='o', s=100, label='Joint 4')
        ax.scatter(end_effector_position[0], end_effector_position[1], end_effector_position[2], color='m', marker='o', s=100, label='End Effector')

        # Connect the joints with lines
        ax.plot(x_points, y_points, z_points, linestyle='-', color='k')

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

        # Show plot
        plt.show()

        return None



    """
    ---------------------------------------------------------------
     Functions below convert the joint command into proper form for
     serial communication
    ---------------------------------------------------------------
    """
    # convert the multiple joint poses in angle into pulse lengths array
    # map the angle from -90 to 90 degree to minimal till maximal servo pulse length
    def angle_to_pulse_length(self, angles):
        clipped_angles = np.clip(angles, self.servo_angle_min, self.servo_angle_max)
        pulse_lengths = ((clipped_angles - self.servo_angle_min) * (self.servo_pulse_max - self.servo_pulse_min) / (self.servo_angle_max - self.servo_angle_min) + self.servo_pulse_min).astype(int)
        return pulse_lengths

    # convert the multiple joint poses in pulse lengths into 8 bytes array
    # format will be JP1_H, JP1_L, ..., unit: length count
    def pulse_length_to_byte(self, pulse_lengths):
        # clipped_pulse_lengths = (list)(np.clip(pulse_lengths, self.servo_pulse_min, self.servo_pulse_max))
        clipped_pulse_lengths = (list)(pulse_lengths)
        ret = []
        for pulse_length in clipped_pulse_lengths:
            # convert the number into high and low bytes
            # pulse_length = (int)pulse_length
            pulse_length_byte = int(pulse_length).to_bytes(2, byteorder='big')
            ret.append(pulse_length_byte[0])
            ret.append(pulse_length_byte[1])
        return ret
    
    
    # Set the joint to the homing position
    # Cautious: The robot will move rapidly if this is executed
    def joints_homing(self):
        # reset robot state
        self.robotstate_joint_poses = self.robot_homing_joint_poses.copy()
        self.robotstate_gripper_close = False

        # compose command
        joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)
        joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_open)
        # print(joint_pulse_lengthes)
        numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
        # print(numbers)
        # Poll for acknowledgement
        while self.ser.in_waiting == 0:
            continue
        # ser.reset_input_buffer()

        # # Send data if acknowledgement received
        if self.ser.read() == b'A':
            self.ser.write(numbers)
            self.ser.flush()


    
    # this is the goto function in joint space
    # input is the array of joint poses(in degree) and the arry of joint velocities(degree/s)  
    def joints_goto(self, goals, speeds):
        # get the current robot joint poses
        start_poses = self.robotstate_joint_poses.copy()
        # print("Start Poses: ", start_poses)
        # calculate the rotation direction of each joints
        angle_diff = goals - start_poses
        # print("angle difference: ", angle_diff)
        # calculate the angle increments under 20Hz update rates
        angle_increments = np.sign(angle_diff) * (speeds / self.com_frequency)
        

        reached_goal = False        
        # update the robot joint poses by adding the angle increments
        while not reached_goal:
            start = time.time()
            # print("Start Poses: ", start_poses)
            # print("angle difference: ", angle_diff)

            # Generate 8 uint8_t numbers
            self.robotstate_joint_poses += angle_increments
            # check if the individual joint reach the goal
            for i in range(self.joint_num):
                if goals[i] > start_poses[i]: # the angle is increasing
                    self.robotstate_joint_poses[i] = np.clip(self.robotstate_joint_poses[i], start_poses[i], goals[i])
                elif goals[i] < start_poses[i]: # the angle is decreasing
                    self.robotstate_joint_poses[i] = np.clip(self.robotstate_joint_poses[i], goals[i], start_poses[i])
                else:
                    self.robotstate_joint_poses[i] = start_poses[i].copy()
            # print("Robotstate: ",self.robotstate_joint_poses)
            # Set the desired print options
            sys.stdout.write('\r' + ' ' * 50 + '\r') # clear the line
            sys.stdout.write("\r" + "Robotstate: " + str(self.robotstate_joint_poses))
            sys.stdout.flush()    
            
            #check if the robot reach the goal joint poses
            if np.all(np.abs(self.robotstate_joint_poses - goals) <= self.joints_goto_tolerance):
                reached_goal = True

            #convert the joint_pose to pulse length
            joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)

            #add one more byte in the pulse length array to as gripper command
            if self.robotstate_gripper_close:
                joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_close)
            else:
                joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_open)
            # print(joint_pulse_lengthes)
            numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
            # print(numbers)   

            # Poll for acknowledgement
            while self.ser.in_waiting == 0:
                continue

            # Send data if acknowledgement received
            if self.ser.read() == b'A':
                self.ser.write(numbers)
                self.ser.flush()
                dur = time.time() - start
                time.sleep(np.clip((1/self.com_frequency)-dur-0.005, 0, (1/self.com_frequency)))#50Hz

        # The function below control the end effector using the servo motor position
    # 0 degree means the gripper is fully opened
    # -90 degree menas the gripper is fully closed
    def gripper_set_angle(self, angle):
        # Clip the angle within operating range
        angle = np.clip(angle, self.gripper_close_angle, self.gripper_open_angle)

        # Convert angle to pulse length
        pulse_length = self.angle_to_pulse_length(angle)
        
        # Store gripper angle
        self.robotstate_gripper_angle = angle

        # Send the pulse length command to the gripper
        joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)

        # Add gripper angle to the pulse length array
        joint_pulse_lengthes = np.append(joint_pulse_lengthes, pulse_length)

        # Convert to bytes and send
        numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
        
        # Poll for acknowledgment
        while self.ser.in_waiting == 0:
            continue

        # Send data if acknowledgment received
        if self.ser.read() == b'A':
            self.ser.write(numbers)
            self.ser.flush()

    # The function below control the end effector using the percentage
    # 0% means the gripper is fully opened
    # 100% menas the gripper is fully closed
    def gripper_set_percentage(self, percentage):
        percentage = np.clip(percentage, 0, 100)

        # convert the percentage into angle
        angle = percentage * (self.gripper_close_angle - self.gripper_open_angle) / 100

        self.gripper_set_angle(angle)
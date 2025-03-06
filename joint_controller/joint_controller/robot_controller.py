import time
import numpy as np
import serial
import sys

np.set_printoptions(precision=2, suppress=False)
np.set_printoptions(formatter={'all': lambda x: f'{x:.2f}'})

class robot_controller():
    def __init__(self) -> None:
        # Define robot parameters
        self.joint_num = 4
        self.joints_goto_tolerance = 10e-3

        # Define robot state
        self.robotstate_joint_poses = np.zeros(self.joint_num)
        self.robotstate_joint_vels = np.zeros(self.joint_num)
        self.robotState_endeffector_orientation = np.zeros(3)
        self.robotstate_endeffector_pose = np.zeros(3)
        self.robotstate_gripper_close = False

        # Define homing position in joint space
        self.robot_homing_joint_poses = np.zeros(self.joint_num)

        # Define the DH parameters for the arm link
        self.dh_params = [  # This is for a 4-joint setting
            [0, 0, 0, 0],  # Joint 1
            [0, 0, 0, 0],  # Joint 2
            [0, 0, 0, 0],  # Joint 3
            [0, 0, 0, 0]   # Joint 4
        ]
        self.angle_offsets = np.array([0, 0, 0, 0])
        self.T_matrices = np.empty(self.joint_num)  # Transformation matrices
        self.base_frame = np.eye(self.joint_num)  # Base frame

        # Define servo motor specifications
        self.servo_angle_max = 90  # Degrees
        self.servo_angle_min = -90  # Degrees
        self.servo_pulse_max = 440  # +90 degrees for MG996R
        self.servo_pulse_min = 70   # -90 degrees for MG996R

        # Define gripper control parameters
        self.gripper_open_angle = 0  # Degrees
        self.gripper_close_angle = -90  # Degrees

        # Compute gripper pulse lengths
        self.gripper_pulse_open = self.angle_to_pulse_length(self.gripper_open_angle)
        self.gripper_pulse_close = self.angle_to_pulse_length(self.gripper_close_angle)

        # Define serial communication parameters
        self.com_port = 'COM3'  # Change if needed
        self.com_baudrate = 115200  # bps
        self.com_frequency = 30  # Hz

    def communication_begin(self):
        self.ser = serial.Serial(self.com_port, self.com_baudrate)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(1)
        while True:
            if self.ser.read() == b'I':
                break
        self.ser.write(b'S')
        time.sleep(0.1)
    
    def communication_end(self):
        self.ser.close()
    
    def angle_to_pulse_length(self, angles):
        clipped_angles = np.clip(angles, self.servo_angle_min, self.servo_angle_max)
        pulse_lengths = ((clipped_angles - self.servo_angle_min) * (self.servo_pulse_max - self.servo_pulse_min) /
                         (self.servo_angle_max - self.servo_angle_min) + self.servo_pulse_min).astype(int)
        return pulse_lengths

    def pulse_length_to_byte(self, pulse_lengths):
        ret = []
        for pulse_length in pulse_lengths:
            pulse_length_byte = int(pulse_length).to_bytes(2, byteorder='big')
            ret.append(pulse_length_byte[0])
            ret.append(pulse_length_byte[1])
        return ret
    
    def joints_homing(self):
        self.robotstate_joint_poses = self.robot_homing_joint_poses.copy()
        self.robotstate_gripper_close = False
        joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)
        joint_pulse_lengthes = np.append(joint_pulse_lengthes, self.gripper_pulse_open)
        numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
        while self.ser.in_waiting == 0:
            continue
        if self.ser.read() == b'A':
            self.ser.write(numbers)
            self.ser.flush()
    
    def joints_goto(self, goals, speeds):
        start_poses = self.robotstate_joint_poses.copy()
        angle_diff = goals - start_poses
        angle_increments = np.sign(angle_diff) * (speeds / self.com_frequency)
        reached_goal = False        
        while not reached_goal:
            start = time.time()
            self.robotstate_joint_poses += angle_increments
            for i in range(self.joint_num):
                if goals[i] > start_poses[i]:
                    self.robotstate_joint_poses[i] = np.clip(self.robotstate_joint_poses[i], start_poses[i], goals[i])
                elif goals[i] < start_poses[i]:
                    self.robotstate_joint_poses[i] = np.clip(self.robotstate_joint_poses[i], goals[i], start_poses[i])
            sys.stdout.write('\r' + ' ' * 50 + '\r')
            sys.stdout.write("\r" + "Robotstate: " + str(self.robotstate_joint_poses))
            sys.stdout.flush()    
            if np.all(np.abs(self.robotstate_joint_poses - goals) <= self.joints_goto_tolerance):
                reached_goal = True
            joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)
            if self.robotstate_gripper_close:
                joint_pulse_lengthes = np.append(joint_pulse_lengthes, self.gripper_pulse_close)
            else:
                joint_pulse_lengthes = np.append(joint_pulse_lengthes, self.gripper_pulse_open)
            numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
            while self.ser.in_waiting == 0:
                continue
            if self.ser.read() == b'A':
                self.ser.write(numbers)
                self.ser.flush()
                dur = time.time() - start
                time.sleep(np.clip((1/self.com_frequency)-dur-0.005, 0, (1/self.com_frequency)))
    
    def gripper_set_angle(self, angle):
        angle = np.clip(angle, self.gripper_close_angle, self.gripper_open_angle)
        pulse_length = self.angle_to_pulse_length(angle)
        self.robotstate_gripper_angle = angle
        joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)
        joint_pulse_lengthes = np.append(joint_pulse_lengthes, pulse_length)
        numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
        while self.ser.in_waiting == 0:
            continue
        if self.ser.read() == b'A':
            self.ser.write(numbers)
            self.ser.flush()
    
    def gripper_set_percentage(self, percentage):
        percentage = np.clip(percentage, 0, 100)
        angle = percentage * (self.gripper_close_angle - self.gripper_open_angle) / 100
        self.gripper_set_angle(angle)

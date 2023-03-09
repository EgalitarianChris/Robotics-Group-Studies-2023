"""
Created on Thu Mar  2 16:59:47 2023
@author: bgh009
"""
import numpy as np
from naoqi import ALProxy, ALBroker
import time

Leg_min = 163   #min and max leg angle (not sure how accurate these are)
Leg_max = 231
Torso_min = 5    #min and max torso angle 
Torso_max = 42


def calculate_angles(A, B):  #A, B are the desired torso/leg angle respectively
    
    if A < Torso_min or A > Torso_max:
        return "Error: Torso angle out of range"
    
    if B < Leg_min or B > Leg_max:
        return "Error: Leg angle out of range"

    C = (B - Leg_min)/(Leg_max - Leg_min)

    Knee_pitch = 1.4 - 1.4905*C
    Ankle_pitch = 0.7608 - 0.8789*C

    D = (Torso_max - A) / (Torso_max - Torso_min)

    Shoulder_pitch = 1.11 + 0.34*D
    LShoulder_roll = 0.095 + 0.455*D
    RShoulder_roll = (-1)*LShoulder_roll
    LElbow_roll = 0.308 - 1.808*D
    RElbow_roll = (-1)*LElbow_roll
    LWrist_yaw = -0.713 + 0.143*D
    RWrist_yaw = (-1)*LWrist_yaw
    Hip_pitch = -0.092 - 0.645*D

    return Knee_pitch, Ankle_pitch, Shoulder_pitch, LShoulder_roll, RShoulder_roll, LElbow_roll, RElbow_roll, LWrist_yaw, RWrist_yaw, Hip_pitch

# def get_torso_angle():  #input: hip pitch, output: torso angle
#     hip_pitch = motionProxy.getAngles("LHipPitch", True)[0] # COMPLETE WITH CORRECT FUNCTION
#     torso_angle = Torso_max - ((-hip_pitch - 0.092) * (Torso_max - Torso_min)) / 0.645
#     return torso_angle

'''
Nao’s IP address is 192.168.1.2 for a wired connection and 192.168.1.3 for a wireless
connection, in both cases the port number is 9559
'''

class NAOController:
    def __init__(self, robot_ip, robot_port=9559):
        # Connect to the NAO robot
        self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)
        
        # Set joint stiffness (optional)
        self.stiffness = 1.0
        self.motion_proxy.setStiffnesses(["Body"], self.stiffness)
        
    def move_joint(self, joint_name, angle, speed):
        #angle_rad = angle * 3.14 / 180.0    #converts angle to radians if needed
        
        # Set joint angle and speed
        self.motion_proxy.setAngles(joint_name, angle, speed)
        
    def move_knee_pitch(self, angle, speed):
        self.move_joint("LKneePitch", angle, speed)
        self.move_joint("RKneePitch", angle, speed)
        
    def move_ankle_pitch(self, angle, speed):
        self.move_joint("LAnklePitch", angle, speed)
        self.move_joint("RAnklePitch", angle, speed)
        
    def move_shoulder_pitch(self, angle, speed):
        self.move_joint("LShoulderPitch", angle, speed)
        self.move_joint("RShoulderPitch", angle, speed)
        
    def move_shoulder_roll(self, angle, speed):
        self.move_joint("LShoulderRoll", angle, speed)
        self.move_joint("RShoulderRoll", -angle, speed)   #need -ve sign on some joints
        
    def move_elbow_roll(self, angle, speed):
        self.move_joint("LElbowRoll", angle, speed)
        self.move_joint("RElbowRoll", -angle, speed)
        
    def move_wrist_yaw(self, angle, speed):
        self.move_joint("LWristYaw", angle, speed)
        self.move_joint("RWristYaw", -angle, speed)
        
    def move_hip_pitch(self, angle, speed):
        self.move_joint("LHipPitch", angle, speed)
        self.move_joint("RHipPitch", angle, speed)

#define set of angles corresponding to extended/contracted position
angles1 = calculate_angles(Torso_min, Leg_min)
angles2 = calculate_angles(Torso_max, Leg_max)
angles3 = calculate_angles(10, 180)  #possible neutral position angle set

# Define speed of joint movement
speed = 0.035   #max value is 1/28.1, can take values between -1 and 1

# Instantiate NAOController class with wireless robot IP address
nao = NAOController("169.254.61.216")

'''
from pmd import pmd1208fs   #canvas code to read large encoder angle
my_pmd = pmd1208fs()  
raw = my_pmd.digin16()
raw2 = raw & 2047
angle = float(raw2)*(360.0/2048.0)
del(my_pmd)
'''
counter = 0
while counter < 20:
    angle = (-1) ** counter
    print(angle)
    time.sleep(1)
    counter += 1

# Simple continuous loop to oscillate between angles1 and angles2 based on x value
while True:
    # Set joint angles based on x value
    #will continue indefinitely
    if angle > 0: #inwards
        nao.move_knee_pitch(angles1[0], 26.8*speed)   #normalised each joint's speed
        nao.move_ankle_pitch(angles1[1], 15.7*speed)
        nao.move_shoulder_pitch(angles1[2], 4.77*speed)
        nao.move_shoulder_roll(angles1[3], -7.36*speed)  # -ve signs mean joint angle is decreasing
        nao.move_elbow_roll(angles1[5], 28.1*speed)
        nao.move_wrist_yaw(angles1[7], -speed)
        nao.move_hip_pitch(angles1[9], -11.6*speed)
    else:  #outstretched
        nao.move_knee_pitch(angles2[0], -26.8*speed)
        nao.move_ankle_pitch(angles2[1], -15.7*speed)
        nao.move_shoulder_pitch(angles2[2], -4.77*speed)
        nao.move_shoulder_roll(angles2[3], 7.36*speed)
        nao.move_elbow_roll(angles2[5], -28.1*speed)
        nao.move_wrist_yaw(angles2[7], speed)
        nao.move_hip_pitch(angles2[9], 11.6*speed)
  
def set_neutral_position():   #sets initial neutral position (returning error currently)
    nao.move_knee_pitch(angles3[0])
    nao.move_ankle_pitch(angles3[1])
    nao.move_shoulder_pitch(angles3[2])
    nao.move_shoulder_roll(angles3[3])
    nao.move_elbow_roll(angles3[5])
    nao.move_wrist_yaw(angles3[7])
    nao.move_hip_pitch(angles3[9])
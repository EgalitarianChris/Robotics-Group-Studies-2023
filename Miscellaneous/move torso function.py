# -*- coding: utf-8 -*-
"""
Created on Thu Mar  9 17:54:29 2023

@author: 44759
"""

from naoqi import ALProxy


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


class NAOController:
    def __init__(self, motion_proxy):
        self.motion_proxy = motion_proxy
        
    def move_joint(self, joint_name, angle, speed):
        self.motion_proxy.setAngles(joint_name, angle, speed)
        
    def move_shoulder_pitch(self, angle, speed):
        self.move_joint("LShoulderPitch", angle, speed)
        self.move_joint("RShoulderPitch", angle, speed)
        
    def move_shoulder_roll(self, angle, speed):
        self.move_joint("LShoulderRoll", angle, speed)
        self.move_joint("RShoulderRoll", -angle, speed)
        
    def move_elbow_roll(self, angle, speed):
        self.move_joint("LElbowRoll", angle, speed)
        self.move_joint("RElbowRoll", -angle, speed)
        
    def move_wrist_yaw(self, angle, speed):
        self.move_joint("LWristYaw", angle, speed)
        self.move_joint("RWristYaw", -angle, speed)
        
    def move_hip_pitch(self, angle, speed):
        self.move_joint("LHipPitch", angle, speed)
        self.move_joint("RHipPitch", angle, speed)

# Initialize the motion proxy outside of the NAOController class
motion_proxy = ALProxy("ALMotion", "169.254.61.216", 9559)
nao = NAOController(motion_proxy)

def move_torso(nao_controller, t_angle, t_speed):   # t_speed is between 0 and 1
    speed = t_speed / 28.1
    angles_set = calculate_angles(t_angle, Leg_min)

    nao_controller.move_shoulder_pitch(angles_set[2], 4.77*speed)
    nao_controller.move_shoulder_roll(angles_set[3], 7.36*speed)
    nao_controller.move_elbow_roll(angles_set[5], 28.1*speed)
    nao_controller.move_wrist_yaw(angles_set[7], speed)
    nao_controller.move_hip_pitch(angles_set[9], 11.6*speed)
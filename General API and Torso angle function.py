# -*- coding: utf-8 -*-
"""
Created on Thu Mar  9 16:24:04 2023

@author: 44759
"""
# -*- coding: utf-8 -*-

from naoqi import ALProxy
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

# Naos IP address is 192.168.1.2 for a wired connection and 192.168.1.3 for a wireless
# connection, in both cases the port number is 9559


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

    
    def move_elbow_yaw(self, angle, speed):
        self.move_joint("LElbowYaw", angle, speed)
        self.move_joint("RElbowYaw", -angle, speed)
    
    def move_head_yaw(self, angle, speed):
        self.move_joint("HeadYaw", angle, speed)
        
    def move_head_pitch(self, angle, speed):
        self.move_joint("HeadPitch", angle, speed)
        
    def move_hip_yaw_pitch(self, angle, speed):
        self.move_joint("LHipYawPitch", angle, speed)
        self.move_joint("RHipYawPitch", angle, speed)
    
    def move_ankle_roll(self, angle, speed):
        self.move_joint("LAnkleRoll", angle, speed)
        self.move_joint("RAnkleRoll", angle, speed)

    def move_hip_roll(self, angle, speed):
        self.move_joint("LHipRoll", angle, speed)
        self.move_joint("RHipRoll", angle, speed)   
    


# elbow yaw, head yaw, head pitch, hip yaw pitch, ankle roll, hip roll


#define set of angles corresponding to extended/contracted position
angles1 = calculate_angles(Torso_min, Leg_min)
angles2 = calculate_angles(Torso_max, Leg_max)
angles3 = calculate_angles(10, 180)  #possible neutral position angle set

# Define speed of joint movement
fraction_speed = 1
speed = fraction_speed / 28.1   #max value is 1/28.1, can take values between -1 and 1


# Instantiate NAOController class with wireless robot IP address
nao = NAOController("169.254.61.216")


from pmd import pmd1208fs   #canvas code to read large encoder angle
my_pmd = pmd1208fs()  
raw = my_pmd.digin16()
raw2 = raw & 2047
angle = float(raw2)*(360.0/2048.0)
del(my_pmd)

def set_neutral_position():   #sets initial neutral position (returning error currently)
    nao.move_knee_pitch(angles3[0], 0.5)
    nao.move_ankle_pitch(angles3[1], 0.5)
    nao.move_shoulder_pitch(angles3[2], 0.5)
    nao.move_shoulder_roll(angles3[3], 0.5)
    nao.move_elbow_roll(angles3[5], 0.5)
    nao.move_wrist_yaw(angles3[7], 0.5)
    nao.move_hip_pitch(angles3[9], 0.5)
    
    nao.move_elbow_yaw(-0.9971, 0.5)
    nao.move_head_yaw(0.0168, 0.5)
    nao.move_head_pitch(0.5148, 0.5)
    nao.move_hip_yaw_pitch(-0.0367, 0.5)
    nao.move_ankle_roll(-0.0199, 0.5)
    nao.move_hip_roll(-0.0014, 0.5)
    
set_neutral_position()
time.sleep(2)

data_list = []
while True:
#angle is the encoder angle that needs to be imported

    data_list.append(angle)
    while len(data_list)<100: # corresponds to 20 seconds of data (assuming 0.2s interval)

        A =  # more complex torso angle functions of e.g large encoder data, small encoder data, current joint positions
        B =  # ''   ''      leg angle    ''     ''
        anglesx = calculate_angles(A, B)
        nao.move_knee_pitch(anglesx[0], -26.8*speed)
        nao.move_ankle_pitch(anglesx[1], -15.7*speed)
        nao.move_shoulder_pitch(anglesx[2], -4.77*speed)
        nao.move_shoulder_roll(anglesx[3], 7.36*speed)
        nao.move_elbow_roll(anglesx[5], -28.1*speed)
        nao.move_wrist_yaw(anglesx[7], speed)
        nao.move_hip_pitch(anglesx[9], 11.6*speed)
     
def move_torso(ta, ts):   # ts between 0 and 1
    anglesta = calculate_angles(ta, leg_min)    
    speed = ts/28.1
    nao.move_shoulder_pitch(anglesta[2], 4.77*speed)
    nao.move_shoulder_roll(anglesta[3], 7.36*speed)
    nao.move_elbow_roll(anglesta[5], 28.1*speed)
    nao.move_wrist_yaw(anglesta[7], speed)
    nao.move_hip_pitch(anglesta[9], 11.6*speed)

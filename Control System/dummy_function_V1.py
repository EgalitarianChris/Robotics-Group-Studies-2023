# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 17:27:27 2023

@author: 44759
"""
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

angles1 = calculate_angles(Torso_min, Leg_min)
angles2 = calculate_angles(Torso_max, Leg_max)

def dummy_function(angle, ):
    speed = 0.03   #can change later
    0 < speed < 0.0355872   #normalised speeds below must have a magnitude < 1
    if angle > 0: #inwards
        Knee_pitch = angles1[0]
        Knee_speed = 26.8*speed
        Ankle_pitch = angles1[1]
        Ankle_speed = 15.7*speed
        Shoulder_pitch = angles1[2]
        Shoulder_pitch_speed = 4.77*speed
        LShoulder_roll = angles1[3]
        Shoulder_roll_speed = 7.36*speed  
        LElbow_roll = angles1[5]
        Elbow_roll_speed = 28.1*speed
        LWrist_yaw = angles1[7]
        Wrist_yaw_speed = speed
        Hip_pitch = angles1[9]
        Hip_pitch_speed = 11.6*speed
    else:  #outstretched
        Knee_pitch = angles2[0]
        Knee_speed = 26.8*speed
        Ankle_pitch = angles2[1]
        Ankle_speed = 15.7*speed
        Shoulder_pitch = angles2[2]
        Shoulder_pitch_speed = 4.77*speed
        LShoulder_roll = angles2[3]         # = -RShoulder_roll
        Shoulder_roll_speed = 7.36*speed  
        LElbow_roll = angles2[5]            # = -RElbow_roll
        Elbow_roll_speed = 28.1*speed
        LWrist_yaw = angles2[7]             # = -RWrist_yaw
        Wrist_yaw_speed = speed
        Hip_pitch = angles2[9]
        Hip_pitch_speed = 11.6*speed
    return Knee_pitch, Knee_speed, Ankle_pitch, Ankle_speed, Shoulder_pitch, Shoulder_pitch_speed, LShoulder_roll,
    Shoulder_roll_speed, LElbow_roll, Elbow_roll_speed, LWrist_yaw, Wrist_yaw_speed, Hip_pitch, Hip_pitch_speed 
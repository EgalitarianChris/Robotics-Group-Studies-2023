# -*- coding: utf-8 -*-
"""
Created on Thu Mar  2 14:52:49 2023

@author: user
"""
# import sys
# from naoqi import ALProxy
# import time


def calculate_angles(A, B):  #A, B are the desired torso/leg angle respectively
    Leg_min = 163   #min and max leg angle (not sure how accurate these are)
    Leg_max = 231
    Torso_min = 5    #min and max torso angle 
    Torso_max = 42
    
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

    print (Knee_pitch, Knee_pitch, Ankle_pitch, Ankle_pitch, Shoulder_pitch, Shoulder_pitch, LShoulder_roll, 
    RShoulder_roll, LElbow_roll, RElbow_roll, LWrist_yaw, RWrist_yaw, Hip_pitch, Hip_pitch)

angles_1 = calculate_angles(10, 200)
print(angles_1)

def main(robotIP):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    motionProxy.setStiffnesses("Body", 1.0)


    # Example showing how to set angles, using a fraction of max speed
    names  = ["LKneePitch", "RKneePitch", "LAnklePitch", "RAnklePitch", "LShoulderPitch", "RShoulderPitch", "LShoulderRoll", "RShoulderRoll", "LElbowRoll", "RElbowRoll", "LWristYaw", "RWrist Yaw", "LHipYawPitch", "RHipYawPitch"]
    angles  = angles_1
    fractionMaxSpeed  = 0.1
    motionProxy.setAngles(names, angles, fractionMaxSpeed)
    

    time.sleep(3.0)
    motionProxy.setStiffnesses("Body", 1.0)


if __name__ == "__main__":
    robotIp = "169.254.61.216"

    if len(sys.argv) <= 1:
        print "Usage python almotion_setangles.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
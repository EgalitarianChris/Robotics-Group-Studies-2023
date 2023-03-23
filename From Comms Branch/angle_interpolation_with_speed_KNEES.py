# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 14:41:20 2023

@author: saanc

This method takes 
"""

import sys
import time
from naoqi import ALProxy

def main(robotIP):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    motionProxy.setStiffnesses("Body", 1.0)

# THIS ONE MOVES LEFT AND RIGHT KNEE TOGETHER

    # Example showing multiple trajectories
    # Interpolate the head yaw to 1.0 radian and back to zero in 2.0 seconds
    # while interpolating HeadPitch up and down over a longer period.
    names  = ["LKneePitch","RKneePitch"]
    # Each joint can have lists of different lengths, but the number of
    # angles and the number of times must be the same for each joint.
    # Here, the second joint ("HeadPitch") has three angles, and
    # three corresponding times.
    angleLists  = [[1.0, 0.0], [-0.5, 0.5, 0.0]]
    timeLists   = [[1.0, 2.0], [ 1.0, 2.0, 3.0]]
    isAbsolute  = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

   # time.sleep(1.0)  UNNECESSARY UNLESS WANT TO DO SERIES OF CONSECUTIVE COMMANDS WITH TIME BETWEEN

# THIS ONE SHOWS HOW TO SET SPEEDS

    # # Example showing a single target for one joint
    # names             = "LKneePitch"
    # targetAngles      = 1.0
    # maxSpeedFraction  = 0.2 # Using 20% of maximum joint speed
    # motionProxy.angleInterpolationWithSpeed(names, targetAngles, maxSpeedFraction)

    # time.sleep(1.0)

# CAN SET LEFT AND RIGHT KNEE TOGETHER- this is unlikely to work but worth a try
# WE NEED TO FIGURE OUT IF THERE IS A CHAIN NAME CONNECTING L AND R KNEES

    # Example showing multiple joints
    # Instead of listing each joint, you can use a chain name, which will
    # be expanded to contain all the joints in the chain. In this case,
    # "Head" will be interpreted as ["HeadYaw", "HeadPitch"]
    names  = "Knee"
    # We still need to specify the correct number of target angles
    targetAngles     = [0.5, 0.25]
    maxSpeedFraction = 0.2 # Using 20% of maximum joint speed
    motionProxy.angleInterpolationWithSpeed(names, targetAngles, maxSpeedFraction)

#   motionProxy.setStiffnesses("Body", 0.0) # SET STIFFNESS BACK TO ZERO AFTER EXECUTING COMMANDS

    
if __name__ == "__main__":
    robotIp = "192.168.1.3"

    if len(sys.argv) <= 1:
        print "Usage python almotion_angleinterpolationwithspeed.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
# -*- coding: utf-8 -*-
'''
You want to run each of these four blocks of code separately.

This method takes positions and times as an input (motion potentially jerky)

THIS MIGHT BE GOOD BECAUSE CAN NORMALISE I.E. MAKE SURE ELBOWS AND KNEES MOVE FROM EXTREME POSITIONS TOGETHER

NEED TO FIGURE OUT ELBOW ROLL V ELBOW YAW
'''

import sys
import time
from naoqi import ALProxy
import almath

def main(robotIP):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    motionProxy.setStiffnesses("Body", 1.0) # set stiffness of body part you are trying to move to something non zero
    # set back to zero to relax Nao

# # JUST MOVE LEFT KNEE FORWARD
#     # Example showing a single target angle for one joint
#     # Interpolate the head yaw to 1.0 radian in 1.0 second
#     names      = "LKneePitch"
#     angleLists = 50.0*almath.TO_RAD
#     timeLists  = 1.0
#     isAbsolute = True
#     motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

#     time.sleep(1.0)

# # JUST MOVE LEFT KNEE FORWARD AND BACK

    # Example showing a single trajectory for one joint
    # Interpolate the head yaw to 1.0 radian and back to zero in 2.0 seconds
    names      = "LElbowRoll"
    #              2 angles
    angleLists = [-1.5, 0.0]
    #              2 times
    timeLists  = [1.0, 2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

    time.sleep(1.0)

# # THIS BLOCK OF CODE IS TO MAKE NAO DO TWO MOVEMENTS, ONE AFTER THE OTHER    
# # Example showing multiple trajectories
#     names      = ["HeadYaw", "HeadPitch"]
#     angleLists = [30.0*almath.TO_RAD, 30.0*almath.TO_RAD]
#     timeLists  = [1.0, 1.2]
#     isAbsolute = True
#     motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

'''
USE THE CODE BELOW - THINK ITS GONNA MOVE LEGS FORWARD AND BACK SIMULTANEOUSLY
'''
# THIS BLOCK OF CODE IS TO MAKE NAO DO TWO MOVEMENTS SIMULTANEOUSLY- hopefully move right and left knee forward and back simultaneously
    # Example showing multiple trajectories
    # Interpolate the head yaw to 1.0 radian and back to zero in 2.0 seconds
    # while interpolating HeadPitch up and down over a longer period.
    names  = ["LElbowRoll","RElbowRoll"]
    # Each joint can have lists of different lengths, but the number of
    # angles and the number of times must be the same for each joint.
    # Here, the second joint ("HeadPitch") has three angles, and
    # three corresponding times.
    angleLists  = [[-1.50, -0.04, 0.00],
                   [0.04, 1.50, 0.00]]
    timeLists   = [[1.0, 2.0, 3.0], [ 1.0, 2.0, 3.0]]
    isAbsolute  = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

    motionProxy.setStiffnesses("Body", 0.0)


if __name__ == "__main__":
    robotIp = "192.168.1.3"

    if len(sys.argv) <= 1:
        print "Usage python almotion_angleinterpolation.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
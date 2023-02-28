# -*- coding: utf-8 -*-
"""
Created on Tue Feb 28 17:29:25 2023

@author: Yuusf Choudhury
"""

import sys
from naoqi import ALProxy
import time

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
    names  = ["LElbowRoll", "RElbowRoll", "LShoulderPitch", "RShoulderPitch", "LKneePitch", "RKneePitch"]
    angles  = [-1.4633, 1.4633, 1.4020, 1.4020, 1.1565, 1.1565]
    fractionMaxSpeed  = 0.1
    motionProxy.setAngles(names, angles, fractionMaxSpeed)
    
    time.sleep(2.0)
    
    names  = ["LElbowRoll", "RElbowRoll", "LShoulderPitch", "RShoulderPitch", "LKneePitch", "RKneePitch"]
    angles  = [-0.3098, 0.3098, 1.1044, 1.1044, -0.0905, -0.0905]
    fractionMaxSpeed  = 0.3
    motionProxy.setAngles(names, angles, fractionMaxSpeed)
    
    time.sleep(0.5)
    
    names  = ["LElbowRoll", "RElbowRoll", "LShoulderPitch", "RShoulderPitch", "LKneePitch", "RKneePitch"]
    angles  = [-1.4802, 1.4802, 1.4634, 1.4634, 1.3989, 1.3989]
    fractionMaxSpeed  = 0.3
    motionProxy.setAngles(names, angles, fractionMaxSpeed)
    
    time.sleep(0.5)
        
    # names  = ["LHipYawPitch", "RHipYawPitch"]
    # angles  = [-0.0367, -0.0367]
    # fractionMaxSpeed  = 0.3
    # motionProxy.setAngles(names, angles, fractionMaxSpeed)
    
    # time.sleep(0.5)


    names  = ["LElbowRoll", "RElbowRoll", "LShoulderPitch", "RShoulderPitch", "LKneePitch", "RKneePitch"]
    angles  = [-1.5155, 1.5155, 1.4495, 1.4495, -0.0923, -0.0923]
    fractionMaxSpeed  = 0.3
    motionProxy.setAngles(names, angles, fractionMaxSpeed)
    
    time.sleep(0.5)
        
    names  = ["LElbowRoll", "RElbowRoll", "LShoulderPitch", "RShoulderPitch", "LKneePitch", "RKneePitch"]
    angles  = [-0.3067, 0.3067, 1.1197, 1.1197, 1.4005, 1.4005]
    fractionMaxSpeed  = 0.3
    motionProxy.setAngles(names, angles, fractionMaxSpeed)
    
    
    # names  = ["LKneePitch", "RKneePitch"]
    # angles  = [-1, -1]
    # fractionMaxSpeed  = 0.2
    # motionProxy.setAngles(names, angles, fractionMaxSpeed)

    time.sleep(3.0)
    motionProxy.setStiffnesses("Body", 1.0)


if __name__ == "__main__":
    robotIp = "192.168.1.2"

    if len(sys.argv) <= 1:
        print "Usage python almotion_setangles.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
    

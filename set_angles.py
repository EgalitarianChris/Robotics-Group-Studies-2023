# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 14:43:03 2023

@author: saanc
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

    motionProxy.setStiffnesses("Head", 1.0)

    # Example showing how to set angles, using a fraction of max speed
    names  = ["HeadYaw", "HeadPitch"]
    angles  = [0.2, -0.2]
    fractionMaxSpeed  = 0.2
    motionProxy.setAngles(names, angles, fractionMaxSpeed)

    time.sleep(3.0)
    motionProxy.setStiffnesses("Head", 0.0)


if __name__ == "__main__":
    robotIp = "192.168.1.3"

    if len(sys.argv) <= 1:
        print "Usage python almotion_setangles.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
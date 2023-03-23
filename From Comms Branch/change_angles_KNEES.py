# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 14:43:39 2023

@author: saanc
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

    # Example showing a slow, relative move of "HeadYaw".
    # Calling this multiple times will move the head further.
    names            = "LKneePitch"
    changes          = 0.5
    fractionMaxSpeed = 0.05
    motionProxy.changeAngles(names, changes, fractionMaxSpeed)

    time.sleep(2.0)

    motionProxy.setStiffnesses("Body", 0.0)


if __name__ == "__main__":
    robotIp = "192.168.1.3"

    if len(sys.argv) <= 1:
        print "Usage python almotion_changeangles.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
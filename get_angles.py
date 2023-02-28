# -*- coding: utf-8 -*-
"""
Created on Tue Feb 28 14:43:11 2023

@author: Yuusf Choudhury
"""

# -*- encoding: UTF-8 -*-

import sys
from naoqi import ALProxy


def main(robotIP):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    # Example that finds the difference between the command and sensed angles.
    names         = "LHipYawPitch"
    useSensors    = True
    commandAngles = motionProxy.getAngles(names, useSensors)
    print "Command angles:"
    print str(commandAngles)
    print ""


if __name__ == "__main__":
    robotIp = "192.168.1.2"

    if len(sys.argv) <= 1:
        print "Usage python almotion_getangles.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
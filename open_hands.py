# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 14:44:35 2023

@author: saanc
"""

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

    # Example showing how to open the left hand
    motionProxy.openHand('LHand')


if __name__ == "__main__":
    robotIp = "192.168.1.3"

    if len(sys.argv) <= 1:
        print "Usage python almotion_openhand.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
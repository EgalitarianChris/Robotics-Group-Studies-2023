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

    motionProxy.setStiffnesses("Body", 1.0)
    
    starttime = time.time()
    while True:
        #print("hello")  #just as a test
        names         = "LKneePitch"
        useSensors    = True
        commandAngles = motionProxy.getAngles(names, useSensors)
        print "Command angles:"
        print str(commandAngles)
        print ""
        time.sleep(0.01 - ((time.time() - starttime) % 0.01)) #hopefully repeats every 0.01s

    # Example showing how to set angles, using a fraction of max speed
    name  = "LKneePitch"
    angle  = 0.1
    fractionMaxSpeed  = 0.2
    motionProxy.setAngles(name, angle, fractionMaxSpeed)
   
    name  = "LKneePitch"
    angle  = 0.2
    fractionMaxSpeed  = 0.4
    motionProxy.setAngles(name, angle, fractionMaxSpeed)
    
    name  = "LKneePitch"
    angle  = 0.4
    fractionMaxSpeed  = 0.6
    motionProxy.setAngles(name, angle, fractionMaxSpeed)

    time.sleep(3.0)
    motionProxy.setStiffnesses("Body", 0.0)


if __name__ == "__main__":
    robotIp = "192.168.1.2"

    if len(sys.argv) <= 1:
        print "Usage python almotion_setangles.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
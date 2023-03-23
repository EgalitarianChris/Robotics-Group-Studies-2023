from naoqi import ALProxy

def move(limb, angle):
    motionProxy = ALProxy("ALMotion", "192.168.1.3", 9559)
    motionProxy.setStiffnesses("Head", 1.0)
    motionProxy.setAngles(limb, angle, 0.1)

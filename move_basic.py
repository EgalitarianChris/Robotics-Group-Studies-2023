# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 14:33:58 2023

@author: saanc
"""

from naoqi import ALProxy
motion = ALProxy("ALMotion", "192.168.1.3", 9559)
motion.moveInit()
motion.moveTo(0.5, 0, 0)

# Make Nao move
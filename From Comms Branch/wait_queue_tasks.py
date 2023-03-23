# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 14:38:03 2023

@author: saanc
"""

from naoqi import ALProxy
motion = ALProxy("ALMotion", "192.168.1.3", 9559)
motion.moveInit()
id = motion.post.moveTo(0.5, 0, 0)
motion.wait(id, 0)

# 'Wait' used to implement task after a given task is finished
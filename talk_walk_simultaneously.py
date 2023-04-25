# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 14:34:35 2023

@author: saanc
"""

from naoqi import ALProxy
motion = ALProxy("ALMotion", "192.168.1.3", 9559)
tts    = ALProxy("ALTextToSpeech", "192.168.1.3", 9559)
motion.moveInit()
motion.post.moveTo(0.5, 0, 0)
tts.say("I'm walking")

# 'Post' used to call long methods in background- can make robot do several things at once.
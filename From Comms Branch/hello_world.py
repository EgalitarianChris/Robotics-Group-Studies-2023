# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 14:32:37 2023

@author: saanc
"""

from naoqi import ALProxy
tts = ALProxy("ALTextToSpeech", "192.168.1.3", 9559)
tts.say("Hello, world!")
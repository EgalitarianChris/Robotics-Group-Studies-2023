# -*- coding: utf-8 -*-
"""
Created on Fri Mar  3 17:32:11 2023

@author: bxd051
"""

def programatic(observation):
    '''
    

    Parameters
    ----------
    observation : Array-like
        8 variable array of leg angle, torso angle, top angle, combined angle and angular speeds
        units of degrees and degrees/second
    Returns
    -------
    action : Array-like
        action space of intended leg angle and speed + intended torso angle and speed
        all normalised between -1 and 1

    '''
    
    return action
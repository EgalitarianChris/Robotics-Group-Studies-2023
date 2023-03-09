# -*- coding: utf-8 -*-
"""
Created on Fri Mar  3 17:32:11 2023

@author: bxd051
"""

import numpy as np
from ML import CustomEnv

def programatic(observation, old = np.array([0,-1,0,-1])):
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
    action = old#do nothing new generally
    if 10 > np.abs(observation[6]) and observation[2] < 0:
        action = np.array([-1,1,-1,65/126])
        #if we're close to the end of the motion, and on the right, tuck as much as possible
    elif 10 > np.abs(observation[6]) and observation[2] > 0:
        action = np.array([1,1,1,65/126])
        #if we're close to the end of the motion, and on the left, extend as much as possible
    print(round(observation[6], 2), action)
    old_for_next = action
    return action, old_for_next


def main():
    """
    Runs the simulation manually, no machine learning here.
    Instantiates the custom Gym environment, listens for keypresses
    then sets action based on input.

    Returns
    -------
    None.

    """
    # Initialise the simulation:
    environment = CustomEnv()
    environment.init_render()
    # check_env(environment)
    # Run the simulation:
    observation = environment.get_obs()
    action, last_action = programatic(observation)

    # Step the simulation, then render the result (rendering in pymunk)
    environment.step(action)
    environment.render()
    
    while True:
        observation = environment.get_obs()
        action, last_action = programatic(observation, last_action)

        # Step the simulation, then render the result (rendering in pymunk)
        environment.step(action)
        environment.render()

if __name__ == "__main__":
    main()

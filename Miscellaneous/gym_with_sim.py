"""
Basic example of a pendulum system for testing ML implementation in gym environment using Pymunk for simulation.
By Hal - 2023
"""

# Import relevent libraries
import sim_for_gym as sim
import pygame
import numpy as np
import pymunk
import gym
from pymunk.pygame_util import DrawOptions

# Create gym environment - Contains the machine learning code and the simulation code:
class CustomEnv(gym.Env):

    # Initialising the environment - SINGLE SETUP FUNCTION CALL to be written by simulations team:
    def __init__(self, env_config={}):
        self.simulation_data = setup_simulation()

    # The actual bit where the simulation happens
    def step(self, action=np.zeros((4), dtype=np.single)):
        self.simulation_data = perform_action(self, action, self.simulation_data)
        self.simulation_data["pm_space"].step(1/1000)
        self.leg_angle = 180/np.pi*(self.simulation_data["pm_space"].bodies[3].angle-self.simulation_data["pm_space"].bodies[3].angle)

        observation, reward, done, info = 0., 0., False, {}
        return observation, reward, done, info


    # Initialise the renderer (NOT RELEVENT TO SIMULATIONS)
    def init_render(self):
        pygame.init()
        self.window = pygame.display.set_mode((1000, 500))
        self.simulation_data["pm_space"].gravity = 0, 981
        self.options = DrawOptions(self.window)
        self.clock = pygame.time.Clock()

    # Render the state of the simulation (NOT RELEVENT TO SIMULATIONS)
    def render(self):
        self.window.fill((255, 255, 255))
        self.simulation_data["pm_space"].debug_draw(self.options)
        pygame.display.update()

    # Reset the simulation for the next training run (NOT RELEVENT TO SIMULATIONS)
    def reset(self):
        pass


#----------------------------------------------------------------------------------------------------------------------
# FUNCTIONS FOR THE SIMULATION TEAM TO MODIFY WITH THEIR SETUP:

def setup_simulation():
    
    pm_space = pymunk.Space()
    pm_space.gravity = 0, 981
    background = pm_space.static_body
    
    setup = {
        #lengths /cm - innacurate
        "rl": 151 + 7,
        "sl1": 16,
        "sl2": 19 + 5,
        "sl3": 17,
        "ll1": 16,
        "ll2": 6,
        "tl": 26,
        
        "bg": (400,200),
        
        #masses /kg
        "rm": 1.235 + 0.381/2,
        "sm1": 0.381/2,
        "sm2": 1.026,
        "sm3": 0.131*2 + 0.070 + 0.390*2,
        "lm1": 0.292*2 + 0.134,
        "lm2": 0.162*2 + 0.134,
        "tm": 1.050 + 0.064 + 0.605 + 0.075 + 0.070,
    }
    centres = {
        "rc": (setup["bg"][0], setup["bg"][1] + setup["rl"]/2),
        "sc": (setup["bg"][0], setup["bg"][1] + setup["rl"] + setup["sl2"]/2),
        "lc": (setup["bg"][0] + setup["sl3"]/2, setup["bg"][1] + setup["rl"] + setup["sl2"] + setup["ll1"]/2),
        "tc": (setup["bg"][0] - setup["sl3"]/2, setup["bg"][1] + setup["rl"] + setup["sl2"] - setup["tl"]/2),
    }
        
    #these add the object to the simulation
    bodies = {
    "rod": sim.Rod(centres["rc"], (0,setup["rl"]/2), (0,-setup["rl"]/2), setup["rm"], pm_space),
    "swing": sim.Swing(centres["sc"], (0,-setup["sl2"]/2), (setup["sl1"]/2,-setup["sl2"]/2), (0,setup["sl2"]/2), (0,-setup["sl2"]/2), (-setup["sl3"]/2 -0.5,setup["sl2"]/2), (setup["sl3"]/2 -0.5,setup["sl2"]/2), setup["sm1"], setup["sm2"], setup["sm3"], pm_space),
    "leg": sim.Leg(centres["lc"], (0, - setup["ll1"]/2), (0, setup["ll1"]/2), (0, setup["ll1"]/2), (setup["ll2"], setup["ll1"]/2), setup["lm1"], setup["lm2"], pm_space),
    "torso": sim.Torso(centres["tc"], (0,-setup["tl"]/2), (0,setup["tl"]/2), 2, pm_space),
    }
    
    #fixed joints of simulation
    joints = {
    "back": sim.Pinjoint(bodies["swing"].body, bodies["torso"].body, (-setup["sl3"]/2 -0.5,setup["sl2"]/2), (0,setup["tl"]/2), pm_space),
    "front": sim.Pinjoint(bodies["swing"].body, bodies["leg"].body, (setup["sl3"]/2 -0.5,setup["sl2"]/2), (0, -setup["ll1"]/2), pm_space),
    "bottom": sim.Pinjoint(bodies["rod"].body, bodies["swing"].body, (0,setup["rl"]/2), (0,-setup["sl2"]/2), pm_space),
    "top": sim.Pinjoint(background, bodies["rod"].body, setup["bg"], (0,-setup["rl"]/2), pm_space),
    }
    
    motors = {
    "back": sim.Simplemotor(bodies["swing"].body, bodies["leg"].body, 0, pm_space),
    "front": sim.Simplemotor(bodies["swing"].body, bodies["torso"].body, 0, pm_space)
    }
    
    return {"pm_space":pm_space, "motors":motors, "bodies":bodies, "joints":joints}

def perform_action(environment, action, simulation_data):
    if action[1] != 0 and abs(action[0] - environment.leg_angle) >= 0.01 :
        pass
        remove_motor_l(simulation_data)
        add_motor_l(simulation_data, action[1])
    else:
        remove_motor_l(simulation_data)
        add_motor_l(simulation_data, 0)
        pass
    return simulation_data

def add_motor_l(simulation_data, speed):
    simulation_data["motors"]["front"] = sim.Simplemotor(simulation_data["bodies"]["swing"].body, simulation_data["bodies"]["leg"].body, speed, simulation_data["pm_space"])

def remove_motor_l(simulation_data):
    simulation_data["motors"]["front"] = sim.Simplemotor(simulation_data["bodies"]["swing"].body, simulation_data["bodies"]["leg"].body, 0, simulation_data["pm_space"])


#----------------------------------------------------------------------------------------------------------------------

# Look at which keys have been pressed, decide what action to apply to the simulation
def get_action_from_keypress(keytouple):
    if keytouple[pygame.K_LEFTBRACKET]:
        return np.array([90, 1, 0, 0])
    elif keytouple[pygame.K_RIGHTBRACKET]:
        return np.array([-90, -1, 0, 0])
    else:
        return np.array([0, 0, 0, 0])
    

# Fetch pygame events
def get_events():
    get_event = pygame.event.get()
    for event in get_event:
        if event.type == pygame.QUIT:
            pygame.quit()
    return pygame.key.get_pressed()

# Main loop - actually runs the code
def main():
    # Initialise the simulation:
    environment = CustomEnv()
    environment.init_render()

    # Run the simulation:
    while True:
        # Get the action to do in the simulation (in this case, true or false for applying a torque impulse to the pendulum in response to a keypress):
        keys_pressed = get_events()
        action = get_action_from_keypress(keys_pressed)

        # Step the simulation, then render the result (rendering in pymunk)
        environment.step(action)
        environment.render()

main()
"""
Basic example of a pendulum system for testing ML implementation in gym environment using Pymunk for simulation.
By Hal - 2023
"""

# Import relevent libraries
import pygame
import numpy as np
import pymunk
import gym
from pymunk.pygame_util import DrawOptions

# Create gym environment - Contains the machine learning code and the simulation code:
class CustomEnv(gym.Env):

    # Initialising the environment - SINGLE SETUP FUNCTION CALL to be written by simulations team:
    def __init__(self, env_config={}):
        self.simulation_data = setup_simulation({"pivot_position":(400,100),"pendulum_length":100})

    # The actual bit where the simulation happens
    def step(self, action=np.zeros((1), dtype=np.single)):
        self.simulation_data = perform_action(action,self.simulation_data)
        self.simulation_data["pm_space"].step(1/1000)
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

def setup_simulation(setup_data):
    motor_active = False
    timer = 0
    duration = 40

    pm_space = pymunk.Space()
    motor_rate = 3
    pivot_body = pm_space.static_body

    body = pymunk.Body()
    body.position = (400, 200)

    shape = pymunk.Segment(body, (0, -100), (0, 100), 10)
    shape.density = 1
    pm_space.add(body, shape)

    pivot_joint = pymunk.constraints.PinJoint(pivot_body, body, (400, 100), (0, -100))
    pm_space.add(pivot_joint)


    return {"pm_space":pm_space, "motor_active":motor_active, "timer":timer, "duration":duration, "motor_rate":motor_rate, "pivot_body":pivot_body, "body":body}

def perform_action(action,simulation_data):
    if simulation_data["motor_active"]:
        if simulation_data["timer"] <= simulation_data["duration"]:
            simulation_data["timer"] += 1
        else:
            simulation_data["motor_active"] = False
            simulation_data = remove_motor(simulation_data)
    else:
        if action[0]:
            simulation_data = add_motor(simulation_data)
            simulation_data["timer"] = 0
            simulation_data["motor_active"] = True
    return simulation_data

def add_motor(simulation_data):
    simulation_data["motor"] = pymunk.SimpleMotor(simulation_data["pivot_body"],simulation_data["body"],simulation_data["motor_rate"])
    simulation_data["pm_space"].add(simulation_data["motor"])
    return simulation_data

def remove_motor(simulation_data):
    simulation_data["pm_space"].remove(simulation_data["motor"])
    return simulation_data

#----------------------------------------------------------------------------------------------------------------------

# Look at which keys have been pressed, decide what action to apply to the simulation
def get_action_from_keypress(keytouple):
    impulse = False
    if keytouple[pygame.K_SPACE]:
        impulse = True
    return np.array([impulse])

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

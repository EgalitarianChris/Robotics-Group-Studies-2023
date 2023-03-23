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
from gym import spaces
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO

# Create gym environment - Contains the machine learning code and the simulation code:
class CustomEnv(gym.Env):
    # Initialising the environment - SINGLE SETUP FUNCTION CALL to be written by simulations team:
    def __init__(self, env_config={}):
        self.run_duration = 30000
        self.run_time = 0
        self.last_theta = 0
        self.timestep = 1/500

        self.action_space = spaces.Box(0,1,(1,),dtype = int)
        self.observation_space = spaces.Box(np.array([-np.pi,-10]),np.array([np.pi,10]),(2,),dtype=np.float32)

        self.simulation_data = setup_simulation({"pivot_position":(400,100),"pendulum_length":100})

    # The actual bit where the simulation happens
    # def step(self, action=np.zeros((1), dtype=np.single)):
    def step(self, action):
        self.simulation_data = perform_action(action,self.simulation_data)
        self.simulation_data["pm_space"].step(self.timestep)

        observation = self.get_obs()
        reward = self.get_reward(observation)
        info = self.get_info()
        done = self.quit_timer()
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
        get_events()
        self.window.fill((255, 255, 255))
        self.simulation_data["pm_space"].debug_draw(self.options)
        pygame.display.update()

    # Reset the simulation for the next training run (NOT RELEVENT TO SIMULATIONS)
    def reset(self):
        self.simulation_data = setup_simulation({"pivot_position":(400,100),"pendulum_length":100})
        observation = self.get_obs()
        return observation

    def get_obs(self):
        theta = np.array([np.arctan2(self.simulation_data["body"].position[0]-400,self.simulation_data["body"].position[1]-100)])
        angular_velocity = (theta - self.last_theta)/self.timestep
        return np.array([theta, angular_velocity])

    def get_reward(self,observation):
        reward = float((np.sin(observation)[0]+1)**3)
        return reward

    def get_info(self):
        return {"empty":None}

    def quit_timer(self):
        if self.run_time >= self.run_duration:
            return True
        else:
            self.run_time += 1
            return False


#----------------------------------------------------------------------------------------------------------------------
# FUNCTIONS FOR THE SIMULATION TEAM TO MODIFY WITH THEIR SETUP:

def setup_simulation(setup_data):
    motor_active = False
    timer = 0
    duration = 20
    cooldown_timer = 0
    cooldown_duration = 200

    pm_space = pymunk.Space()
    motor_rate = 2
    pivot_body = pm_space.static_body

    body = pymunk.Body()
    body.position = (400, 200)

    shape = pymunk.Segment(body, (0, -100), (0, 100), 10)
    shape.density = 1
    pm_space.add(body, shape)

    pivot_joint = pymunk.constraints.PinJoint(pivot_body, body, (400, 100), (0, -100))
    pm_space.add(pivot_joint)


    return {"pm_space":pm_space, "motor_active":motor_active, "timer":timer, "duration":duration, "motor_rate":motor_rate, "pivot_body":pivot_body, "body":body, "cooldown_timer":cooldown_timer,"cooldown_duration":cooldown_duration}

def perform_action(action,simulation_data):
    simulation_data["cooldown_timer"] += 1
    if simulation_data["motor_active"]:
        if simulation_data["timer"] <= simulation_data["duration"]:
            simulation_data["timer"] += 1
        else:
            simulation_data["motor_active"] = False
            simulation_data = remove_motor(simulation_data)
    else:
        if action[0] and simulation_data["cooldown_timer"] >= simulation_data["cooldown_duration"]:
            simulation_data = add_motor(simulation_data)
            simulation_data["timer"] = 0
            simulation_data["cooldown_timer"] = 0
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
    check_env(environment)
    # Run the simulation:
    while True:
        # Get the action to do in the simulation (in this case, true or false for applying a torque impulse to the pendulum in response to a keypress):
        keys_pressed = get_events()
        action = get_action_from_keypress(keys_pressed)

        # Step the simulation, then render the result (rendering in pymunk)
        environment.step(action)
        environment.render()

main()

# --------------------------------------------------------------------
# PPO stuff

def PPO_main():
    env = CustomEnv()
    model = PPO("MlpPolicy",env,verbose=1)
    model.learn(total_timesteps=1600000)
    model.save("test_PPO_model_data")
    print("model saved\n---------------------------------------------------------")
    del model

    model = PPO.load("test_PPO_model_data")
    print("model loaded\n---------------------------------------------------------")
    obs = env.reset()
    print("initialising renderer")
    env.init_render()
    print("starting while loop (running the trained model)")
    while True:
        action, _states = model.predict(obs)
        print("action:",action)
        obs, rewards, done, info = env.step(action)
        print("observation:",obs,"rewards:",rewards)
        env.render()

#PPO_main()

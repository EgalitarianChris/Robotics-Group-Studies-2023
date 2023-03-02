# Import relevent libraries
from Sim import setup_simulation, perform_action, get_action
from effort_parameter import get_effort
import pygame
import numpy as np
import gym
from gym import spaces
from pymunk.pygame_util import DrawOptions
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO


# Create gym environment - Contains the machine learning code and the simulation code:
class CustomEnv(gym.Env):

    # Initialising the environment - SINGLE SETUP FUNCTION CALL to be written by simulations team:
    def __init__(self, env_config={}):
        self.run_duration = 10000
        self.run_time = 0
        self.reward = 0
        self.step_length = 1 / 1000
        self.observation = np.zeros(12)

        self.action_space = spaces.Box(np.array([-45, 0, 63, 0]), np.array([90, 360, 128, 360]), dtype=float)
        self.observation_space = spaces.Box(np.array([-180, -180, -180, -180, -360, -360, -180, -180, -100, -100, -100, -100]), np.array([180, 180, 180, 180, 360, 360, 180, 180, 100, 100, 100, 100]), dtype=np.float32)

        self.simulation_data = setup_simulation()

    # The actual bit where the simulation happens
    def step(self, action=np.zeros((4), dtype=np.single)):
        self.simulation_data = perform_action(self, action, self.simulation_data)
        self.last_action = action
        self.simulation_data["pm_space"].step(self.step_length) # might want to include a bit of random variation to the step duration to help train the agent for running on NAO

        observation = self.get_obs()

        self.reward += self.get_reward(observation)
        info = self.get_info()
        done = self.quit_timer()
        self.observation = observation
        return observation, self.reward, done, info

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
        self.run_time = 0
        self.reward = 0
        self.simulation_data = setup_simulation()
        observation = self.get_obs()
        return observation

    def get_obs(self):
                #could optimise by making array
        leg_angle = 180/np.pi * (self.simulation_data["pm_space"].bodies[3].angle -self.simulation_data["pm_space"].bodies[1].angle)
        leg_angle_velocity = 180/np.pi * (self.simulation_data["pm_space"].bodies[3].angular_velocity -self.simulation_data["pm_space"].bodies[1].angular_velocity)
        leg_angle_acc = (leg_angle_velocity - self.observation[4]) / self.step_length

        torso_angle = 180/np.pi * (self.simulation_data["pm_space"].bodies[2].angle -self.simulation_data["pm_space"].bodies[1].angle)
        torso_angle_velocity = 180/np.pi * (self.simulation_data["pm_space"].bodies[2].angular_velocity -self.simulation_data["pm_space"].bodies[1].angular_velocity)
        torso_angle_acc = (torso_angle_velocity - self.observation[5]) / self.step_length

        top_angle = 180/np.pi * (self.simulation_data["pm_space"].bodies[0].angle - self.simulation_data["setup"]["phi"])
        top_angle_velocity = 180/np.pi * (self.simulation_data["pm_space"].bodies[0].angular_velocity)
        top_angle_acc = (top_angle_velocity - self.observation[6]) / self.step_length

        combined_joint_angle = top_angle - 180/np.pi * (self.simulation_data["pm_space"].bodies[1].angle)
        combined_joint_angle_velocity = 180/np.pi * (self.simulation_data["pm_space"].bodies[0].angular_velocity - self.simulation_data["pm_space"].bodies[1].angular_velocity)
        combined_joint_angle_acc = (combined_joint_angle_velocity - self.observation[7]) / self.step_length

        observation = np.array([leg_angle, torso_angle, top_angle, combined_joint_angle, leg_angle_velocity, torso_angle_velocity, top_angle_velocity, combined_joint_angle_velocity, leg_angle_acc, torso_angle_acc, top_angle_acc, combined_joint_angle_acc])
        # print(observation)
        return observation

    def get_reward(self, observation):
        k1,k2,k3,k4,k5,k6,k7 = 1,0,0,0,0,0,0
        top_angle, combined_joint_angle  = observation[2:4]
        leg_acc, torso_acc = observation[7:9]
        reward = top_angle * top_angle
        penalty = combined_joint_angle * combined_joint_angle
        effort = get_effort(self, leg_acc, torso_acc,k4,k5,k6,k7)
        # print(effort,reward,penalty)
        return k1*reward - k2*penalty - k3*effort

    def get_info(self):
        return {"empty":None}

    def quit_timer(self):
        if self.run_time >= self.run_duration:
            return True
        else:
            self.run_time += 1
            return False


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
        keys_pressed = get_events()
        action = get_action(keys_pressed)

        # Step the simulation, then render the result (rendering in pymunk)
        environment.step(action)
        environment.render()


main()

# --------------------------------------------------------------------
# PPO stuff

def PPO_main():
    env = CustomEnv()
    model = PPO("MlpPolicy",env,verbose=1)
    model.learn(total_timesteps=1000000)
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

# PPO_main()

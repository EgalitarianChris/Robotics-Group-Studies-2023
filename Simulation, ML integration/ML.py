# Import relevent libraries
import time
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
from Sim import setup_simulation, perform_action, get_action
# from effort_parameter import get_effort
import pygame
import numpy as np
import gym
from gym import spaces
from pymunk.pygame_util import DrawOptions
# from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

# Create gym environment - Contains the machine learning code and the simulation code:
class CustomEnv(gym.Env):

    # Initialising the environment - SINGLE SETUP FUNCTION CALL to be written by simulations team:
    def __init__(self):
        self.run_time = 0
        self.reward = 0
        self.realtime_ep_duration = 50
        self.observation = np.zeros(12)
        self.action_space = spaces.Box(np.array([-1., -1., -1., -1.]),
                                       np.array([1., 1., 1., 1.]),
                                       dtype=np.float64)
        self.observation_space = spaces.Box(np.array([-180., -180., -180., -180., -360., -360., -180., -180.]),
                                            np.array([180., 180., 180., 180., 360., 360., 180., 180.]),
                                            dtype=np.float64)
        self.simulation_data = setup_simulation()
        self.sim_steps_per_decision = self.simulation_data["setup"]["sim_steps_per_decision"]
        self.step_length = self.simulation_data["setup"]["step_length"] * self.sim_steps_per_decision
        self.run_duration = self.realtime_ep_duration / self.step_length
        self.window = None
        self.options = None

    def step(self, action=np.zeros(4), dtype=np.single):
        """
        The actual bit where the simulation happens. Ticks everything forward by one timestep
        and returns values which get passed into PPO.
        Parameters
        ----------
        action : array-like
            A normalised array of how the arms and legs intend to move. The default is an array of zeros.
        Returns
        -------
        observation : array-like
            Angles and velocities in the swing which gets passed into PPO.
        reward : float
            A score of how successfully the robot is swinging, calculated from get_reward().
        done : boolean
            Stops the PPO algorithm when it is finished, determined from quit_timer().
        info : dict
            Doesn't do anything yet.
        """
        self.simulation_data = perform_action(self, action, self.simulation_data)
        self.last_action = action
        for i in range(self.sim_steps_per_decision):
            self.simulation_data["pm_space"].step(self.step_length/self.sim_steps_per_decision)
        # Potentially add random variation in step duration to help train for running on NAO

        observation = self.get_obs()

        reward = self.get_reward(observation, action)
        info = self.get_info()
        done = self.quit_timer()
        self.observation = observation
        return observation, reward, done, info


    def init_render(self):
        """
        Initialises the renderer, is only called to visualise the simulation,
        it isn't necessary for the machine learning. (NOT RELEVENT TO SIMULATIONS)
        Returns
        -------
        None.
        """
        pygame.init()
        self.window = pygame.display.set_mode((1000, 500))
        self.simulation_data["pm_space"].gravity = 0, 981
        self.options = DrawOptions(self.window)
        self.clock = pygame.time.Clock()

    def render(self):
        """
        Renders the state of the simulation (NOT RELEVENT TO SIMULATIONS)
        Returns
        -------
        None.
        """
        get_events()
        self.window.fill((255, 255, 255))
        self.simulation_data["pm_space"].debug_draw(self.options)
        pygame.display.update()
        self.clock.tick(1/self.step_length)

    # Reset the simulation for the next training run (NOT RELEVENT TO SIMULATIONS)
    def reset(self):
        """
        Used to initiate a new episode, outputting final observations.
        Returns
        -------
        observation : array-like
            Angles and velocities in the swing which gets passed into PPO.
        """
        self.run_time = 0
        self.reward = 0
        self.simulation_data = setup_simulation()
        observation = self.get_obs()
        return observation

    def get_obs(self):      #could optimise by making array
        """
        Calculates leg, torso, top pivot and combined joint angles and velocities.
        Gets passed into observation array for PPO to use.
        Returns
        -------
        observation : array-like
            Angles and velocities in the swing which gets passed into PPO.
        """
        leg_angle = 180/np.pi * (self.simulation_data["pm_space"].bodies[3].angle - self.simulation_data["pm_space"].bodies[1].angle)
        leg_angle_velocity = 180/np.pi * (self.simulation_data["pm_space"].bodies[3].angular_velocity -self.simulation_data["pm_space"].bodies[1].angular_velocity)
        # leg_angle_acc = (leg_angle_velocity - self.observation[4]) / self.step_length

        torso_angle = 180/np.pi * (self.simulation_data["pm_space"].bodies[2].angle -self.simulation_data["pm_space"].bodies[1].angle)
        torso_angle_velocity = 180/np.pi * (self.simulation_data["pm_space"].bodies[2].angular_velocity -self.simulation_data["pm_space"].bodies[1].angular_velocity)
        # torso_angle_acc = (torso_angle_velocity - self.observation[5]) / self.step_length

        top_angle = 180/np.pi * (self.simulation_data["pm_space"].bodies[0].angle - self.simulation_data["setup"]["phi"])
        top_angle_velocity = 180/np.pi * (self.simulation_data["pm_space"].bodies[0].angular_velocity)
        # top_angle_acc = (top_angle_velocity - self.observation[6]) / self.step_length

        combined_joint_angle = top_angle - 180/np.pi * (self.simulation_data["pm_space"].bodies[1].angle)
        combined_joint_angle_velocity = 180/np.pi * (self.simulation_data["pm_space"].bodies[0].angular_velocity - self.simulation_data["pm_space"].bodies[1].angular_velocity)
        # combined_joint_angle_acc = (combined_joint_angle_velocity - self.observation[7]) / self.step_length

        observation = np.array([leg_angle, torso_angle, top_angle, combined_joint_angle,
                                leg_angle_velocity, torso_angle_velocity, top_angle_velocity,
                                combined_joint_angle_velocity])
        return observation

    def get_reward(self, observation, action):
        """
        Takes in observations and uses them to calculate a reward function.
        k_1 through k_4 are parameters used to tweak the importance of different
        behaviour in the neural network.
        Parameters
        ----------
        observation : array-like
            Angles and velocities in the swing which gets passed into PPO.
        action : array-like
            Requested leg and torso angles and speeds normalised between -1 and 1.
        Returns
        -------
        reward : float
            A score of how successfully the robot is swinging, PPO attempts to
            maximise this.
        """
        k_1, k_2, k_3, k_4 = 10, 1, 5, 5
        action = np.array([action[0]*63 + 32,
                       action[1]*190.3 + 190.3,
                       action[2]*32.5 + 5.5,
                       action[3]*190.3 + 190.3])
        top_angle, combined_joint_angle  = observation[2:4]
        reward = top_angle**2
        penalty = combined_joint_angle**2
        leg_velocity, torso_velocity = observation[4:6]
        lv_action = np.sign(observation[0] - action[0])*action[1]
        tv_action = np.sign(observation[1] - action[2])*action[3]
        norm_delta_v_leg = np.abs(leg_velocity - lv_action)/720
        norm_delta_v_torso = np.abs(torso_velocity - tv_action)/720
        # print(k_1*reward, k_2*penalty, k_3*norm_delta_v_leg, k_4*norm_delta_v_torso)
        return k_1*reward - k_2*penalty - k_3*norm_delta_v_leg - k_4*norm_delta_v_torso

    def get_info(self):
        """
        Does nothing right now. Ideally, this would output values which aren't
        necessarily known to PPO such as separate reward, penalty and effort
        constituents or potentially the current highest pivot angle attained.
        Returns
        -------
        info : dict
            Doesn't do anything yet.
        """
        return {"empty":None}

    def quit_timer(self):
        """
        Stops the program when enough time-steps have been done, otherwise
        increments time by 1.
        Returns
        -------
        done : boolean
            Stops the PPO algorithm when it is finished.
        """
        if self.run_time >= self.run_duration:
            return True
        self.run_time += 1
        return False


def get_events():
    """
    Listens to keystrokes for use with manual simulation.
    If the windows is closed its supposed to quit pygame,
    but this doesn't work right now.
    Returns
    -------
    keys_pressed : array-like
        A list of booleans representing which keys are being pressed.
    """
    get_event = pygame.event.get()
    for event in get_event:
        if event.type == pygame.QUIT:
            pygame.quit()
    return pygame.key.get_pressed()


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
    while True:
        keys_pressed = get_events()
        action = get_action(keys_pressed)

        # Step the simulation, then render the result (rendering in pymunk)
        environment.step(action)
        environment.render()


def ppo_main(filename="test_PPO_model_data", episodes = 3000, cores = os.cpu_count() - 1):
    """
    Runs the simulation using stable-baselines3 proximal policy optimisation
    algorithm.
    Episodes are run for a limited amount of time equal to env.run_duration,
    then the simulation is run for a set number of episodes.
    Parameters
    ----------
    filename : string
        DESCRIPTION. name of file
    episodes : int
        DESCRIPTION. number of episodes
    Returns
    -------
    None.
    """
    env = CustomEnv()
    if cores > 1:
        vecenv = make_vec_env(CustomEnv, n_envs=cores, vec_env_cls=SubprocVecEnv)
        model = PPO("MlpPolicy", vecenv, verbose=2, tensorboard_log="./tensorboard/")
    else:
        model = PPO("MlpPolicy", env, verbose=2, tensorboard_log="./tensorboard/")
    model.learn(total_timesteps= env.run_duration * episodes)
    model.save(filename)
    print("model saved\n---------------------------------------------------------")
    del model

    run_learned(filename)

def run_learned(filename = "test_PPO_model_data"):
    '''
    Runs the most recently learned PPO-model

    Parameters
    ----------
    filename - string
    
    Returns
    -------
    None.

    '''
    model = PPO.load(filename)
    print("model loaded\n---------------------------------------------------------")
    env = CustomEnv()
    obs = env.reset()
    print("initialising renderer")
    env.init_render()
    print("starting while loop (running the trained model)")
    while True:
        action, _states = model.predict(obs)
        print("action:",action)
        obs, rewards = env.step(action)[0:2]
        print("observation:",obs,"rewards:",rewards)
        env.render()

def continue_learning(filename = "test_PPO_model_data", episodes = 3000, cores = os.cpu_count() - 1):
    '''
    Allows you to continue learning with a model that some training has already been done with

    Parameters
    ----------
    filename : string
        DESCRIPTION. name of file
    episodes : int
        DESCRIPTION. number of episodes
    Returns
    -------
    None.

    '''
    env = CustomEnv()
    if cores > 1:
        vecenv = make_vec_env(CustomEnv, n_envs=cores, vec_env_cls=SubprocVecEnv)
        model = PPO.load(filename, vecenv, tensorboard_log="./tensorboard/")
    else:
        model = PPO.load(filename, env)
    model.learn(total_timesteps= env.run_duration * episodes, tb_log_name="PPO", reset_num_timesteps=False)
    model.save(filename)
    print("model saved\n---------------------------------------------------------")
    del model

def long_term_learning(run_length = 3600, filename = "test_PPO_model_data", episodes = 3000):
    """
    Calls the continue_learning() function until a set amount of time has passed.
    Parameters
    ----------
    time : int
        Time in seconds to run ML for.
    filename : string
        Name of file
    episodes : int
        Number of episodes
    Returns
    -------
    None.
    """
    start = time.time()
    dif = 0
    while dif < run_length:
        dif = time.time() - start
        continue_learning(filename, episodes)


if __name__ == "__main__":
    # main()
    #ppo_main()
    # run_learned()
    # continue_learning()

    long_term_learning(run_length=14400)

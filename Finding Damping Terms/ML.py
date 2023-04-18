# Import relevant libraries
import time
import csv
from Sim import setup_simulation, perform_action, get_action
from effort_parameter import get_effort
import pygame
import numpy as np
import gym
from gym import spaces
from pymunk.pygame_util import DrawOptions
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


# Create gym environment - Contains the machine learning code and the simulation code:
class CustomEnv(gym.Env):

    # Initialising the environment - SINGLE SETUP FUNCTION CALL to be written by simulations team:
    def __init__(self, stiffness_coefficient, damping_coefficient):
        self.run_time = 0
        self.reward = 0
        self.realtime_duration = 200
        # self.step_length = 1 / 100
        self.observation = np.zeros(12)
        # [-31, 0, -27, 0]    [95, 380.6, 38, 380.6]
        self.action_space = spaces.Box(np.array([-1, -1, -1, -1]),
                                       np.array([1, 1, 1, 1]),
                                       dtype=np.float32)
        self.observation_space = spaces.Box(np.array([-180, -180, -180, -180, -360, -360, -180, -180]),
                                            np.array([180, 180, 180, 180, 360, 360, 180, 180]),
                                            dtype=np.float32)

        self.simulation_data = setup_simulation(stiffness_coefficient, damping_coefficient)
        self.step_length = self.simulation_data["setup"]["step_length"]
        self.run_duration = self.realtime_duration / self.step_length
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
        self.simulation_data["pm_space"].step(self.step_length)
        # Potentially add random variation in step duration to help train for running on NAO

        observation = self.get_obs()

        # self.reward += self.get_reward(observation)
        info = self.get_info()
        done = self.quit_timer()
        self.observation = observation
        return observation, self.reward, done, info

    def init_render(self):
        """
        Initialises the renderer, is only called to visualise the simulation,
        it isn't necessary for the machine learning. (NOT RELEVANT TO SIMULATIONS)
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
        Renders the state of the simulation (NOT RELEVANT TO SIMULATIONS)
        Returns
        -------
        None.
        """
        get_events()
        self.window.fill((255, 255, 255))
        self.simulation_data["pm_space"].debug_draw(self.options)
        pygame.display.update()
        self.clock.tick(20)

    # Reset the simulation for the next training run (NOT RELEVANT TO SIMULATIONS)
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
        # self.simulation_data = setup_simulation(stiffness_coefficient, damping_coefficient)
        observation = self.get_obs()
        return observation

    def get_obs(self):  # could optimise by making array
        """
        Calculates leg, torso, top pivot and combined joint angles and velocities.
        Gets passed into observation array for PPO to use.
        Returns
        -------
        observation : array-like
            Angles and velocities in the swing which gets passed into PPO.
        """
        leg_angle = 180 / np.pi * (
                    self.simulation_data["pm_space"].bodies[3].angle - self.simulation_data["pm_space"].bodies[1].angle)
        leg_angle_velocity = 180 / np.pi * (self.simulation_data["pm_space"].bodies[3].angular_velocity -
                                            self.simulation_data["pm_space"].bodies[1].angular_velocity)
        # leg_angle_acc = (leg_angle_velocity - self.observation[4]) / self.step_length

        torso_angle = 180 / np.pi * (
                    self.simulation_data["pm_space"].bodies[2].angle - self.simulation_data["pm_space"].bodies[1].angle)
        torso_angle_velocity = 180 / np.pi * (self.simulation_data["pm_space"].bodies[2].angular_velocity -
                                              self.simulation_data["pm_space"].bodies[1].angular_velocity)
        # torso_angle_acc = (torso_angle_velocity - self.observation[5]) / self.step_length

        top_angle = 180 / np.pi * (
                    self.simulation_data["pm_space"].bodies[0].angle - self.simulation_data["setup"]["phi"])
        top_angle_velocity = 180 / np.pi * (self.simulation_data["pm_space"].bodies[0].angular_velocity)
        # top_angle_acc = (top_angle_velocity - self.observation[6]) / self.step_length

        combined_joint_angle = top_angle - 180 / np.pi * (self.simulation_data["pm_space"].bodies[1].angle)
        combined_joint_angle_velocity = 180 / np.pi * (self.simulation_data["pm_space"].bodies[0].angular_velocity -
                                                       self.simulation_data["pm_space"].bodies[1].angular_velocity)
        # combined_joint_angle_acc = (combined_joint_angle_velocity - self.observation[7]) / self.step_length

        observation = np.array([leg_angle, torso_angle, top_angle, combined_joint_angle,
                                leg_angle_velocity, torso_angle_velocity, top_angle_velocity,
                                combined_joint_angle_velocity])
        return observation

    def get_reward(self, observation):
        """
        Takes in observations and uses them to calculate a reward function.
        k_1 through k_7 are parameters used to tweak the importance of different
        behaviour in the neural network.
        Parameters
        ----------
        observation : array-like
            Angles and velocities in the swing which gets passed into PPO.
        Returns
        -------
        reward : float
            A score of how successfully the robot is swinging, PPO attempts to
            maximise this.
        """
        k_1, k_2, k_3, k_4, k_5, k_6, k_7 = 1, 0, 0, 0, 0, 0, 0
        top_angle, combined_joint_angle = observation[2:4]
        # leg_acc, torso_acc = observation[7:9]
        leg_acc, torso_acc = [0, 0]
        reward = top_angle * top_angle
        penalty = combined_joint_angle * combined_joint_angle
        effort = get_effort(self, leg_acc, torso_acc, k_4, k_5, k_6, k_7)
        # print(effort,reward,penalty)
        return k_1 * reward - k_2 * penalty - k_3 * effort

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
        return {"theta": 180 / np.pi * (
                    self.simulation_data["pm_space"].bodies[0].angle - self.simulation_data["setup"]["phi"]),
                "alpha": 180 / np.pi * (
                            self.simulation_data["pm_space"].bodies[1].angle - self.simulation_data["setup"][
                        "alpha"] - 8.328 * np.pi / 180
                            )}

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
    If the windows is closed should quit pygame,
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


def main(stiffness_coefficient, damping_coefficient):
    """
    Runs the simulation manually, no machine learning here.
    Instantiates the custom Gym environment, listens for keypresses
    then sets action based on input.
    Returns
    -------
    None.
    """
    # Initialise the simulation:
    environment = CustomEnv(stiffness_coefficient, damping_coefficient)
    # environment.init_render()
    # check_env(environment)
    # Run the simulation:
    angle_data = np.empty((int(environment.run_duration), 2), dtype=np.float32)
    seat_angle_data = np.empty((int(environment.run_duration), 2), dtype=np.float32)

    for i in range(int(environment.run_duration)):
        if i % int(environment.run_duration / 10) == 0:
            print("progress: ", round(100 * i / int(environment.run_duration), 0), "%")
        # keys_pressed = get_events()
        # action = get_action(keys_pressed)
        action = np.array([0, -1, 0, -1])
        # Step the simulation, then render the result (rendering in pymunk)
        environment.step(action)
        # environment.render()
        info = environment.get_info()
        angle_data[i] = np.array([i * environment.step_length, info["theta"]])
        seat_angle_data[i] = np.array([i * environment.step_length, info["alpha"]])

    with open(
            "large_angle_decay_SC_" + str(round(stiffness_coefficient, 2)) + "_DC_" + str(round(damping_coefficient, 2))
            + "" + ".csv", "w", newline="") as csv_container:
        writer = csv.writer(csv_container, delimiter=",")
        for data_point in angle_data:
            writer.writerow(data_point)
    with open(
            "small_angle_decay_SC_" + str(round(stiffness_coefficient, 2)) + "_DC_" + str(round(damping_coefficient, 2))
            + "" + ".csv", "w", newline="") as csv_container:
        writer = csv.writer(csv_container, delimiter=",")
        for data_point in seat_angle_data:
            writer.writerow(data_point)

    fit, _ = curve_fit(fitting_function, angle_data[:, 0], angle_data[:, 1])
    fit2, _ = curve_fit(fitting_function, seat_angle_data[:, 0], seat_angle_data[:, 1])
    print(fit, type(fit))
    print(fit2, type(fit2))

    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    # plt.suptitle("Amplitude vs Time, Stiffness:"+str(round(stiffness_coefficient, 2))+", Damping: "+str(round(damping_coefficient, 2)))
    plt.suptitle("Amplitude vs Time, Pivots with Damping: " + str(damping_coefficient))
    ax1.plot(angle_data[:, 0], angle_data[:, 1], label="Large Encoder Angle")
    ax2.plot(seat_angle_data[:, 0], seat_angle_data[:, 1], label="Small Encoder Angle")
    ax1.plot(angle_data[:, 0], fitting_function(angle_data[:, 0], fit[0], 0, fit[2]), label="Large Decay")
    ax2.plot(seat_angle_data[:, 0], fitting_function(seat_angle_data[:, 0], fit2[0], 0, fit2[2]), label="Small Decay")
    ax1.legend()
    ax2.legend()
    ax1.set_title("Upper Pivot: A = "+str(round(fit[0], 2))+", Decay const = "+str(round(fit[2], 5)))
    ax2.set_title("Lower Pivot: Damping: A = " + str(round(fit2[0], 2)) + ", Decay const = " + str(round(fit2[2], 5)))
    plt.xlabel("Time, s")
    plt.ylabel("Angle, degrees")
    plt.savefig(
        "plot_SC_" + str(round(stiffness_coefficient, 2)) + "_DC_" + str(round(damping_coefficient, 2)) + ".png",
        bbox_inches="tight", dpi=1200)
    plt.close(fig)

    del (angle_data)
    del (seat_angle_data)


def fitting_function(t, A, w, B):
    return A * np.cos(w * t) * np.exp(-t * B)


def ppo_main(filename, episodes=3000):
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
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=env.run_duration * episodes)
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
        print("action:", action)
        obs, rewards = env.step(action)[0:2]
        print("observation:", obs, "rewards:", rewards)
        env.render()


def run_learned(filename="test_PPO_model_data"):
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
    env = CustomEnv()
    obs = env.reset()
    env.init_render()
    while True:
        action, _states = model.predict(obs)
        print("action:", action)
        obs, rewards = env.step(action)[0:2]
        print("observation:", obs, "rewards:", rewards)
        env.render()


def continue_learning(filename="test_PPO_model_data", episodes=3000):
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
    model = PPO.load(filename, env)
    model.learn(total_timesteps=env.run_duration * episodes)
    model.save(filename)
    print("model saved\n---------------------------------------------------------")
    del model

    model = PPO.load(filename)
    print("model loaded\n---------------------------------------------------------")
    obs = env.reset()
    print("initialising renderer")
    env.init_render()
    print("starting while loop (running the trained model)")
    while True:
        action, _states = model.predict(obs)
        print("action:", action)
        obs, rewards = env.step(action)[0:2]

        print("observation:", obs, "rewards:", rewards)
        env.render()


def test_spring_dampness():
    test_damp_values = np.linspace(2266.5, 2266.5, 1)
    stiff_value = 0
    for test_damp_value in test_damp_values:
        print("testing:" + str(np.where(test_damp_values == test_damp_value)[0][0] + 1) + "/" + str(
            len(test_damp_values)))
        print("testing with damping coefficient:", round(test_damp_value, 3))

        main(stiff_value, test_damp_value)


def test_spring_stiffness():
    test_stiff_values = np.logspace(0, 5, 5)
    damp_value = 1922
    for test_stiff_value in test_stiff_values:
        print("testing:" + str(np.where(test_stiff_values == test_stiff_value)[0][0] + 1) + "/" + str(
            len(test_stiff_values)))
        print("testing with stiffness coefficient:", round(test_stiff_value, 3))

        main(test_stiff_value, damp_value)


if __name__ == "__main__":
    # main()
    # ppo_main()
    # run_learned()
    # continue_learning()
    # test_spring_stiffness()
    test_spring_dampness()

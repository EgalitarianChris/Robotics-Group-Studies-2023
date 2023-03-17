"""
Tests an alternative approach to the problem using the
A2C machine learning algorithm.

By outputting tensorboard data, any potential usefulness
of the algorithm can be shown.

Functions in this module:
    - a2c_test() tests A2C algorithm in the custom gym
    environment
"""
import os
from ML import CustomEnv
from stable_baselines3 import A2C
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

def a2c_test(episodes = 1000, filename = "a2c_test"):
    """
    Tests the A2C machine learning algorithm, saves output
    model and tensorboard data.

    Parameters
    ----------
        episodes : int
            Number of episodes to test for
        filename : str
            Name of file to save to
    
    Returns
    -------
        filename : str
            Returns the same filename passed in
    """
    env = CustomEnv()
    vecenv = make_vec_env(CustomEnv, n_envs=os.cpu_count() - 1, vec_env_cls=SubprocVecEnv)
    model = A2C("MlpPolicy", vecenv, verbose=1, tensorboard_log="./tensorboard/")
    model.learn(total_timesteps= env.run_duration * episodes)
    model.save(filename)
    return filename

if __name__ == "__main__":
    a2c_test()

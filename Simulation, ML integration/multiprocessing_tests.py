"""
Multiprocessing tests for the machine learning team.
Imports the custom gym class for manipulation here.
To see actual robot swinging simulation + machine learning
refer to ML.py instead.
"""
import time
import os
from ML import CustomEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv
import matplotlib.pyplot as plt

def multiproc_tests(episodes = 1000, cores = os.cpu_count() - 1):
    """
    Tests the timings of multiprocessing to prove advantage gained
    from parallelising the problem.

    Parameters
    ----------
    episodes : int
        Time in seconds to run ML for
    cores : int
        The maximum number of cores to test until (default is
        one less than all cores on your pc)

    Returns
    -------
    timings : tuple
        First element is a list from 1 to the number of cores
        used, second element is the corresponding time to
        compute for each
    """
    env = CustomEnv()
    times = []
    print(f"Beginning multiprocessing timing tests using {cores} cores")
    for i in range(1, cores+1):
        if i == 1:
            model = PPO("MlpPolicy", env, verbose=0)
        else:
            vecenv = make_vec_env(CustomEnv, n_envs=i, vec_env_cls=SubprocVecEnv)
            model = PPO("MlpPolicy", vecenv, verbose=0)

        start = time.time()
        model.learn(total_timesteps= env.run_duration * episodes)
        times.append(time.time() - start)
        del model
        print(f"Finished run {i}/{cores}")
        print(times)
    return list(range(1, cores+1)), times

def plotting_times(x_coord, y_coord):
    """
    Plots the timings passed from multiproc_tests().

    Parameters
    ----------
    x : list
        List of cores used
    y : list
        Corresponding time taken to compute

    Returns
    -------
    None
    """
    plt.plot(x_coord, y_coord)
    plt.title("Time to Complete Learning Against Number of Utilized Cores")
    plt.xlabel("Cores used")
    plt.xticks(x_coord)
    plt.ylabel("Time (s)")
    plt.show()

if __name__ == "__main__":
    coreList, procTimes = multiproc_tests()
    plotting_times(coreList, procTimes)
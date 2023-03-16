from ML import CustomEnv, run_learned
from stable_baselines3 import A2C
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv
import os

def a2c_test(episodes = 1000, filename = "a2c_test"):
    env = CustomEnv()
    vecenv = make_vec_env(CustomEnv, n_envs=os.cpu_count() - 1, vec_env_cls=SubprocVecEnv)
    model = A2C("MlpPolicy", vecenv, verbose=1, tensorboard_log="./tensorboard/")
    model.learn(total_timesteps= env.run_duration * episodes)
    model.save(filename)
    return filename

if __name__ == "__main__":
    a2c_test()
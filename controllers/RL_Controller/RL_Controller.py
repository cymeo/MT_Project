import From_Webot as FW
from Environment import WeBot_environment as W_Env

# for training
from stable_baselines3 import PPO   
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv


#for plotting 
import matplotlib.pyplot as plt
import numpy as np

# env = W_Env()
# #check_env(env, warn = False)
# env = Monitor(env, filename=f"monitor_logs/env_00") 
# env = DummyVecEnv([lambda: env])
# model = PPO(policy = "MultiInputPolicy", env= env, n_steps=2048)                                 

    
# for episode in range(20):
#     done = False
#     obs = env.reset()
#     while not done: 
#         action,_ = model.predict(obs, deterministic=False)
#         print(env.step(action))
#         obs, reward, done, truncated = env.step(action)
#         #print(f"Step Return: obs={obs}, reward={reward}, done={done}, truncated={truncated}, info={info}")


#         model.learn(total_timesteps= 1 , reset_num_timesteps=False)
#         if episode %2 == 0 :
#             print("episode:", episode)
            
# model.save("ppo0")

for i in range(10):
    print("FW.get_dist", FW.get_dist())
    
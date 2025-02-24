import From_Webot as FW
from Environment import WeBot_environment as W_Env
import torch
print(torch.cuda.is_available())  # Should return True if GPU is available
print(torch.cuda.get_device_name(0))  # Show GPU name if available

# for training
from stable_baselines3 import PPO   
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv
import pandas as pd 
import numpy as np

env = W_Env()
#check_env(env, warn = False)
#print("environment checked")
env = Monitor(env, filename=f"monitor_logs/env_03_41") 
env = DummyVecEnv([lambda: env])


model = PPO.load('ppo3_4')
model.set_env(env)  # Set environment

# model = PPO(
#     policy = "MultiInputPolicy", 
#     env= env, 
#     device="cuda",
#     batch_size=1024,
#     learning_rate= 3e-4,  
#     n_steps= 2048,
#     )


print("model loaded")
obs = env.reset()
steps = 30
episodes = 40000
model.learn(total_timesteps= episodes*steps, tb_log_name= "PPO_log3_41")   
model.save("ppo3_4")

print('start test')
successed = []
steps_per_Episodes = []
for episode in range(100):
    done = False
    steps = 0 
    while True: 
        action,_ = model.predict(obs, deterministic=False)
        obs, reward, done, truncated = env.step(action)        
        if reward >= 0: 
            successed.append(1)  
            
            break
        if done: 
            successed.append(0)
            break
        steps +=1
    obs = env.reset()
    steps_per_Episodes.append(steps)
    print(steps, ' steps in episode ',episode)   
    
print(np.sum(successed), "goal reached, ", "average steps: ", np.mean(steps_per_Episodes) )
DF = pd.DataFrame({
    "success": successed,
    "steps":np.array(steps_per_Episodes)
    })
DF.to_csv("test_results3_41")


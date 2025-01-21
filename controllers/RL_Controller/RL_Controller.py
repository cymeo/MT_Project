import From_Webot as FW
from Environment import WeBot_environment as W_Env

# for training
from stable_baselines3 import PPO   
from stable_baselines3 import SAC 
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv


env = W_Env()
check_env(env, warn = False)
print("environment checked")


env = Monitor(env, filename=f"monitor_logs/env_00") 
env = DummyVecEnv([lambda: env])
model = SAC(policy = "MultiInputPolicy", env= env)                                 


obs = env.reset()
# for episode in range(5000):
#     print ('episode: ', episode)
#     done = False
#     steps = 0
#     while not done:
         
#         action,_ = model.predict(obs, deterministic=False)
#         #print(env.step(action))
#         obs, reward, done, truncated = env.step(action)
#         steps +=1
#     obs = env.reset()
#     print(steps, ' steps in episode ',episode)    
steps = 200
episodes = 1000
model.learn(total_timesteps= episodes*steps, tb_log_name= "SAC_log")    

model.save("ppo0")

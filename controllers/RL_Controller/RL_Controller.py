import From_Webot as FW
from Environment import WeBot_environment as W_Env

# for training
from stable_baselines3 import PPO   
from stable_baselines3 import SAC 
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv

env = W_Env()
#check_env(env, warn = False)
#print("environment checked")

env = Monitor(env, filename=f"monitor_logs/env_00") 
env = DummyVecEnv([lambda: env])
#model = SAC(policy = "MultiInputPolicy", env= env)                                 
#model= SAC("SAC0.zip")
model = PPO(policy = "MultiInputPolicy", env= env, n_steps= 512 )
print("model loaded")
obs = env.reset()
 
steps = 500
episodes = 5000
model.learn(total_timesteps= episodes*steps, tb_log_name= "SAC_log")   
model.save("ppo0")

print('start test')
successnumber = 0
for episode in range(100):
    done = False
    steps = 0
    while True: 
        action,_ = model.predict(obs, deterministic=False)
        #print(env.step(action))
        obs, reward, done, truncated = env.step(action)        
        if reward >= 0: 
            successnumber +=1  
            break
        if done: 
            break
        steps +=1
    obs = env.reset()
    print(steps, ' steps in episode ',episode)   
print(successnumber, "goal reached, successrate= ", successnumber/100 ) 





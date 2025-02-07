import From_Webot3 as FW
from Environment3 import WeBot_environment as W_Env
import torch
print(torch.cuda.is_available())  # Should return True if GPU is available
print(torch.cuda.get_device_name(0))  # Show GPU name if available

# for training
from stable_baselines3 import PPO   
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv


env = W_Env()
#check_env(env, warn = False)
#print("environment checked")
env = Monitor(env, filename=f"env_03") 
env = DummyVecEnv([lambda: env])

#model = SAC(policy = "MultiInputPolicy", env= env)                                 
#model= SAC("SAC0.zip")
print("PPO on cuda")
model = PPO(
    policy = "MultiInputPolicy", 
    env= env, 
    device="cuda",
    batch_size=1024,
    learning_rate= 3e-4,  
    n_steps= 2048,
    )

#model = PPO.load('ppo_RP300')
#print("model loaded")
obs = env.reset()
 
steps = 350
episodes = 5000
model.learn(total_timesteps= episodes*steps, tb_log_name= "PPO_log2")   
model.save("ppo2")

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





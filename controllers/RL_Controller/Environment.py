from controller import Robot, Supervisor, Connector
from ikpy.chain import Chain

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class WeBot_environment(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self):
        super().__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        self.action_space = spaces.Box(low= -np.pi/2, high = np.pi/2, shape = (6,))
        self.observation_space = spaces.Dict(
            {
            
                "P_End": spaces.Box(
                    low = np.array([-100,-100,-100,-np.pi,-np.pi,-np.pi]),
                    high =np.array([100,100,100,np.py,np.py,np.py]), 
                    dtype = float
                    ), 
                "theta": spaces.Box(
                        low = np.array([-np.pi, -np.pi, -np.pi,-np.pi, -np.pi, -np.pi]),
                        high =np.array([np.pi, np.pi, np.pi,np.pi, np.pi, np.pi]),
                        dtype = float
                    ),
                "P_Goal":spaces.Box(
                    low = np.array([-100,-100,-100,-np.pi,-np.pi,-np.pi]),
                    high =np.array([100,100,100,np.py,np.py,np.py]), 
                    dtype = float    
                ),
                "P_Arm":spaces.Box(
                    low = np.array([-100,-100,-100,-np.pi,-np.pi,-np.pi]),
                    high =np.array([100,100,100,np.py,np.py,np.py]), 
                    dtype = float    
                ),
                "V_Arm":spaces.Box( # in m/sec
                    low = np.array([-1,-1,-1,-np.pi,-np.pi,-np.pi]),
                    high =np.array([1,1,1,np.py,np.py,np.py]), 
                    dtype = float    
                ), 
                "D_Arm2Link":spaces.Box(
                    low = np.array([-200,-200,-200,-np.pi,-np.pi,-np.pi]),
                    high =np.array([200,200,200,np.py,np.py,np.py]), 
                    dtype = float    
                )                                                                 
            }
        )



    def get_observation(self):
         #### todos: 
         # get motor,angles 
         # get Inversed kinematics 
         # get goal pose
         # get Arm object 
         # get arm object velocity 
         # get Distance from each link to arm 
         
        return observation

    def step(self, action):
        
        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        ...
        return observation, info

    def render(self):
        ...

    def close(self):
        ...
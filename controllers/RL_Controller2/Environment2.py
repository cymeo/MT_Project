import From_Webot2 as FW
from gymnasium import Env
import numpy as np
from gymnasium import spaces
from scipy.spatial.transform import Rotation as R
import time 

class WeBot_environment(Env):
    """Custom Environment that follows gym interface."""

    def __init__(self):
                
        super().__init__()
        ###learning parameters#######
        # rewards for -distance to goal,-rotational_distance, success, -max_steps, crash ###
        self.weights = np.array([1,0.0,500,500]) 
        self.max_step = 100
        
        self.crashed = False
        self.done = False      
        self.current_step = 0        
                               
        #### addistional params: 
        self.p_ee = np.array([])
        self.ee_pose = np.array([]) # transfrom matrix

        # action space: pose steps   
        self.action_space = spaces.Box(low= -np.pi/10, high =np.pi/10, shape = (6,))        
        self.observation_space = spaces.Dict(
            {
                "pose_ee": spaces.Box( # end effector pose
                    low = np.array([-1.2,-1.2,-1.2, -1,-1,-1,-1]),
                    high =np.array([1.2,1.2,1.2, 1,1,1,1]), 
                    dtype = float
                    ), 
                "v_ee": spaces.Box( # end effector velocity 
                    low = np.array([-1,-1,-1]),
                    high = np.array([1,1,1]),
                    dtype = float 
                ),               
                "theta": spaces.Box( # robot joint poses
                        low = np.array([-2*np.pi, -2*np.pi, -2*np.pi,-2*np.pi, -2*np.pi, -2*np.pi]),
                        high =np.array([2*np.pi, 2*np.pi, 2*np.pi,2*np.pi, 2*np.pi, 2*np.pi]),
                        dtype = float
                    ),
                "goal":spaces.Box( # goal position 
                    low = np.array([-1,-1,-1]),
                    high =np.array([1,1,1]), 
                    dtype = float    
                ),
                "d_goal_abs": spaces.Box(low = 0,high = 2, dtype=float), # absolute distance to goal 
                "d_goal": spaces.Box( # distance to goal 
                    low = np.array([-2,-2,-2]),
                    high =np.array([2,2,2]), 
                    dtype = float
                    ),
                "p_arm": spaces.Box( # position of arm 
                    low = np.array([0,0,0]),
                    high =np.array([2,2,2]), 
                    dtype = float
                    ),  
                "d_arm_abs": spaces.Box(low = 0,high = 2, dtype=float), # absolute distance to arm 
                "d_arm": spaces.Box( # relative distance to arm
                    low = np.array([-2,-2,-2]),
                    high =np.array([2,2,2]), 
                    dtype = float
                    ),                                    
                "vel_limit": spaces.Box( # maximal allowed distance 
                    low = np.array([-1,-1,-1]),
                    high = np.array([1,1,1]), 
                    dtype = float                     
                ),  
                "time": spaces.Box( # in sec 
                    low = 0,
                    high = 1000,
                    dtype = float 
                )
            }
        )


    #def get_observation(self):   # returns observation dict  
     #   return FW.get_observation
   
   
    def check_done(self):     #returne done and if successed 
        #sucess
        if (self.observation_space["d_goal_abs"] <= 0.05):
            success = True
            print('success')
            return True, True
        # step mlimit reached           
        if (self.current_step >= self.max_step):
            print("too many steps") 
            return True, False
        if self.crashed: 
            print("Crash")
            return True, False
        return False, False
        
    def get_reward(self): 
        R_success = 0 
        R_fail = 0 
        R_dist= 0 ## distance to goal
        R_rot_dist = 0 
        self.done, success  = self.check_done()
        R_dist, R_rot_dist = FW.get_distance
        if success: 
           R_success = 1  
        if self.crashed: 
            R_fail = 1
 
        total_reward = (
            -self.weights[0]*R_dist + 
            -self.weights[1]*R_rot_dist + 
            self.weights[2]*R_success + 
            -self.weights[3]*R_fail) 
            #- self.weights[4]*R_crash)   
        #print("reward", total_reward)               
        return total_reward

    def step(self, action):
        #count steps
        self.current_step += 1 
    
        # move robot        
        new_theta = np.clip((self.theta+action), -2*np.pi, 2*np.pi)
        new_theta[2] = np.clip(new_theta[2], -np.pi, np.pi)
        new_theta[1] = np.clip(new_theta[1],-np.pi/2,0)
        self.crashed = FW.move_robot(new_theta) 
        
        # get observation and reward
        observation = FW.get_observation()
        reward = self.get_reward()

        truncated = False
        info = {}

        return observation, reward, self.done, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        
        if self.crashed: 
            FW.reset_sim()
        
        #time.sleep(0.1)
        # new goal_pos
        rand_x = np.random.uniform(0.3, 0.7) 
        rand_y = np.random.uniform(-0.5, 0.5)
        rand_z = np.random.uniform(0.05,0.5)    
        self.goal = np.array([rand_x,rand_y,rand_z])
        #self.goal = np.array([0.5,0.1, 0.2 ])    
        FW.show_goal(self.goal)
    
        # reset parameters    
        self.crashed = False        
        self.done = False 
        self.current_step = 0
        FW.get_Arm()
        FW.get_motor_info()
        FW.get_robot_info()
        observation = FW.get_observation()
        info = {}
        
        return observation, info

    def render(self):
        pass

    def close(self):
        pass
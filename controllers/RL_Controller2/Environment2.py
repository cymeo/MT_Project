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
        self.reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])
         
        # observation parameters
        self.p_ee = np.array([])
        self.v_ee = np.array([])
        self.theta = np.zeros(6)
        self.goal = np.array([0.3,0.2,0.3])
        self.dist = 1 # dist to goal 
        self.p_arm = np.array([])
        self.d_arm = np.array([])       
        self.vel_limit = np.array([])
        self.time = 0 
        
        # action space: pose steps   
        self.action_space = spaces.Box(low= -np.pi/10, high =np.pi/10, shape = (6))        
        self.observation_space = spaces.Dict(
            {
                "p_ee": spaces.Box( # end effector pose
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
                "d_goal": spaces.Box(0,2,dtype=float), # absolute distance to goal 
                "d_goal_rel": spaces.Box( # distance to goal 
                    low = np.array([-2,-2,-2]),
                    high =np.array([2,2,2]), 
                    dtype = float
                    ),
                "p_arm": spaces.Box( # position of arm 
                    low = np.array([0,0,0]),
                    high =np.array([2,2,2]), 
                    dtype = float
                    ),  
                "d_arm": spaces.Box(0,2,dtype=float), # absolute distance to arm 
                "d_arm_rel": spaces.Box( # relative distance to arm
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

    def get_observation(self):
        #### todos: 
        # get motor,angles theta
        self.theta, self.p_ee, self.v_ee = FW.get_robot_info()
        
        transl = np.array(self.p_ee[:3,3])
        rot= R.from_matrix(self.p_ee[:3,:3])
        rot_quat= np.array(rot.as_quat())
        self.p_ee = np.concatenate((transl,rot_quat))
        self.dist, self.rot_dist = self.get_distance()
        self.p_arm  = FW.get_Arm()
        self.d_arm = np.linalg.norm(self.p_arm - self.p_ee[:3])

        observation = { 
            "v_ee": self.v_ee,
            "p_ee": self.p_ee, 
            "theta": self.theta,
            "goal": self.goal,
            "d_goal": self.dist,
            "d_goal_rel": np.subtract(self.goal, self.p_ee[:3]),
            "p_arm": self.p_arm,
            "d_arm": self.d_arm,
            "d_arm_rel": np.subtract(self.p_arm, self.p_ee[:3]),
            "vel_limit": self.vel_limit,
            "time": self.time
        }
        return observation

    def get_distance(self):
        dist = np.linalg.norm(self.goal - self.p_ee[:3])
        goal_rot = np.array([0,-1,0])
        ee_rot_quat = R.from_quat(self.p_ee[3:])
        ee_rot= ee_rot_quat.as_matrix()
        # rot_dist = np.arccos((np.trace(np.dot(goal_rot.T,ee_rot))-1)/2)
         #Compute the cosine similarity (dot product)
        y_axis = ee_rot[:, 1]  # Assuming 3x3 rotation matrix
        y_down = np.array([0,-1,0])  # downward direction for y 
        rot_dist = np.dot(y_axis, y_down) / (np.linalg.norm(y_axis) * np.linalg.norm(y_down))
        return dist, np.absolute(rot_dist)
    
    #return maximal allowed velocity according to ssm
    def get_max_vel(self): 
        min_dist = self.d_arm
        
   
   #returne done and if successed 
    def check_done(self):     

        #sucess
        if (self.dist <= 0.05):
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
        R_dist = self.dist     
        R_rot_dist = self.rot_dist
    
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

        self.current_step += 1 
        new_theta = np.clip((self.theta+action), -2*np.pi, 2*np.pi)
        new_theta[2] = np.clip(new_theta[2], -np.pi, np.pi)
        new_theta[1] = np.clip(new_theta[1],-np.pi/2,0)
        #print("new_theta",new_theta)
        self.theta, self.crashed = FW.move_robot(new_theta) 
        
        observation = self.get_observation()
        #self.done, _ = self.check_done()
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
        observation = self.get_observation()
        info = {}
        
        return observation, info

    def render(self):
        pass

    def close(self):
        pass
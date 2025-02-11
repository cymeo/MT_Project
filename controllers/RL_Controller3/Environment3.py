import From_Webot3 as FW
import gymnasium as gym
from gymnasium import Env
import numpy as np
from gymnasium import spaces
from scipy.spatial.transform import Rotation as R
import time


class WeBot_environment(Env):
    """Custom Environment that follows gym interface."""

    def __init__(self):
        
        ######### rewards for -distance to goal,-rotational_distance, success, -max_steps, crash ############
        self.weights = np.array([2,0.0,400,400]) 
        
        super().__init__()
        # Define action and observation space
        #action = Motorangle steps  
        self.action_space = spaces.Box(low= -np.pi/10, high = np.pi/10, shape = (6,))
        #observation = Endeffector pose, motor angles, Goal Pose, 
        self.reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])
        self.observation_space = spaces.Dict(
            {
                "goal":spaces.Box(
                    low = np.array([-1,-1,-1]),
                    high =np.array([1,1,1]), 
                    dtype = float    
                ),
            
                "p_end": spaces.Box(
                    low = np.array([-1.2,-1.2,-1.2, -1,-1,-1,-1]),
                    high =np.array([1.2,1.2,1.2, 1,1,1,1]), 
                    dtype = float
                    ), 
                "theta": spaces.Box(
                        low = np.array([-2*np.pi, -2*np.pi, -2*np.pi,-2*np.pi, -2*np.pi, -2*np.pi]),
                        high =np.array([2*np.pi, 2*np.pi, 2*np.pi,2*np.pi, 2*np.pi, 2*np.pi]),
                        dtype = float
                    ),
                #"stepnumber": spaces.Box(low = 0 , high = 500, dtype= int),
                "d_goal": spaces.Box(0,2,dtype=float),
                "d_goal_rel": spaces.Box(
                    low = np.array([-2,-2,-2]),
                    high =np.array([2,2,2]), 
                    dtype = float
                    )                                                    
            }
        )

        self.action = np.zeros(6)
        self.timestep = 0
        self.max_step = 1000
        
        
        #further Parameters
        self.current_step = 0
        self.theta = np.zeros(6)
        self.goal = np.array([0.3,0.2,0.3])
        self.dist = 1
        self.prev_dist = 1 
        
        self.crashed = False
        self.done = False

    def get_observation(self):
        #### todos: 
        # get motor,angles theta
        self.theta = FW.get_motor_pos()
        # get foreward kinematics 
        p_end = FW.get_forward_kinematics(self.theta)
        transl = np.array(p_end[:3,3])
        rot= R.from_matrix(p_end[:3,:3])
        rot_quat= np.array(rot.as_quat())
        #print("transl: ", transl, "   rot: ", rot_quat)
        self.p_end = np.concatenate((transl,rot_quat))
        self.dist, self.rot_dist = self.get_distance()

        observation = { 
            "goal": self.goal,
            "p_end": self.p_end, 
            "theta": self.theta, 
            #"stepnumber": self.stepnumber,
            "d_goal": self.dist,
            "d_goal_rel": np.subtract(self.goal, self.p_end[:3])
        }
        
        return observation

    def get_distance(self):
        dist = np.linalg.norm(self.goal - self.p_end[:3])
        goal_rot = np.array([0,-1,0])
        ee_rot_quat = R.from_quat(self.p_end[3:])
        ee_rot= ee_rot_quat.as_matrix()
        # rot_dist = np.arccos((np.trace(np.dot(goal_rot.T,ee_rot))-1)/2)
         #Compute the cosine similarity (dot product)
        y_axis = ee_rot[:, 1]  # Assuming 3x3 rotation matrix
        y_down = np.array([0,-1,0])  # downward direction for y 
        rot_dist = np.dot(y_axis, y_down) / (np.linalg.norm(y_axis) * np.linalg.norm(y_down))
        return dist, np.absolute(rot_dist)
   
   #returne done and if successed 
    def check_done(self):     

        #sucess
        if (self.dist <= 0.05):
            success = True
            print(success)
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
        self.done, _ = self.check_done()
        reward = self.get_reward()

        truncated = False
        info = {}

        return observation, reward, self.done, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        FW.reset_sim()
        #time.sleep(0.1)
        # new goal_pos
        rand_x = np.random.uniform(0.1, 0.6) 
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
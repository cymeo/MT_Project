import From_Webot as FW
import gymnasium as gym
from gymnasium import Env
import numpy as np
from gymnasium import spaces
from scipy.spatial.transform import Rotation as R

class WeBot_environment(Env):
    """Custom Environment that follows gym interface."""

    def __init__(self):
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
                    )
                
                #,
                # "P_Arm":spaces.Box(
                #     low = np.array([-100,-100,-100,-np.pi,-np.pi,-np.pi]),
                #     high =np.array([100,100,100,np.py,np.py,np.py]), 
                #     dtype = float    
                # ),
                # "V_Arm":spaces.Box( # in m/sec
                #     low = np.array([-1,-1,-1,-np.pi,-np.pi,-np.pi]),
                #     high =np.array([1,1,1,np.py,np.py,np.py]), 
                #     dtype = float    
                # ), 
                # "D_Arm2Link":spaces.Box(
                #     low = np.array([-200,-200,-200,-np.pi,-np.pi,-np.pi]),
                #     high =np.array([200,200,200,np.py,np.py,np.py]), 
                #     dtype = float    
                # )                                                                 
            }
        )

        self.action = np.zeros(6)
        self.timestep = 0
        self.weights = np.array([0.8,0.2,500,300]) #rewards for -distance to goal,-rotational_distance, success, -max_steps/crash         
        
        #further Parameters
        self.current_step = 0
        self.theta = np.zeros(6)
        self.crashed = False
        self.goal = np.array([0.3,0.2,0.3])

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

        observation = { 
            "goal": self.goal,
            "p_end": self.p_end, 
            "theta": self.theta
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
        return dist, rot_dist
    
    def check_done(self):
        success =False 
        dist, rot_dist = self.get_distance()
        #sucess
        if (dist <= 0.05 and rot_dist <= np.pi/6):
            success = True
            return True, success
        # step mlimit reached           
        if (self.current_step >= 600):
            print("too many steps") 
            return True, success
        if self.crashed: 
            #print("Crash")
            return True, success
        
        return False, success
        
    def get_reward(self): 
        R_success = 0 
        R_fail = 0 
        R_dist= 0 ## distance to goal
        R_rot_dist = 0 
        done, success  = self.check_done()
        
        R_dist, R_rot_dist = self.get_distance()
    
        if success: 
           R_success = 1  
        if (done and (success == False)): 
            R_fail = 1
                       
        total_reward = (
            -self.weights[0]*R_dist + 
            -self.weights[1]*R_rot_dist + 
            self.weights[2]*R_success + 
            -self.weights[3]*R_fail )       
        return total_reward

    def step(self, action):
        
        self.current_step += 1 
        new_theta = np.clip((self.theta+action), -2*np.pi, 2*np.pi)
        new_theta[2] = np.clip(new_theta[2], -np.pi, np.pi)
        new_theta[1] = np.clip(new_theta[1],-np.pi,0)
        #print("new_theta",new_theta)
        self.theta, self.crashed = FW.move_robot(new_theta) 
        
        observation = self.get_observation()
        reward = self.get_reward()
        done, _ = self.check_done()
        truncated = False 
        info = {}
        
        # print("observation", observation)
        # print("reward", reward)
        # print("done",done)
        # print("trunc",truncated)
        # print("info" ,info)

        return observation, reward, done, truncated, info

    def reset(self, seed=None, options=None):
        print("reset")
        super().reset(seed=seed)
        #FW.reset_sim()     
        # new goal_pos
        rand_x = np.random.uniform(0.1, 0.8) 
        rand_y = np.random.uniform(-0.5, 0.5)
        rand_z = np.random.uniform(0,0.5)
        self.goal = np.array([rand_x,rand_y,rand_z])
        FW.show_goal(self.goal)
        FW.move_robot(self.reset_pose)
        self.crashed = False
        #rand_angles = np.random.uniform(-1.8 * np.pi, 1.8 * np.pi, 6)
        #self.goal = FW.get_forward_kinematics(rand_angles)
        
        
        self.current_step = 0
        observation = self.get_observation()
        info = {}
        return observation, info

    def render(self):
        pass

    def close(self):
        pass